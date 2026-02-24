import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn


def run_onnx(onnx_model, observation, actions=None):
    """
    Run ONNX model inference, matching the C++ API pattern: onnx.run(observation, actions)
    
    Args:
        onnx_model: onnxruntime.InferenceSession object or path to ONNX file
        observation: numpy array of shape (obs_dim,) or (1, obs_dim) containing observation
        actions: optional numpy array to store actions. If None, a new array is created.
    
    Returns:
        numpy array of actions, or None if inference fails
    """
    # If onnx_model is a string, load it
    if isinstance(onnx_model, str):
        session = ort.InferenceSession(onnx_model)
    else:
        session = onnx_model
    
    # Ensure observation is 2D (batch_size, obs_dim)
    obs = np.array(observation, dtype=np.float32)
    if obs.ndim == 1:
        obs = obs.reshape(1, -1)
    
    # Get input/output names
    input_name = session.get_inputs()[0].name
    output_name = session.get_outputs()[0].name
    
    # Run inference
    outputs = session.run([output_name], {input_name: obs})
    output = outputs[0]
    
    # Reshape to 1D if needed
    if output.ndim > 1:
        output = output.flatten()
    
    # If actions array provided, copy into it (matching C++ in-place behavior)
    if actions is not None:
        actions[:] = output[:len(actions)]
        return actions
    
    return output


def load_pt_model(pt_path, model_type='actor'):
    """
    Load PyTorch model checkpoint (.pt file) and reconstruct a callable nn.Module.
    
    Args:
        pt_path: Path to the .pt checkpoint file
        model_type: 'actor' or 'critic' - which model to load
    
    Returns:
        nn.Module: A callable PyTorch module that takes observation tensor and returns actions/values
    """
    checkpoint = torch.load(pt_path, map_location="cpu", weights_only=False)
    
    # Get state dict
    state_dict = checkpoint.get(f'{model_type}_state_dict')
    if state_dict is None:
        # Handle legacy format
        if 'model_state_dict' in checkpoint:
            model_state_dict = checkpoint['model_state_dict']
            state_dict = {}
            prefix = f"{model_type}."
            for key, value in model_state_dict.items():
                if key.startswith(prefix):
                    new_key = key.replace(prefix, "mlp." if not key.startswith(f"{prefix}obs_normalizer") else key.replace(f"{prefix}obs_normalizer", "obs_normalizer"))
                    state_dict[new_key] = value
                elif key.startswith(f"{model_type}_obs_normalizer."):
                    new_key = key.replace(f"{model_type}_obs_normalizer.", "obs_normalizer.")
                    state_dict[new_key] = value
                elif model_type == 'actor' and key in ["std", "log_std"]:
                    state_dict[key] = value
        else:
            raise ValueError(f"No {model_type}_state_dict found in checkpoint")
    
    # Infer architecture from state dict
    has_obs_normalizer = any('obs_normalizer' in k for k in state_dict.keys())
    has_std = 'std' in state_dict
    has_log_std = 'log_std' in state_dict
    is_stochastic = has_std or has_log_std
    
    # Get input dimension
    if has_obs_normalizer:
        # Get obs_normalizer shape - it's stored as [1, obs_dim] in checkpoint
        for k, v in state_dict.items():
            if 'obs_normalizer._mean' in k or 'obs_normalizer._var' in k:
                # Handle both [1, obs_dim] and [obs_dim] shapes
                if v.ndim == 2:
                    obs_dim = v.shape[1]
                else:
                    obs_dim = v.shape[-1] if v.ndim > 0 else v.item()
                break
    else:
        # Get from first MLP layer
        for k, v in state_dict.items():
            if 'mlp.0.weight' in k:
                obs_dim = v.shape[1]
                break
    
    # Get output dimension and hidden dims from MLP layers
    mlp_weights = [(k, v) for k, v in state_dict.items() if 'mlp.' in k and 'weight' in k]
    mlp_weights.sort(key=lambda x: int(x[0].split('.')[1]))
    
    hidden_dims = []
    for i, (k, v) in enumerate(mlp_weights):
        if i == 0:
            # First layer: input_dim -> hidden_dim
            hidden_dims.append(v.shape[0])
        elif i == len(mlp_weights) - 1:
            # Last layer: hidden_dim -> output_dim
            output_dim = v.shape[0]
        else:
            # Middle layers
            hidden_dims.append(v.shape[0])
    
    # Get activation from config or default to ELU
    activation = nn.ELU()
    
    # EmpiricalNormalization helper class
    # Note: buffers are stored as [1, obs_dim] in checkpoint to match rsl_rl format
    class EmpiricalNormalization(nn.Module):
        def __init__(self, shape):
            super().__init__()
            # Register buffers with shape [1, obs_dim] to match checkpoint format
            self.register_buffer("_mean", torch.zeros(1, shape))
            self.register_buffer("_var", torch.ones(1, shape))
            self.register_buffer("_std", torch.ones(1, shape))
            self.eps = 1e-2
        
        def forward(self, x):
            # x is (batch_size, obs_dim), _mean/_std are (1, obs_dim)
            return (x - self._mean) / (self._std + self.eps)
    
    # Reconstruct the model
    class SimpleActorModel(nn.Module):
        def __init__(self):
            super().__init__()
            
            # Observation normalizer
            if has_obs_normalizer:
                self.obs_normalizer = EmpiricalNormalization(obs_dim)
            else:
                self.obs_normalizer = nn.Identity()
            
            # MLP
            layers = []
            layers.append(nn.Linear(obs_dim, hidden_dims[0]))
            layers.append(activation)
            
            for i in range(len(hidden_dims) - 1):
                layers.append(nn.Linear(hidden_dims[i], hidden_dims[i + 1]))
                layers.append(activation)
            
            layers.append(nn.Linear(hidden_dims[-1], output_dim))
            self.mlp = nn.Sequential(*layers)
            
            # Stochastic parameters
            if has_std:
                self.std = nn.Parameter(torch.ones(output_dim))
            elif has_log_std:
                self.log_std = nn.Parameter(torch.zeros(output_dim))
        
        def forward(self, obs):
            # obs should be (batch_size, obs_dim) or (obs_dim,)
            if obs.ndim == 1:
                obs = obs.unsqueeze(0)
            
            # Normalize
            x = self.obs_normalizer(obs)
            
            # MLP
            output = self.mlp(x)
            
            # Squeeze batch dimension if input was 1D
            if obs.shape[0] == 1 and obs.ndim == 2:
                output = output.squeeze(0)
            
            return output
    
    # Create model and load state dict
    model = SimpleActorModel()
    
    # Filter state dict to only include keys that exist in model
    model_state_dict = {}
    for k, v in state_dict.items():
        # Map state dict keys to model keys
        if 'obs_normalizer' in k:
            model_state_dict[k] = v
        elif 'mlp.' in k:
            model_state_dict[k] = v
        elif k in ['std', 'log_std']:
            model_state_dict[k] = v
    
    model.load_state_dict(model_state_dict, strict=False)
    model.eval()
    
    return model


if __name__ == "__main__":
    model = load_pt_model("/home/yao/Desktop/repo/heima/mujoco_approach/mjlab/logs/rsl_rl/heima_armless_velocity/2026-02-23_15-55-40/model_15550.pt")
    obs = np.random.randn(10, 45)
    actions = model(torch.from_numpy(obs).float())
    print(actions.shape)
    print(actions)
