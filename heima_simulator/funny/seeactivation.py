import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
import imageio
import os
from tqdm import tqdm


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
        
        def forward(self, obs, return_intermediates=False):
            # obs should be (batch_size, obs_dim) or (obs_dim,)
            was_1d = obs.ndim == 1
            if was_1d:
                obs = obs.unsqueeze(0)
            
            # Normalize
            x = self.obs_normalizer(obs)
            intermediates = {'normalized_obs': x.clone()}
            
            # MLP - manually step through to capture intermediate values
            current = x
            linear_idx = 0
            
            for i, layer in enumerate(self.mlp):
                current = layer(current)
                
                # Store output after each operation
                if isinstance(layer, nn.Linear):
                    intermediates[f'linear_{linear_idx}_out'] = current.clone()
                    linear_idx += 1
                elif isinstance(layer, nn.ELU):
                    intermediates[f'elu_{linear_idx - 1}_out'] = current.clone()
                else:
                    intermediates[f'layer_{i}_out'] = current.clone()
            
            output = current
            
            # Squeeze batch dimension if input was 1D
            if was_1d:
                output = output.squeeze(0)
                for key in intermediates:
                    if intermediates[key].ndim > 1 and intermediates[key].shape[0] == 1:
                        intermediates[key] = intermediates[key].squeeze(0)
            
            if return_intermediates:
                intermediates['final_output'] = output
                return intermediates
            else:
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


def make_activation_video(activation_list, output_path, video_name="activation_video", fps=10):
    """
    Create a video from a list of activation arrays, where each element becomes a frame.
    
    Args:
        activation_list: List of numpy arrays, each representing activations at a timestep
        output_path: Directory path where the video will be saved
        video_name: Name of the output video file (without extension)
        fps: Frames per second for the video
    """
    os.makedirs(os.path.join(output_path, "videos"), exist_ok=True)
    os.makedirs(os.path.join(output_path, "videos", "tmp"), exist_ok=True)
    
    # Convert to numpy array and compute consistent global color range across all frames
    activation_array = np.array(activation_list)
    vmin, vmax = activation_array.min(), activation_array.max()
    
    # Create a frame for each activation
    for idx, activation in enumerate(tqdm(activation_list, desc="Creating frames")):
        fig, ax = plt.subplots(figsize=(8, 6))
        data = np.array(activation)
        if data.ndim == 1:
            data = data.reshape(1, -1)
        im = ax.imshow(data, aspect="auto", vmin=vmin, vmax=vmax, cmap="viridis")
        fig.colorbar(im, ax=ax, fraction=0.03, pad=0.02)  # narrow bar, close to axes
        ax.set_title(f"step {idx}")
        fig.tight_layout()
        fig.savefig(os.path.join(output_path, "videos", "tmp", f"activation_{idx:05d}.png"), dpi=120)
        plt.close(fig)
    
    # Merge the images into a mp4 video using imageio + ffmpeg
    num_frames = len(activation_list)
    frames = [imageio.imread(os.path.join(output_path, "videos", "tmp", f"activation_{idx:05d}.png")) 
              for idx in range(num_frames)]
    imageio.mimwrite(os.path.join(output_path, "videos", f"{video_name}.mp4"), frames, fps=fps, codec="libx264")
    
    print(f"Video saved to: {os.path.join(output_path, 'videos', f'{video_name}.mp4')}")


def make_all_elu_video(elu_outputs_list, output_path, video_name="all_elu_outputs", fps=10):
    """
    Create a video from a list of dictionaries containing all ELU outputs, 
    where each frame shows all ELU outputs stacked vertically.
    
    Args:
        elu_outputs_list: List of dictionaries, each containing ELU outputs for one timestep
                          Keys should be like 'elu_0_out', 'elu_1_out', etc.
        output_path: Directory path where the video will be saved
        video_name: Name of the output video file (without extension)
        fps: Frames per second for the video
    """
    os.makedirs(os.path.join(output_path, "videos"), exist_ok=True)
    os.makedirs(os.path.join(output_path, "videos", "tmp"), exist_ok=True)
    
    # Get all ELU keys from the first timestep
    if len(elu_outputs_list) == 0:
        print("No ELU outputs to visualize")
        return
    
    elu_keys = sorted([k for k in elu_outputs_list[0].keys() if k.startswith('elu_')])
    if len(elu_keys) == 0:
        print("No ELU outputs found")
        return
    
    # Collect all values to compute global color range
    # Since different ELU layers may have different dimensions, we collect all values first
    all_values = []
    for elu_dict in elu_outputs_list:
        for key in elu_keys:
            val = np.array(elu_dict[key])
            all_values.append(val.flatten())
    
    # Concatenate all values and compute statistics
    all_values_flat = np.concatenate(all_values)
    vmin = all_values_flat.min()
    # Strip away top 1% when computing vmax (use 99th percentile)
    vmax = np.percentile(all_values_flat, 99)
    
    # Create a frame for each timestep with all ELU outputs stacked
    for idx, elu_dict in enumerate(tqdm(elu_outputs_list, desc="Creating frames")):
        num_elu = len(elu_keys)
        fig, axes = plt.subplots(num_elu, 1, figsize=(10, 2 * num_elu))
        
        # Handle single subplot case
        if num_elu == 1:
            axes = [axes]
        
        for ax_idx, elu_key in enumerate(elu_keys):
            data = np.array(elu_dict[elu_key])
            if data.ndim == 1:
                data = data.reshape(1, -1)
            
            im = axes[ax_idx].imshow(data, aspect="auto", vmin=vmin, vmax=vmax, cmap="viridis")
            axes[ax_idx].set_title(f"{elu_key} - step {idx}")
            axes[ax_idx].set_ylabel("Neuron")
            if ax_idx == len(elu_keys) - 1:
                axes[ax_idx].set_xlabel("Time")
            plt.colorbar(im, ax=axes[ax_idx], fraction=0.03, pad=0.02)
        
        fig.tight_layout()
        fig.savefig(os.path.join(output_path, "videos", "tmp", f"all_elu_{idx:05d}.png"), dpi=120)
        plt.close(fig)
    
    # Merge the images into a mp4 video
    num_frames = len(elu_outputs_list)
    frames = [imageio.imread(os.path.join(output_path, "videos", "tmp", f"all_elu_{idx:05d}.png")) 
              for idx in range(num_frames)]
    imageio.mimwrite(os.path.join(output_path, "videos", f"{video_name}.mp4"), frames, fps=fps, codec="libx264")
    
    print(f"Video saved to: {os.path.join(output_path, 'videos', f'{video_name}.mp4')}")


if __name__ == "__main__":
    model = load_pt_model("/home/yao/Desktop/repo/heima/mujoco_approach/mjlab/logs/rsl_rl/heima_armless_velocity/2026-02-23_15-55-40/model_15550.pt")
    obs = np.random.randn(10, 45)
    actions = model(torch.from_numpy(obs).float())
    print(actions.shape)
    print(actions)

    # read observations.csv and actions.csv
    import pandas as pd
    observations = pd.read_csv("/home/yao/Desktop/repo/heima/latest_deploy/HeimaFrameWork/heima_simulator/code/build/observations.csv", header=None)
    actions = pd.read_csv("/home/yao/Desktop/repo/heima/latest_deploy/HeimaFrameWork/heima_simulator/code/build/actions.csv", header=None)
    # calculated_actions = model(torch.from_numpy(observations.values).float())

    # calculated_actions_onnx = run_onnx("/home/yao/Desktop/repo/heima/mujoco_approach/mjlab/logs/rsl_rl/heima_armless_velocity/2026-02-23_15-55-40/model_15550.onnx", observations.values[0])

    # print(actions.values - calculated_actions.detach().numpy())
    # print(actions.values[0] - calculated_actions_onnx)

    # exit()
    # Collect all ELU outputs for each observation
    all_elu_outputs = []
    for obs in observations.values[100:]:
        calculated_actions = model(torch.from_numpy(obs).float(), return_intermediates=True)
        # Extract all ELU outputs and convert to numpy
        elu_dict = {}
        for key, value in calculated_actions.items():
            if key.startswith('elu_'):
                elu_dict[key] = value.detach().numpy()
        all_elu_outputs.append(elu_dict)

    # Create video with all ELU outputs in the same frame
    output_dir = "/home/yao/Desktop/repo/heima/latest_deploy/HeimaFrameWork/heima_simulator/funny/build"
    make_all_elu_video(all_elu_outputs, output_dir, video_name="all_elu_outputs", fps=10)
    
    # Optionally also plot heatmap
    # import matplotlib.pyplot as plt
    # plt.imshow(tmp, aspect="auto")
    # plt.colorbar()
    # plt.show()
    exit()
