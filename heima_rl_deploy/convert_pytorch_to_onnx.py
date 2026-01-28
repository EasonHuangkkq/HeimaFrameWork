#!/usr/bin/env python3
"""
Convert PyTorch JIT model to ONNX format for deployment.

Usage:
    python convert_pytorch_to_onnx.py --input /path/to/policy_1.pt --output policy_heima_noarm.onnx
"""

import argparse
import torch
import torch.onnx
import os
import sys

def convert_pytorch_to_onnx(input_path, output_path, input_size=48, output_size=12):
    """
    Convert PyTorch JIT model to ONNX format.
    
    Args:
        input_path: Path to the PyTorch .pt file
        output_path: Path where the ONNX model will be saved
        input_size: Expected input observation size (default: 48 for heima_noarm)
        output_size: Expected output action size (default: 12 for heima_noarm)
    """
    print(f"Loading PyTorch model from: {input_path}")
    
    if not os.path.exists(input_path):
        raise FileNotFoundError(f"Model file not found: {input_path}")
    
    # Load the JIT scripted model
    try:
        model = torch.jit.load(input_path, map_location='cpu')
        model.eval()
        print("✓ Model loaded successfully")
    except Exception as e:
        raise RuntimeError(f"Failed to load model: {e}")
    
    # Create dummy input matching the expected observation size
    dummy_input = torch.zeros(1, input_size, dtype=torch.float32)
    
    print(f"Model input shape: {dummy_input.shape}")
    
    # Test forward pass
    try:
        with torch.no_grad():
            output = model(dummy_input)
        print(f"✓ Forward pass successful, output shape: {output.shape}")
        if output.shape[1] != output_size:
            print(f"Warning: Expected output size {output_size}, got {output.shape[1]}")
    except Exception as e:
        raise RuntimeError(f"Forward pass failed: {e}")
    
    # Export to ONNX
    print(f"\nExporting to ONNX format...")
    try:
        torch.onnx.export(
            model,                          # Model to export
            dummy_input,                   # Model input (dummy)
            output_path,                   # Output file path
            export_params=True,            # Store trained parameter weights
            opset_version=11,              # ONNX opset version
            do_constant_folding=True,      # Execute constant folding optimization
            input_names=['observations'],   # Input tensor name
            output_names=['actions'],      # Output tensor name
            verbose=False
        )
        print(f"✓ ONNX model exported successfully to: {output_path}")
    except Exception as e:
        raise RuntimeError(f"ONNX export failed: {e}")
    
    # Verify the exported ONNX model
    try:
        import onnx
        onnx_model = onnx.load(output_path)
        onnx.checker.check_model(onnx_model)
        print("✓ ONNX model verification passed")
        
        # Print model info
        print("\nModel Information:")
        print(f"  Input: {onnx_model.graph.input[0].name}")
        print(f"  Output: {onnx_model.graph.output[0].name}")
        
        # Get input/output shapes
        input_shape = [dim.dim_value if dim.dim_value > 0 else 'dynamic' 
                      for dim in onnx_model.graph.input[0].type.tensor_type.shape.dim]
        output_shape = [dim.dim_value if dim.dim_value > 0 else 'dynamic'
                       for dim in onnx_model.graph.output[0].type.tensor_type.shape.dim]
        print(f"  Input shape: {input_shape}")
        print(f"  Output shape: {output_shape}")
        
    except ImportError:
        print("Warning: onnx package not installed, skipping verification")
        print("  Install with: pip install onnx")
    except Exception as e:
        print(f"Warning: ONNX verification failed: {e}")
    
    print(f"\n✓ Conversion complete!")
    print(f"  Input: {input_path}")
    print(f"  Output: {output_path}")

def main():
    parser = argparse.ArgumentParser(
        description='Convert PyTorch JIT model to ONNX format for deployment'
    )
    parser.add_argument(
        '--input', '-i',
        type=str,
        required=True,
        help='Path to input PyTorch .pt file'
    )
    parser.add_argument(
        '--output', '-o',
        type=str,
        default=None,
        help='Path to output ONNX file (default: same name as input with .onnx extension)'
    )
    parser.add_argument(
        '--input-size',
        type=int,
        default=45,
        help='Expected input observation size (default: 48 for heima_noarm)'
    )
    parser.add_argument(
        '--output-size',
        type=int,
        default=12,
        help='Expected output action size (default: 12 for heima_noarm)'
    )
    
    args = parser.parse_args()
    
    # Determine output path
    if args.output is None:
        base_name = os.path.splitext(os.path.basename(args.input))[0]
        output_dir = os.path.dirname(args.input)
        args.output = os.path.join(output_dir, f"{base_name}.onnx")
    
    # Create output directory if needed
    os.makedirs(os.path.dirname(args.output) if os.path.dirname(args.output) else '.', exist_ok=True)
    
    try:
        convert_pytorch_to_onnx(
            args.input,
            args.output,
            args.input_size,
            args.output_size
        )
    except Exception as e:
        print(f"\n✗ Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()

