/*
 * Simple ONNX Runtime wrapper implementation
 */

#include "onnx_wrapper.h"
#include "onnxruntime/onnxruntime_cxx_api.h"
#include <iostream>
#include <stdexcept>
#include <cstring>

OnnxWrapper::OnnxWrapper() 
    : initialized_(false), input_size_(0), output_size_(0) {
}

OnnxWrapper::~OnnxWrapper() {
    // Smart pointers will handle cleanup
}

bool OnnxWrapper::init(const std::string& onnx_file) {
    try {
        // Create ONNX Runtime environment
        env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "heima_rl_deploy");
        
        // Create session options
        Ort::SessionOptions session_options;
        session_options.SetInterOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(
            GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
        
        // Create session
        session_ = std::make_unique<Ort::Session>(*env_, onnx_file.c_str(), session_options);
        
        // Get input/output names and shapes
        Ort::AllocatorWithDefaultOptions allocator;
        
        // Input
        size_t num_input_nodes = session_->GetInputCount();
        if (num_input_nodes != 1) {
            std::cerr << "Error: Expected 1 input, got " << num_input_nodes << std::endl;
            return false;
        }
        
        auto input_name_ptr = session_->GetInputNameAllocated(0, allocator);
        input_name_ = input_name_ptr.get();
        auto input_type_info = session_->GetInputTypeInfo(0);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        input_shape_ = input_tensor_info.GetShape();
        
        // Handle dynamic batch dimension
        if (input_shape_.size() > 0 && input_shape_[0] == -1) {
            input_shape_[0] = 1;  // Set batch size to 1
        }
        
        input_size_ = 1;
        for (size_t i = 0; i < input_shape_.size(); ++i) {
            if (input_shape_[i] > 0) {
                input_size_ *= input_shape_[i];
            }
        }
        
        // Output
        size_t num_output_nodes = session_->GetOutputCount();
        if (num_output_nodes != 1) {
            std::cerr << "Error: Expected 1 output, got " << num_output_nodes << std::endl;
            return false;
        }
        
        auto output_name_ptr = session_->GetOutputNameAllocated(0, allocator);
        output_name_ = output_name_ptr.get();
        auto output_type_info = session_->GetOutputTypeInfo(0);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        output_shape_ = output_tensor_info.GetShape();
        
        // Handle dynamic batch dimension
        if (output_shape_.size() > 0 && output_shape_[0] == -1) {
            output_shape_[0] = 1;
        }
        
        output_size_ = 1;
        for (size_t i = 0; i < output_shape_.size(); ++i) {
            if (output_shape_[i] > 0) {
                output_size_ *= output_shape_[i];
            }
        }
        
        // Create memory info
        memory_info_ = std::make_unique<Ort::MemoryInfo>(
            Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU));
        
        // Create run options
        run_options_ = std::make_unique<Ort::RunOptions>();
        
        initialized_ = true;
        
        std::cout << "ONNX model loaded successfully:" << std::endl;
        std::cout << "  Input: " << input_name_ << " shape: [";
        for (size_t i = 0; i < input_shape_.size(); ++i) {
            std::cout << input_shape_[i];
            if (i < input_shape_.size() - 1) std::cout << ", ";
        }
        std::cout << "] (size: " << input_size_ << ")" << std::endl;
        
        std::cout << "  Output: " << output_name_ << " shape: [";
        for (size_t i = 0; i < output_shape_.size(); ++i) {
            std::cout << output_shape_[i];
            if (i < output_shape_.size() - 1) std::cout << ", ";
        }
        std::cout << "] (size: " << output_size_ << ")" << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing ONNX model: " << e.what() << std::endl;
        return false;
    }
}

bool OnnxWrapper::run(const std::vector<float>& input, std::vector<float>& output) {
    if (!initialized_) {
        std::cerr << "Error: ONNX model not initialized" << std::endl;
        return false;
    }
    
    if (input.size() != input_size_) {
        std::cerr << "Error: Input size mismatch. Expected " << input_size_ 
                  << ", got " << input.size() << std::endl;
        return false;
    }
    
    try {
        // Create input tensor
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            *memory_info_,
            const_cast<float*>(input.data()),
            input_size_,
            input_shape_.data(),
            input_shape_.size()
        );
        
        // Run inference
        const char* input_names[] = {input_name_.c_str()};
        const char* output_names[] = {output_name_.c_str()};
        
        auto output_tensors = session_->Run(
            *run_options_,
            input_names, &input_tensor, 1,
            output_names, 1
        );
        
        // Extract output
        float* output_data = output_tensors[0].GetTensorMutableData<float>();
        size_t output_count = output_tensors[0].GetTensorTypeAndShapeInfo().GetElementCount();
        
        output.resize(output_count);
        std::memcpy(output.data(), output_data, output_count * sizeof(float));
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error running inference: " << e.what() << std::endl;
        return false;
    }
}

