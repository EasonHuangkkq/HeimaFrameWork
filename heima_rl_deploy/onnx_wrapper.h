/*
 * Simple ONNX Runtime wrapper for neural network inference
 */

#pragma once

#include <string>
#include <vector>
#include <memory>

// Forward declarations
namespace Ort {
    class Env;
    class Session;
    class Value;
    class MemoryInfo;
    class RunOptions;
}

class OnnxWrapper {
public:
    OnnxWrapper();
    ~OnnxWrapper();
    
    // Initialize ONNX model
    bool init(const std::string& onnx_file);
    
    // Run inference
    bool run(const std::vector<float>& input, std::vector<float>& output);
    
    // Get input/output sizes
    size_t getInputSize() const { return input_size_; }
    size_t getOutputSize() const { return output_size_; }
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }

private:
    bool initialized_;
    size_t input_size_;
    size_t output_size_;
    
    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::unique_ptr<Ort::RunOptions> run_options_;
    std::unique_ptr<Ort::MemoryInfo> memory_info_;
    
    std::string input_name_;
    std::string output_name_;
    
    std::vector<int64_t> input_shape_;
    std::vector<int64_t> output_shape_;
};

