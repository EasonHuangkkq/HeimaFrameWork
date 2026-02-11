/*
 * Standalone test for ONNX policy inference.
 * Loads an ONNX model, runs inference with zero and simple test inputs, prints outputs.
 *
 * Usage:
 *   ./test_onnx [onnx_file]
 * Default: checkpoints/policy_1_0127.onnx
 */

#include "onnx_wrapper.h"
#include <iostream>
#include <vector>
#include <iomanip>

int main(int argc, char* argv[]) {
	std::string onnx_file = "checkpoints/policy_1_0127.onnx";
	if(argc >= 2){
		onnx_file = argv[1];
	}

	std::cout << "=== ONNX policy test ===" << std::endl;
	std::cout << "Model: " << onnx_file << std::endl;

	OnnxWrapper onnx;
	if(!onnx.init(onnx_file)){
		std::cerr << "Failed to load ONNX model." << std::endl;
		return 1;
	}

	size_t inputSize = onnx.getInputSize();
	size_t outputSize = onnx.getOutputSize();
	std::cout << "\nInput size: " << inputSize << ", Output size: " << outputSize << std::endl;

	// Test 1: zero observation
	std::vector<float> obs(inputSize, 0.0f);
	std::vector<float> actions(outputSize, 0.0f);

	if(!onnx.run(obs, actions)){
		std::cerr << "Inference failed (zero input)." << std::endl;
		return 1;
	}
	std::cout << "\n--- Zero observation ---" << std::endl;
	std::cout << "Output:";
	for(size_t i = 0; i < actions.size(); ++i){
		std::cout << " " << std::fixed << std::setprecision(4) << actions[i];
	}
	std::cout << std::endl;

	// Test 2: small constant observation (e.g. scaled 0.1)
	for(size_t i = 0; i < obs.size(); ++i){
		obs[i] = 0.1f;
	}
	if(!onnx.run(obs, actions)){
		std::cerr << "Inference failed (constant input)." << std::endl;
		return 1;
	}
	std::cout << "\n--- Constant observation (0.1) ---" << std::endl;
	std::cout << "Output:";
	for(size_t i = 0; i < actions.size(); ++i){
		std::cout << " " << std::fixed << std::setprecision(4) << actions[i];
	}
	std::cout << std::endl;

	// Test 3: simple pattern (first half 0, second half 0.2)
	for(size_t i = 0; i < obs.size(); ++i){
		obs[i] = (i < obs.size() / 2) ? 0.0f : 0.2f;
	}
	if(!onnx.run(obs, actions)){
		std::cerr << "Inference failed (pattern input)." << std::endl;
		return 1;
	}
	std::cout << "\n--- Pattern observation (0 then 0.2) ---" << std::endl;
	std::cout << "Output:";
	for(size_t i = 0; i < actions.size(); ++i){
		std::cout << " " << std::fixed << std::setprecision(4) << actions[i];
	}
	std::cout << std::endl;

	std::cout << "\n=== All inferences passed ===" << std::endl;
	return 0;
}
