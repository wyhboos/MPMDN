#include <torch/script.h> // One-stop header.

#include <iostream>
#include <memory>
#include <chrono>

// int main(int argc, const char* argv[]) {
int main() {
//   if (argc != 2) {
//     std::cerr << "usage: example-app <path-to-exported-script-module>\n";
//     return -1;
//   }

  // std::cout<<argc<<std::endl;
  // std::cout<<argv[0]<<std::endl;
  std::string argv = "../Data/Model_structure/MPN_Pnet.pt";
  torch::jit::script::Module module;
  try {
    // Deserialize the ScriptModule from a file using torch::jit::load().
    module = torch::jit::load("../../Data/Model_structure/MPN_Pnet.pt");
  }
  catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return -1;
  }
  std::vector<torch::jit::IValue> inputs_1;
  inputs_1.push_back(torch::ones({1,28}));
  inputs_1.push_back(torch::ones({1,2}));
  inputs_1.push_back(torch::ones({1,2}));
  auto start = std::chrono::high_resolution_clock::now();
  at::Tensor output = module.forward(inputs_1).toTensor();
  auto end = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  std::cout << "代码执行时间:" << duration.count() << " 毫秒" << std::endl;
  std::cout<<output<<std::endl;
  std::cout << "ok\n";
  return 0;
}