#include <iostream>
#include <string>
#include <memory>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>

class AceLocal {
public:
    AceLocal() = default;
    AceLocal(std::string &model_path):model_path_(model_path){
        std::cout<< "Set Model path" <<std::endl;
        initModel();
    }

    void initModel();
    void forward(cv::Mat& img, torch::Tensor& output);
    void test_torch();
    void preProcessImg(cv::Mat& input, cv::Mat& output);



private:
    std::string model_path_;
    torch::jit::Module model_;
};