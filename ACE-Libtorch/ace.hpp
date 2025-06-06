#include <iostream>
#include <string>
#include <memory>
#include <torch/script.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <android/log.h>
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, __FILE__, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, __FILE__, __VA_ARGS__)
#define printf(...)  __android_log_print(ANDROID_LOG_ERROR, __FILE__, __VA_ARGS__)

class AceLocal {
public:
    AceLocal() = default;
    AceLocal(std::string &model_path):model_path_(model_path){
        std::cout<< "Set Model path" <<std::endl;
        initModel();
    }

    void initModel();
    Eigen::Matrix4f forward(cv::Mat img);
    void test_torch();
    void test_ace(double& temp);
    void preProcessImg(cv::Mat input, cv::Mat& output);
    



private:
    std::string model_path_;
    torch::jit::Module model_;
};