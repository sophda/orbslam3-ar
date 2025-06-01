#include "ace.hpp"
#include <cstdio>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <torch/torch.h>
#include <iostream>
#include <torch/script.h>
#include "dsacstar.h"
#include <opencv2/opencv.hpp>


void AceLocal::test_torch(){
    // std::cout<< "Test torch" << std::endl;
    torch::Tensor f = torch::arange(25).reshape({5,5});
    torch::Tensor bar = torch::einsum("ii",f);
    std::cout<< f <<std::endl;
    
};
void AceLocal::initModel() {
    model_ = torch::jit::load(model_path_);

}

void AceLocal::forward(cv::Mat img, torch::Tensor& output){
    cv::Mat img_processed;
    preProcessImg(img,img_processed);

    at::Tensor img_tensor = torch::from_blob(
            img_processed.data,
            {480,854,1},
            torch::kFloat32
            );
    img_tensor = img_tensor.permute({2,0,1});
    img_tensor = img_tensor.unsqueeze(0);
    torch::Tensor model_output, dsacstar_output;
    dsacstar_output = torch::zeros({4,4});
    torch::NoGradGuard no_grad;
    model_output = model_({img_tensor}).toTensor();
    std::cout<<img_tensor.sizes() <<std::endl;
//    std::cout<<img_tensor<<std::endl;

//    using dsacstar
    float focal_length = 742.5671, ppx = 427.0, ppy = 240.0;
    dsacstar_rgb_forward(
            model_output,
            dsacstar_output,
            64,
            10,
            focal_length,
            ppx,
            ppy,
            100,
            100,
            8
            );
    // output = dsacstar_output.inverse();
    output = dsacstar_output;

}

void AceLocal::preProcessImg(cv::Mat input, cv::Mat& output) {
    cv::Mat gray_img, img_f;
    cv::imwrite("/storage/emulated/0/ARBSLAM/imgf.jpg", input);
    printf("ace %d", input.rows);
    cv::cvtColor(input, gray_img,cv::COLOR_BGR2GRAY);
    gray_img.convertTo(img_f,CV_32FC3,1/255.0);
    cv::subtract(img_f, cv::Scalar(0.485,0.456,0.406), img_f);
    cv::divide(img_f, cv::Scalar(0.229,0.224,0.225), img_f);
    output = img_f;
    
    
}

void AceLocal::test_ace(double& temp) {
    // std::string model_path = "/storage/emulated/0/ARBSLAM/ace.jit";
    // AceLocal ace(model_path);
//    ace.test_torch();
    torch::Tensor pos ;
    // std::string i,temp;
//    /home/sophda/project/ace/datasets/Cambridge_KingsCollege/train/rgb/seq1_frame00001.png
    cv::Mat img = cv::imread("/storage/emulated/0/ARBSLAM/seq1_frame00002.png"); // img_r,img_f,img_n;

    this->forward(img, pos);
    // torch::Tensor max = torch::argmax(pos,-1);
    // printf("ace: %f",max.item().toFloat());
    temp = 15.06;

        // printf()

}


// int main(){

//     std::string model_path = "/home/sophda/project/ace/output/ace.jit";
//     AceLocal ace(model_path);
// //    ace.test_torch();
//     torch::Tensor pos ;
//     std::string i,temp;
//     while(1){
//         printf("input:\n");
//         std::cin>>temp;
//         i = "/home/sophda/project/ace/datasets/Cambridge_KingsCollege/train/rgb/seq1_frame00002.png";

// //    /home/sophda/project/ace/datasets/Cambridge_KingsCollege/train/rgb/seq1_frame00001.png
//         cv::Mat img = cv::imread(i),
//                 img_r,img_f,img_n;
//         cv::cvtColor(img,img_r,cv::COLOR_BGR2GRAY);
//         img_r.convertTo(img_f,CV_32FC3,1/255.0);

//         cv::Mat normalized;
//         cv::subtract(img_f, cv::Scalar(0.485, 0.456, 0.406), normalized);
//         cv::divide(normalized, cv::Scalar(0.229, 0.224, 0.225), normalized);

//         normalized = img_f;

//         ace.forward(img,pos);

//         std::cout<< pos <<std::endl;
//     }

//     return 0;
// }