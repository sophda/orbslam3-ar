#ifndef NDKCAMERA_H
#define NDKCAMERA_H

#include "PoseMerge.hpp"
#include <cstddef>
#include <opencv2/core/mat.hpp>
#include <utility>
#include <queue>
#include <unistd.h>
#include <cinttypes>
#include <cstring>
#include <media/NdkImage.h>
#include <camera/NdkCameraManager.h>
#include <android/native_window.h>
#include <media/NdkImageReader.h>
#include <functional>
#include <android/log.h>
#include <opencv2/opencv.hpp>
#include <thread>
#define TAG "NCamera"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)


class Camera {
public:
    Camera() ;
    Camera(PoseMerge *posemerge):posemerge_(posemerge){};
    
private:
    int img_heigth=1280,img_width=720;
    std::mutex mtx_img;
    PoseMerge *posemerge_ = nullptr;
    static Camera* caminstance_;
    cv::Mat img;
    ACameraManager* cameraManager = nullptr;
    ACameraDevice* cameraDevice = nullptr;
    ACaptureRequest* captureRequest = nullptr;
    ACameraOutputTarget* cameraOutputTarget = nullptr;
    ACameraCaptureSession* captureSession = nullptr;
    AImageReader* imageReader = nullptr;
    ANativeWindow* nativeWindow = nullptr;

    static void onCameraDeviceError(void* context, ACameraDevice* device, int error){};

    static void onCameraDeviceDisconnected(void* context, ACameraDevice* device){};

    static void onCaptureSessionClosed(void* context, ACameraCaptureSession* session){} ;

    static void onCaptureSessionActive(void* context, ACameraCaptureSession* session){} ;

    static void onImageAvailable(void* context, AImageReader* reader); //回调函数

    bool openCamera(const char* cameraId);

    bool createCaptureRequest(ACameraDevice_request_template templateId);

    bool createCaptureSession();

    bool createImageReader(int width, int height, int format, int maxImages);

    void setimg(const cv::Mat& callback_img);


public:
    void startCamera();

    void pauseCamera();

    void stopCamera();

    void getimg(cv::Mat& return_img);

public:
    void set_posemerge_instance();

    void set_image_to_posemerge();


};





#endif NDKCAMERA_H