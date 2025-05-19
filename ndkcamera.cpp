#include "ndkcamera.hpp"
#include <cstddef>
#include <mutex>
#include <opencv2/core/mat.hpp>

Camera* Camera::caminstance_ = nullptr;

Camera::Camera() {
    this->caminstance_ = this;
}

void Camera::onImageAvailable(void* context, AImageReader* reader) {
    AImage* image = nullptr;
    media_status_t mStatus = AImageReader_acquireNextImage(reader, &image);
    if (mStatus != AMEDIA_OK) {
        return;
    }else{
    	//一般打开摄像头后最重要的就在这里
    	//如果想要做什么事一定是在这个回调处获取得到的image进行处理
        //handleHandData(image);
        // AImage *yuv_image = nullptr;
        // auto status = AImageReader_acquireNextImage(reader, &(caminstance_->image));
        LOGI("[File]%s [Func]%s [Line]%d", __FILE__, __FUNCTION__, __LINE__);
        // Check status here ...


        // Try to process data without blocking the callback
        std::thread processor([=](){
            LOGI("<---------- Image processing loop ... --------->");
            // uint8_t *yPixel = nullptr;
            // uint8_t *uPixel = nullptr;
            // uint8_t *vPixel = nullptr;

            // int32_t yLen = 0;
            // int32_t uLen = 0;
            // int32_t vLen = 0;

            // cv::Mat _yuv_rgb_img, _yuv_gray_img;

            // AImage_getPlaneData((caminstance_->image), 0, &yPixel, &yLen);
            // AImage_getPlaneData((caminstance_->image), 1, &uPixel, &uLen);
            // AImage_getPlaneData((caminstance_->image), 2, &vPixel, &vLen);

            // uint8_t * data = new uint8_t[yLen + vLen + uLen];
            // memcpy(data, yPixel, yLen);
            // memcpy(data+yLen, vPixel, vLen);
            // memcpy(data+yLen+vLen, uPixel, uLen);

            // cv::Mat mYUV = cv::Mat(640*1.5,480, CV_8UC1, data);
            // cv::cvtColor(mYUV, _yuv_rgb_img, CV_YUV2RGB_NV12, 3);

            // cv::rotate(_yuv_rgb_img, _yuv_rgb_img, cv::ROTATE_90_CLOCKWISE);
            // _yuv_gray_img = cv::Mat(640, 480, CV_8UC1, yPixel);
            // cv::rotate(_yuv_gray_img, _yuv_gray_img, cv::ROTATE_90_CLOCKWISE);

            // // LOGI(...)
            // LOGI("---NDK CAM:rows:%d, cols:%d",_yuv_rgb_img.rows, _yuv_rgb_img.cols);
// ===========================================================================================
            uint8_t *yBuffer, *uBuffer, *vBuffer;
            int yRowStride, uRowStride, vRowStride;
            int yPixelStride, uPixelStride, vPixelStride;
            int width, height;

            AImage_getWidth(image, &width);
            AImage_getHeight(image, &height);
            
            int32_t yLen = 0;
            int32_t uLen = 0;
            int32_t vLen = 0;
            // 获取平面数据
            AImage_getPlaneData(image, 0, &yBuffer, &yLen);
            AImage_getPlaneRowStride(image, 0, &yRowStride);
            AImage_getPlanePixelStride(image, 0, &yPixelStride);

            AImage_getPlaneData(image, 1, &uBuffer, &uLen);
            AImage_getPlaneRowStride(image, 1, &uRowStride);
            AImage_getPlanePixelStride(image, 1, &uPixelStride);

            AImage_getPlaneData(image, 2, &vBuffer, &vLen);
            AImage_getPlaneRowStride(image, 2, &vRowStride);
            AImage_getPlanePixelStride(image, 2, &vPixelStride);

            // 判断 UV 平面是否交错
            bool isNV21 = (uPixelStride == 2 && vPixelStride == 2);

            cv::Mat yuv_mat,rgb_mat;
            if (isNV21) {
            yuv_mat = cv::Mat(height + height/2, width, CV_8UC1);

            // 复制 Y 平面
            for (int i = 0; i < height; i++) {
                memcpy(yuv_mat.ptr(i), yBuffer + i * yRowStride, width);
            }

            // 复制 UV 交错数据（NV21）
            uint8_t* uvDst = yuv_mat.ptr(height);
            for (int i = 0; i < height/2; i++) {
                for (int j = 0; j < width/2; j++) {
                    uvDst[i * width + 2 * j] = vBuffer[i * vRowStride + j * 2]; // V
                    uvDst[i * width + 2 * j + 1] = uBuffer[i * uRowStride + j * 2]; // U
                }
            }

            cv::cvtColor(yuv_mat, rgb_mat, cv::COLOR_YUV2RGB_NV12);
            } else {
            // 处理 I420 格式
            cv::Mat yuv_i420(height * 3/2, width, CV_8UC1);

            // 复制 Y 平面
            for (int i = 0; i < height; i++) {
                memcpy(yuv_i420.ptr(i), yBuffer + i * yRowStride, width);
            }

            // 复制 U 和 V 平面
            int uvHeight = height / 2;
            int uvWidth = width / 2;
            for (int i = 0; i < uvHeight; i++) {
                memcpy(yuv_i420.ptr(height + i), uBuffer + i * uRowStride, uvWidth);
                memcpy(yuv_i420.ptr(height + uvHeight + i), vBuffer + i * vRowStride, uvWidth);
            }

            cv::cvtColor(yuv_i420, rgb_mat, cv::COLOR_YUV2RGB_I420);
            }


            caminstance_->setimg(rgb_mat);

            AImage_delete(image);

    });
    processor.detach();
    //得到image处理完后记得删除，不然缓冲区溢出了你打开的camera就寄了
}
};

void Camera::setimg(const cv::Mat& callback_img) {
    std::unique_lock<std::mutex> lock(mtx_img);
    this->img = callback_img.clone();
}

void Camera::getimg(cv::Mat& return_img) {
    std::unique_lock<std::mutex> lock(this->mtx_img);
    return_img = this->img.clone();
    // return this->img;
}



bool Camera::openCamera(const char* cameraId) {
        // 根据cameraID打开摄像头
    ACameraDevice_StateCallbacks deviceStateCallbacks {
            .context = nullptr,
            .onDisconnected = onCameraDeviceDisconnected,
            .onError = onCameraDeviceError
    };
    camera_status_t status = ACameraManager_openCamera(cameraManager, cameraId, &deviceStateCallbacks, &cameraDevice);
    return status == ACAMERA_OK;

};

bool Camera::createCaptureRequest(ACameraDevice_request_template templateId) {
    camera_status_t status = ACameraDevice_createCaptureRequest(cameraDevice, templateId, &captureRequest);
    if (status != ACAMERA_OK) return false;
    status = ACaptureRequest_addTarget(captureRequest, cameraOutputTarget);
    return status == ACAMERA_OK;
};

bool Camera::createCaptureSession() {
    int *i = nullptr;
    ACaptureSessionOutputContainer* outputContainer = nullptr;
    ACaptureSessionOutput* sessionOutput = nullptr;
    //初始化ACaptureSessionOutputContainer
    camera_status_t status = ACaptureSessionOutputContainer_create(&outputContainer);
    if (status != ACAMERA_OK) return false;
    //初始化ACaptureSessionOutput
    status = ACaptureSessionOutput_create(nativeWindow, &sessionOutput);
    if (status != ACAMERA_OK) return false;
    status = ACaptureSessionOutputContainer_add(outputContainer, sessionOutput);
    if (status != ACAMERA_OK) return false;
    ACameraCaptureSession_stateCallbacks sessionStateCallbacks {
            .context = nullptr,
            .onClosed = onCaptureSessionClosed,
            .onReady = nullptr,
            .onActive = onCaptureSessionActive
    };
    //初始化ACameraCaptureSession
    status = ACameraDevice_createCaptureSession(cameraDevice, outputContainer, &sessionStateCallbacks, &captureSession);
    if (status != ACAMERA_OK) return false;
	//在这里开始去重复请求拿到图片进行输出
    status = ACameraCaptureSession_setRepeatingRequest(captureSession, nullptr, 1, &captureRequest,
                                                       i++);
    return status == ACAMERA_OK;
};

bool Camera::createImageReader(int width, int height, int format, int maxImages) {
    //要循环去读取图片，创建一个reader
    media_status_t status = AImageReader_new(width, height, format, maxImages, &imageReader);
    if (status != AMEDIA_OK) {
        LOGI(TAG,"创建失败");
        return false;
    }
	//在这里注册AImage的回调，重中之重
    AImageReader_ImageListener imageListener {
            .context = nullptr,
            .onImageAvailable = onImageAvailable
    };
    status = AImageReader_setImageListener(imageReader, &imageListener);
    if (status != AMEDIA_OK) return false;
    status = AImageReader_getWindow(imageReader, &nativeWindow);
    return status == AMEDIA_OK;
};

void Camera::startCamera() {
// 开始使用摄像头预览
    cameraManager = ACameraManager_create();
    // 获取摄像头id
    ACameraIdList *cameraIdList = nullptr;
    ACameraManager_getCameraIdList(cameraManager, &cameraIdList);
    const char *cameraId = nullptr;
    for (int i = 0; i < cameraIdList->numCameras; i++)
    {
        const char *id = cameraIdList->cameraIds[i];
        ACameraMetadata *metadata = nullptr;
        ACameraManager_getCameraCharacteristics(cameraManager, id, &metadata);
        ACameraMetadata_const_entry entry;
        ACameraMetadata_getConstEntry(metadata, ACAMERA_LENS_FACING, &entry);
        auto facing = static_cast<acamera_metadata_enum_android_lens_facing_t>(entry.data.u8[0]);
        if (facing == ACAMERA_LENS_FACING_BACK)
        {
            cameraId = id;
            LOGI(TAG,"cameraid=====%s",cameraId);
            break;
        }
    }
    LOGI("cameraid:%s",cameraId);

    // 打开摄像头并且读取image
    //camera的参数自己定制
    if (openCamera(cameraId) && createImageReader(img_heigth , img_width, AIMAGE_FORMAT_YUV_420_888, 1)){
        // 创建一个输出的目标
        ACameraOutputTarget_create(nativeWindow, &cameraOutputTarget);
        // 创建捕获请求和捕获会话
        if (createCaptureRequest(TEMPLATE_RECORD)){
            createCaptureSession();

        }
    }
};

void Camera::pauseCamera(){
    ACameraDevice_close(cameraDevice);
    cameraDevice = nullptr;
}

//关闭camera，注意要释放掉不为空的结构
void Camera::stopCamera(){
    // Stop the camera and preview and release the resources
    if (captureSession != nullptr) {
        ACameraCaptureSession_stopRepeating(captureSession);
        ACameraCaptureSession_close(captureSession);
        captureSession = nullptr;
    }
    if (captureRequest != nullptr) {
        ACaptureRequest_removeTarget(captureRequest, cameraOutputTarget);
        ACaptureRequest_free(captureRequest);
        captureRequest = nullptr;
    }
    if (cameraOutputTarget != nullptr) {
        ACameraOutputTarget_free(cameraOutputTarget);
        cameraOutputTarget = nullptr;
    }
    if (cameraDevice != nullptr) {
        ACameraDevice_close(cameraDevice);
        cameraDevice = nullptr;
    }
    if (imageReader != nullptr) {
        AImageReader_delete(imageReader);
        imageReader = nullptr;
    }
    if (cameraManager != nullptr) {
        ACameraManager_delete(cameraManager);
        cameraManager = nullptr;
    }
    // Closed the camera and preview successfully
}
