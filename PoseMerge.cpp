#include "PoseMerge.hpp"
// #include "Core/Matrix.h"
#include "ImuTypes.h"
#include "System.h"
#include "opencv2/core/mat.hpp"
#include <cstddef>
#include <memory>
#include <utility>
#include <vector>

// #if defined (ANDROID)
ASensorEventQueue *PoseMerge::accelerometerEventQueue = nullptr;
ASensorEventQueue *PoseMerge::gyroscopeEventQueue = nullptr;

// #endif

// 新建一个静态instance实例，在初始化前别乱☞
PoseMerge* PoseMerge::instance_ = nullptr;

PoseMerge::PoseMerge() {
    this->instance_ = this;
    imuMeas_.reserve(5000);

}

PoseMerge::PoseMerge(std::shared_ptr<ORB_SLAM3::System> orbsys){

    this->instance_ = this;
    this->orbsys_ = orbsys;
    imuMeas_.reserve(5000);

    
};

PoseMerge::PoseMerge(std::string vocbin, std::string yaml, std::string ace_model_path) {

    this->instance_ = this;
    LOGI("create instance");

    this->orbsys_ = std::make_shared<ORB_SLAM3::System>(vocbin, yaml, 
                                                        ORB_SLAM3::System::IMU_MONOCULAR);
    LOGI("create slam");

    imuMeas_.reserve(5000);

};

Eigen::Matrix4f PoseMerge::getPose() {
    return this -> curORB_;
};


void PoseMerge::imuStart() {
    // #if defined (ANDROID)
    ASensorManager *sensorManager = ASensorManager_getInstance();
    assert(sensorManager != NULL);

    ALooper *looper = ALooper_forThread();
    if (looper == NULL)
        looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
    assert(looper != NULL);

    accelerometerEventQueue = ASensorManager_createEventQueue(sensorManager, looper,
                                                              LOOPER_ID_USER, NULL,
                                                              NULL);
    assert(accelerometerEventQueue != NULL);
    const ASensor *accelerometer = ASensorManager_getDefaultSensor(sensorManager,
                                                                   ASENSOR_TYPE_ACCELEROMETER);
    assert(accelerometer != NULL);
    auto status = ASensorEventQueue_enableSensor(accelerometerEventQueue,
                                                 accelerometer);
    assert(status >= 0);
    status = ASensorEventQueue_setEventRate(accelerometerEventQueue,
                                            accelerometer,
                                            SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);

    gyroscopeEventQueue = ASensorManager_createEventQueue(sensorManager, looper,
                                                          LOOPER_ID_USER, process_imu_sensor_events,
                                                          NULL);
    assert(gyroscopeEventQueue != NULL);
    const ASensor *gyroscope = ASensorManager_getDefaultSensor(sensorManager,
                                                               ASENSOR_TYPE_GYROSCOPE);
    assert(gyroscope != NULL);
    status = ASensorEventQueue_enableSensor(gyroscopeEventQueue,
                                            gyroscope);
    assert(status >= 0);
    status = ASensorEventQueue_setEventRate(gyroscopeEventQueue,
                                            gyroscope,
                                            SENSOR_REFRESH_PERIOD_US);
    assert(status >= 0);

    // LOGI("IMU EventQueues initialized and started");


    // #endif
};


// #if defined (ANDROID)
int PoseMerge::process_imu_sensor_events(int fd, int events, void *data) {
    static ASensorEvent acclEvent;
    static double acclEventTimestamp = -1.0;
    ASensorEvent gyroEvent;

    while (ASensorEventQueue_getEvents(gyroscopeEventQueue, &gyroEvent, 1) > 0) {
        assert(gyroEvent.type == ASENSOR_TYPE_GYROSCOPE);


        double timeStampGyro = (gyroEvent.timestamp)/1e9;
//        LOGI("IMU gyro event timeStamp: %lf", timeStampGyro);
        //The timestamp is the amount of time in seconds since the device booted.
        assert(timeStampGyro > 0);

        IMU_MSG gyro_msg;
        gyro_msg.header = timeStampGyro;
        // in iOS and in Android the unit is rad/s
        gyro_msg.gyr << gyroEvent.uncalibrated_gyro.x_uncalib, //latestGyro.rotationRate.x,
                gyroEvent.uncalibrated_gyro.y_uncalib, //latestGyro.rotationRate.y,
                gyroEvent.uncalibrated_gyro.z_uncalib; //latestGyro.rotationRate.z;

        if (instance_->gyro_buf.size() == 0) {
            LOGI("gyro interpolation buffer empty. should only happen once.");
            instance_->gyro_buf.push_back(gyro_msg);
            instance_->gyro_buf.push_back(gyro_msg);
            continue;
        } else if (gyro_msg.header <= instance_->gyro_buf[1].header) {
            // Apparently events can be fired twice
            // Drop this event as it isn't more recent than the last one
            continue;
        } else {
            instance_->gyro_buf[0] = instance_->gyro_buf[1];
            instance_->gyro_buf[1] = gyro_msg;
        }

        if (instance_->imu_prepare < 10) {
            instance_->imu_prepare++;
            continue;
        }

        while (acclEventTimestamp < instance_->gyro_buf[0].header) {
//            LOGI("acclEventTimestamp < gyroEvent.timestamp: %lf < %lf", acclEventTimestamp , instance->gyro_buf[0].header);
            ssize_t numEvents;
            while ((numEvents = ASensorEventQueue_getEvents(accelerometerEventQueue, &acclEvent,
                                                            1)) == 0) {
//                LOGI("having to wait for accl event");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            assert(numEvents == 1);
            assert(acclEvent.type == ASENSOR_TYPE_ACCELEROMETER);

            acclEventTimestamp = (acclEvent.timestamp)/1e9;
            // LOGI("imu_timestamp_from_c++: %f ", acclEventTimestamp);
            instance_->imu_timestamp_ = acclEventTimestamp;
//            LOGI("IMU accl event timeStamp: %lf", timeStampAccl);
            shared_ptr<IMU_MSG> acc_msg(new IMU_MSG());
            acc_msg->header = acclEventTimestamp;
            // TODO: Apply a matrix multiplication to enable use of different coordinate systems
            // in Android the unit is m/s^2 in iOS it is g (9.8m/s^2)
            acc_msg->acc << acclEvent.acceleration.x,
                    acclEvent.acceleration.y,
                    acclEvent.acceleration.z;
            instance_->cur_acc = acc_msg;
        }
//        LOGI("waited for accl event: %lf >= %lf", acclEventTimestamp, instance->gyro_buf[0].header);
        if (instance_->gyro_buf[1].header < acclEventTimestamp) {
            // LOGE("having to wait for fitting gyro event"); // This should not happen if the frequency is the same
            continue;
        }


        //interpolation
         std::shared_ptr<IMU_MSG > imu_msg(new IMU_MSG());
//        LOGI("instance->cur_acc->header: \t\t%lf \ninstance->gyro_buf[0].header: \t%lf \ninstance->gyro_buf[1].header: \t%lf", instance->cur_acc->header, instance->gyro_buf[0].header, instance->gyro_buf[1].header);
        if (instance_->cur_acc->header >= instance_->gyro_buf[0].header &&
            instance_->cur_acc->header < instance_->gyro_buf[1].header) {
            imu_msg->header = instance_->cur_acc->header;
//            imu_msg->header = (double)cv::getTickCount() / cv::getTickFrequency();
            imu_msg->acc = instance_->cur_acc->acc;
            imu_msg->gyr = instance_->gyro_buf[0].gyr +
                           (instance_->gyro_buf[1].gyr - instance_->gyro_buf[0].gyr) *
                           (instance_->cur_acc->header - instance_->gyro_buf[0].header) /
                           (instance_->gyro_buf[1].header - instance_->gyro_buf[0].header);
        } else {
            LOGE("imu error %lf %lf %lf\n", instance_->gyro_buf[0].header, instance_->cur_acc->header,
                 instance_->gyro_buf[1].header);
            continue;
        }

        instance_->recvImu(imu_msg);
    }
    return 1;
}
// #endif

// std::vector<ORB_SLAM3::IMU::Point> PoseMerge::getMeasIMU() {
//     std::vector<ORB_SLAM3::IMU::Point > tmp;
//     for (auto iter : ) {
    
//     }
// };

void PoseMerge::recvImu(std::shared_ptr<IMU_MSG> imu_Msg) {
    LOGI("GET IMU DATA： %f %f %f %f %f %f",imu_Msg->acc.x(), imu_Msg->acc.y(), imu_Msg->acc.z(),
    imu_Msg->gyr.x(), imu_Msg->gyr.y(), imu_Msg->gyr.z());
    mtx_MeasVec.lock();
    this->imuMeas_.push_back( ORB_SLAM3::IMU::Point(
        imu_Msg->acc.x(), imu_Msg->acc.y(), imu_Msg->acc.z(),
        imu_Msg->gyr.x(), imu_Msg->gyr.y(), imu_Msg->gyr.z(),
        imu_Msg->header
    )
    );
    mtx_MeasVec.unlock();
};

bool PoseMerge::alinTimestamp(double timestamp) {
    // 当第一帧来的时候需要将两个时间戳进行对齐
    // 但是imu开始的时间较早，imuMeas中已经有了不少数据，第一帧抛弃，同时 之前的imu清空
    if (!this->balinTimestamp_) {
        balinTimestamp_ = true;
        timeGap_ = imu_timestamp_ - timestamp; // 假设imu的时间戳更大，也就是图片时间戳要加上这个时间戳才是imu同步的情况

        mtx_MeasVec.lock();
        imuMeas_.clear(); //
        mtx_MeasVec.unlock();
    }
    return balinTimestamp_;
};

void PoseMerge::putImg(cv::Mat& img, double timestamp) {
    if(alinTimestamp(timestamp)) {
        std::vector<ORB_SLAM3::IMU::Point> tmpIMU;
        tmpIMU.reserve(5000);

        mtx_MeasVec.lock();

        tmpIMU.swap(imuMeas_);
        LOGI("imu buf size:%d ",tmpIMU.size());
        mtx_MeasVec.unlock();
        if (tmpIMU.empty()) {
            return;
        }
        auto tcw = orbsys_->TrackMonocular(img, timestamp + timeGap_, tmpIMU);
        auto twc = tcw.inverse();
        curORB_ = twc.matrix();

    }
};