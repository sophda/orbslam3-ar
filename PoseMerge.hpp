#ifndef POSE_MERGE_H
#define POSE_MERGE_H


// #include "Eigen/Core/Matrix.h"
#include "ImuTypes.h"
#include "MapPoint.h"
#include "System.h"
#include "opencv2/core/types.hpp"
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <opencv2/videoio.hpp>
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>

#include "ThreadSafeQueue.hpp"
// #if defined (ANDROID)
// android IMU sensor
#include <android/looper.h>
#include <android/sensor.h>

#include <android/log.h>

#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, __FILE__, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, __FILE__, __VA_ARGS__)
#define printf(...)  __android_log_print(ANDROID_LOG_ERROR, __FILE__, __VA_ARGS__)

// #endif

class Transform {
public: 
    Eigen::Matrix3f t;
    Eigen::Matrix<float, 3, 3> r;

};

struct IMU_MSG {
    double header;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};

struct IMG_MSG {
    double t;
    cv::Mat img;
};

class PoseMerge{
public:
    PoseMerge();
    
    PoseMerge(std::shared_ptr<ORB_SLAM3::System> orbsys);

    PoseMerge(std::string vocbin, std::string yaml, std::string ace_model_path);

    void putImg(cv::Mat img, double timestamp);

    void alinOrbAce(std::map<double, Transform> Torb, std::map<double, Transform> Tace);

    Eigen::Matrix4f getPose() const;

    void set_out_file(std::string path) {
        out_file_imu.open(path+"/imu.txt",ios::out );
        out_file_img.open(path+"/img.txt",ios::out );
        video_out.open(path+"/video.avi",cv::VideoWriter::fourcc('M','J','P','G'),30, cv::Size(1280,720));
    }

    void imuStart();

    void start();

    void start_record();

    void stop_record() {
        if (record_flag.load()) {
            record_flag.store(false);
            if (th_record.joinable()) {
                th_record.join();
            }
        }
    }

    

// thread
private:
    std::thread th_slam, th_record;
    // record
    std::atomic<bool> record_flag;
    std::ofstream out_file_img,out_file_imu;
    cv::VideoWriter video_out;

    void process();
    void process_record();
    

// orb
private:

    std::vector<IMU_MSG > gyro_buf;

    std::string ace_model_path_;
    static PoseMerge* instance_;
    bool bskipImg_=true, balinTimestamp_=false; 
    Eigen::Matrix4f curORB_;

    std::shared_ptr<ORB_SLAM3::System > orbsys_;

    std::vector<ORB_SLAM3::IMU::Point> imuMeas_;

    SafeQueue<ORB_SLAM3::IMU::Point> imu_queue_;
    SafeQueue<IMG_MSG> img_queue_;

    bool getM(cv::Mat& img, double& timestamp, std::vector<ORB_SLAM3::IMU::Point>& imu_vec);


// 获取imu数据
private:
    double imu_timestamp_, img_timestamp_;
    int imu_prepare = 0;
    shared_ptr<IMU_MSG> cur_acc = shared_ptr<IMU_MSG>(
        new IMU_MSG());
    std::mutex mtx_MeasVec;
    double timeGap_=0;
    
    // #if defined (ANDROID)
    static ASensorEventQueue *accelerometerEventQueue;
    static ASensorEventQueue *gyroscopeEventQueue;

    const int LOOPER_ID_USER = 3;
    const int SENSOR_REFRESH_RATE_HZ = 100;
    const int32_t SENSOR_REFRESH_PERIOD_US = int32_t(1000000 / SENSOR_REFRESH_RATE_HZ);

    static int process_imu_sensor_events(int fd, int events, void *data);
    
    void recvImu(std::shared_ptr<ORB_SLAM3::IMU::Point> imu_Msg);
    
    bool alinTimestamp(double timestamp);




    // #endif


};




#endif 