// #include "System.h"
#include <cstring>
#include <future>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "PoseMerge.hpp"
#include "boost/version.hpp"
#include <iostream>
#include <fstream>
#include "Eigen/Core"

#include <iomanip>
#include <string>
#include <thread>
#include <opencv2/core/core.hpp>
#include "System.h"
#include "Atlas.h"
#include "Map.h"
#include "ndkcamera.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc/types_c.h"
#include "sophus/se3.hpp"
#include <opencv2/core/eigen.hpp>
#include <unistd.h>

using namespace std;
using namespace cv;
// using namespace ORB_SLAM3;
string settings_file = "/storage/emulated/0/4DAR/Settings.yaml";
string voc_file = "/storage/emulated/0/4DAR/ORBvoc.bin";

cv::VideoWriter videoout;
std::ofstream gyro;
int timestamp_gyro = 0;
int coder=cv::VideoWriter::fourcc('M','J','P','G');//选择编码格式


double timestamp = 0;
int global_img = 0;
bool savevideo = true;
bool savevideoinit = false;
void ransac(vector<Point3f>& pts_3d, int max_iter, float threshold,
float &a1,float &b1,float &c1,float &d1);


void Log(string str)
{
    std::ofstream out;
    // get the time
    time_t timep;
    time(&timep);

    out.open("/storage/emulated/0/4DAR/log_fun.txt",ios::out|ios::app);
    // out<< "call save altas function" <<std::endl;
    out << asctime(localtime(&timep)) << "      :" <<str<<std::endl;

    out.close();
}
// test
extern "C"
{
    int sum(int x,int y)
    {
        return x+y;
    }
}


// slam
// ORB_SLAM3::System SLAM(voc_file,settings_file,ORB_SLAM3::System::MONOCULAR);
// ORB_SLAM3::Map *activeMap ;

// std::shared_ptr<ORB_SLAM3::System > kslam;

Sophus::SE3f tcw; 
Mat img_glob;
Mat R_,R_T;

std::shared_ptr<PoseMerge> kposeMerge ;
std::shared_ptr<Camera> ndkcam;

uchar* data;

// ndk camera & pose merge func
extern "C" {

    void init_sys() {

        kposeMerge = std::make_shared<PoseMerge>(voc_file,settings_file,"ABC");
        kposeMerge->set_out_file("/storage/emulated/0/4DAR/");

        ndkcam = std::make_shared<Camera>(kposeMerge);
        kposeMerge->imuStart();
        kposeMerge->setCamRecordFre(3);
        data = new uchar[1280*720*4];

    }

    void start_cam_imu() {
        // sleep(1);
        ndkcam->startCamera();
    }

    void kpose_record() {
        kposeMerge->start_record();
    }

    void kpose_stop_record() {
        kposeMerge->stop_record();
    }

    void kpose_start_slam() {
        kposeMerge->start();
    }



    void ndkcam_saveimg() {
        cv::Mat temp_img;
        ndkcam->getimg(temp_img);
        cv::imwrite("/storage/emulated/0/4DAR/ndk.jpg", temp_img);

    }

    uchar* ndkcam_getimg(int& size) {
        cv::Mat temp_img;
        ndkcam->getimg(temp_img);

        cv::cvtColor(temp_img, temp_img, cv::COLOR_BGR2RGBA);
        
        size = temp_img.cols * temp_img.rows * temp_img.channels();
        memcpy(data, temp_img.data, temp_img.total()*sizeof(uchar)*4);
        return data;

    };

    void getpose(float T[],float R[]) {
        Eigen::Matrix4f twc = kposeMerge->getPose();
        Mat twc_mat;
        cv::eigen2cv(twc,twc_mat);
        
        R[0] = twc_mat.at<float>(0,0);
        R[1] = twc_mat.at<float>(1,0);
        R[2] = twc_mat.at<float>(2,0);
        R[3] = twc_mat.at<float>(0,1);
        R[4] = twc_mat.at<float>(1,1);
        R[5] = twc_mat.at<float>(2,1);
        R[6] = twc_mat.at<float>(0,2);
        R[7] = twc_mat.at<float>(1,2);
        R[8] = twc_mat.at<float>(2,2);

        T[0] = twc_mat.at<float>(0,3);
        T[1] = twc_mat.at<float>(1,3);
        T[2] = twc_mat.at<float>(2,3);

    };

}



// slam using image from unity. very very slow
extern "C"
{
    void ProcessImage(uchar *image_data,float T[],float R[], int width, int height, double timestamp)
    {
        Mat img_rec;
        img_rec = Mat(height,width,CV_8UC4,image_data);

        // float imageScale = SLAM.GetImageScale();
        // int new_width = img_rec.cols * imageScale;
        // int new_height = img_rec.rows * imageScale;
        // resize(img_rec,img_rec,Size(new_width,new_height));

        Mat img_right;
        transpose(img_rec,img_right);
        rotate(img_right,img_right,ROTATE_90_CLOCKWISE);
        flip(img_right,img_right,-1);

        Mat img_gray;
        cvtColor(img_right,img_gray,CV_RGBA2GRAY);

        img_glob = img_right;
        

        kposeMerge->putImg(img_gray, timestamp);
        Eigen::Matrix4f twc = kposeMerge->getPose();
        Mat twc_mat;
        cv::eigen2cv(twc,twc_mat);
        // R_ = Mat(3,3,CV_32FC1,tcw.rotationMatrix().data());

        R[0] = twc_mat.at<float>(0,0);
        R[1] = twc_mat.at<float>(1,0);
        R[2] = twc_mat.at<float>(2,0);
        R[3] = twc_mat.at<float>(0,1);
        R[4] = twc_mat.at<float>(1,1);
        R[5] = twc_mat.at<float>(2,1);
        R[6] = twc_mat.at<float>(0,2);
        R[7] = twc_mat.at<float>(1,2);
        R[8] = twc_mat.at<float>(2,2);

        T[0] = twc_mat.at<float>(0,3);
        T[1] = twc_mat.at<float>(1,3);
        T[2] = twc_mat.at<float>(2,3);
        
    }

    void mapping()
    {
        // SLAM.change2mapping();
    }

        void SaveImage(uchar *image_data,float g_data[], int a)
    {

        if(!savevideoinit)
        {
            gyro.open("/storage/emulated/0/4DAR/gyro.txt", ios::out);
            videoout.open("/storage/emulated/0/4DAR/map.avi",coder,30,cv::Size(1280,720));
            gyro<<"#timestamp [frame id],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]"<<endl;
            savevideoinit = true;
        }
        if (a==0)
        {
            
        }
        if(a==1)
        {
            gyro<<timestamp_gyro<<","
            << g_data[0]<<","<<g_data[1]<<","<<g_data[2]<<","
            << g_data[3]<<","<<g_data[4]<<","<<g_data[5]<<endl;
            timestamp_gyro ++;
            
            Mat img_rec;
            img_rec = Mat(720,1280,CV_8UC4,image_data);

            // float imageScale = SLAM.GetImageScale();
            // int new_width = img_rec.cols * imageScale;
            // int new_height = img_rec.rows * imageScale;
            // resize(img_rec,img_rec,Size(new_width,new_height));

            Mat img_right;
            transpose(img_rec,img_right);
            rotate(img_right,img_right,ROTATE_90_CLOCKWISE);
            flip(img_right,img_right,-1);
            videoout.write(img_right);
        }
        if(a==2)
        {
            videoout.release();
        }

        
        // string s = std::to_string(global_img);
        // imwrite("/storage/emulated/0/4DAR/"+s+".jpg", img_glob);
        // global_img++;
        // if(savevideo)
        // {
        //     savevideo=false;
        //     // videoout.release();
        // }
        // else
        // {
        //     savevideo=true;
        // }
    }

     void SaveMap()
    {
        // SLAM.SaveAtlas(0);
    }


void GetLocalMap()
    {
        ofstream out ;
        out.open("/storage/emulated/0/4DAR/local.pcd");
        out << "# .PCD v0.7 - Point Cloud Data file format \n"
            << "VERSION 0.7\n"
            << "FIELDS x y z\n"
            << "SIZE 4 4 4 \n"
            << "TYPE F F F \n"
            << "COUNT 1 1 1\n"<<endl;
        // Atlas 这个类实在orbslam3作用域下声明的，所以要带上作用域或使用namespace
        // ORB_SLAM3::Atlas *atlas;
        vector<ORB_SLAM3::MapPoint*> vpmaps ;
        // SLAM.GetLocalMap(vpmaps);
        vector<Point3f> mapPoints;

        out <<"WIDTH"<< " "<<vpmaps.size()<<"\n"
            <<"HEIGHT 1 \n" 
            << "VIEWPOINT 0 0 0 1 0 0 0\n"
            << "POINTS" << " "<< vpmaps.size() <<"\n"
            << "DATA ascii \n"
            << endl;

        for(size_t i = 0,iend = vpmaps.size();i<iend;i++)
        {
            if (vpmaps[i]->isBad())
                continue;
            Eigen::Matrix<float ,3,1> pos = vpmaps[i]->GetWorldPos();
            Point3f tmpPoint;
            tmpPoint.x = pos(0);
            tmpPoint.y = pos(1);
            tmpPoint.z = pos(2);
            mapPoints.push_back(tmpPoint);

            out<<pos(0)<< " " <<pos(1)<< " " <<pos(2)<<endl;
            
        }
        out.close();

    }



    void FitPlane(float plane_arg[])
    {
        float a,b,c,d;
        vector<ORB_SLAM3::MapPoint*> vpmaps ;
        // SLAM.GetLocalMap(vpmaps);
        vector<Point3f> mapPoints;
        float mean_x,mean_y,mean_z;
        float sum_x=0,sum_y=0,sum_z=0;
        for(size_t i = 0,iend = vpmaps.size();i<iend;i++)
        {
            if (vpmaps[i]->isBad())
                continue;
            Eigen::Matrix<float ,3,1> pos = vpmaps[i]->GetWorldPos();
            Point3f tmpPoint;
            tmpPoint.x = pos(0);
            tmpPoint.y = pos(1);
            tmpPoint.z = pos(2);
            mapPoints.push_back(tmpPoint);
            sum_x = sum_x+pos(0);
            sum_y = sum_y+pos(1);
            sum_z = sum_z+pos(2);
        }
        mean_x = sum_x/vpmaps.size();
        mean_y = sum_y/vpmaps.size();
        mean_z = sum_z/vpmaps.size();
        ransac(mapPoints,100,2,a,b,c,d);
        plane_arg[0] = a;
        plane_arg[1] = b;
        plane_arg[2] = c;
        plane_arg[3] = d;
        plane_arg[4] = mean_x;
        plane_arg[5] = mean_y;
        plane_arg[6] = mean_z;
    }

        void SlamShutDown()
    {
        // save atlas
        // SLAM.Shutdown();
    }

         
    void Relocalization(float R_[],float t_[])
    {
        // vector<Eigen::Vector3f> p1,p2;
        // Eigen::Matrix3f R;
        // Eigen::Vector3f t;
        // ReadPcd("/storage/emulated/0/4DAR/local.pcd",p1);
        // ReadPcd("/storage/emulated/0/4DAR/map.pcd",p2);
        // ICP(p1,p2,R,t);
        // R_[0] = R(0,0);
        // R_[0] = R(1,0);
        // R_[0] = R(2,0);
        // R_[0] = R(0,1);
        // R_[0] = R(1,1);
        // R_[0] = R(2,1);
        // R_[0] = R(0,2);
        // R_[0] = R(1,2);
        // R_[0] = R(2,2);
        // t_[0] = t(0);
        // t_[1] = t(1);
        // t_[2] = t(2);

    }

}






void ransac(vector<Point3f>& pts_3d, int max_iter, float threshold,
float &a1,float &b1,float &c1,float &d1)
{
    int size_old = 3;
    float a,b,c,d; //

    while (--max_iter)
    {
        /* code */
        vector<int > index;
        for (int k = 0;k<3;++k)
        {
            index.push_back(rand() % pts_3d.size());

        }

        // 根据迭代器获取三个点的xyz坐标
        auto idx = index.begin();
        float x1 = pts_3d.at(*idx).x;
        float y1 = pts_3d.at(*idx).y;
        float z1 = pts_3d.at(*idx).z;
        idx++;

        float x2 = pts_3d.at(*idx).x;
        float y2 = pts_3d.at(*idx).y;
        float z2 = pts_3d.at(*idx).z;
        idx++;

        float x3 = pts_3d.at(*idx).x;
        float y3 = pts_3d.at(*idx).y;
        float z3 = pts_3d.at(*idx).z;

        //根据三个点来拟合平面
        a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		d = -(a*x1 + b*y1 + c*z1);

        for(auto iter = pts_3d.begin();iter != pts_3d.end(); ++iter)
        {
            float dis = fabs(a*iter->x 
                            + b*iter->y
                            + c*iter->z +d)/
                            sqrt(a*a+b*b+c*c);

            if(dis < threshold)
            {
                index.push_back(iter - pts_3d.begin());
            }

            if (index.size() > size_old)
            {
                size_old = index.size();
            }
            index.clear();

        }

    }
    a1 = a;
    b1 = b;
    c1 = c;
    d1 = d;
    
}

