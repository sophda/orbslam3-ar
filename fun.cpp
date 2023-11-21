// #include "System.h"
#include <opencv2/opencv.hpp>
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

using namespace std;
using namespace cv;
// using namespace ORB_SLAM3;
string settings_file = "/storage/emulated/0/4DAR/Settings.yaml";
string voc_file = "/storage/emulated/0/4DAR/ORBvoc.bin";

void ransac(vector<Point3f>& pts_3d, int max_iter, float threshold,
double &a1,double &b1,double &c1,double &d1);

// test
extern "C"
{
    int sum(int x,int y)
    {
        return x+y;
    }
}


// slam
ORB_SLAM3::System SLAM(voc_file,settings_file,ORB_SLAM3::System::MONOCULAR, false);
// ORB_SLAM3::Map *activeMap ;

Sophus::SE3f tcw; 
Mat img_glob;
Mat pose;

extern "C"
{
    void ProcessImage(uchar *image_data,float T[],float R[],int width, int height)
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
        

        tcw = SLAM.TrackMonocular(img_gray,0);
        pose = Mat(3,3,CV_32FC1,tcw.rotationMatrix().data());
        
        Mat r,t,slam_r,slam_t;
        // pose.rowRange(0,3).colRange(0,3).copyTo(r);
        // pose.colRange(0,3).col(3).copyTo(t);
        // slam_r = -pose.rowRange(0,2).
        Mat se_translation(3,1,CV_32FC1, tcw.translation().data());

        // hhh rowRange是左闭右开捏
        // 3*3的旋转矩阵×3*1的平移矩阵，得到世界坐标下的相机位置
        // slam_t = -pose.rowRange(0,3).colRange(0,3).t()*se_translation;
        slam_t = se_translation;

        R[0] = pose.at<float>(0,0);
        R[1] = pose.at<float>(1,0);
        R[2] = pose.at<float>(2,0);
        R[3] = pose.at<float>(0,1);
        R[4] = pose.at<float>(1,1);
        R[5] = pose.at<float>(2,1);
        R[6] = pose.at<float>(0,2);
        R[7] = pose.at<float>(1,2);
        R[8] = pose.at<float>(2,2);
        T[0] = slam_t.at<float>(0,0);
        T[1] = slam_t.at<float>(1,0);
        T[2] = slam_t.at<float>(2,0);
        
    }
}

extern "C"
{
    void SaveImage()
    {
        imwrite("/storage/emulated/0/4DAR/l.jpg", img_glob);
    }
}

extern "C"
{
    void SaveMap()
    {
        // SLAM.SaveAtlas(0);
    }
}
extern "C"
{
    void GetLocalMap()
    {
        ofstream out ;
        out.open("/storage/emulated/0/4DAR/cloud.txt");
        out << "# .PCD v0.7 - Point Cloud Data file format \n"
            << "VERSION 0.7\n"
            << "FIELDS x y z\n"
            << "SIZE 4 4 4 \n"
            << "TYPE F F F \n"
            << "COUNT 1 1 1\n"<<endl;
        // Atlas 这个类实在orbslam3作用域下声明的，所以要带上作用域或使用namespace
        // ORB_SLAM3::Atlas *atlas;
        vector<ORB_SLAM3::MapPoint*> vpmaps ;
        SLAM.GetLocalMap(vpmaps);
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
}

extern "C"
{
    void FitPlane(double plane_arg[])
    {
        double a,b,c,d;
        vector<ORB_SLAM3::MapPoint*> vpmaps ;
        SLAM.GetLocalMap(vpmaps);
        vector<Point3f> mapPoints;
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
        }
        ransac(mapPoints,100,2,a,b,c,d);
        plane_arg[0] = a;
        plane_arg[1] = b;
        plane_arg[2] = c;
        plane_arg[3] = d;
    }
}


void ransac(vector<Point3f>& pts_3d, int max_iter, float threshold,
double &a1,double &b1,double &c1,double &d1)
{
    int size_old = 3;
    double a,b,c,d; //

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
        double x1 = pts_3d.at(*idx).x;
        double y1 = pts_3d.at(*idx).y;
        double z1 = pts_3d.at(*idx).z;
        idx++;

        double x2 = pts_3d.at(*idx).x;
        double y2 = pts_3d.at(*idx).y;
        double z2 = pts_3d.at(*idx).z;
        idx++;

        double x3 = pts_3d.at(*idx).x;
        double y3 = pts_3d.at(*idx).y;
        double z3 = pts_3d.at(*idx).z;

        //根据三个点来拟合平面
        a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
		d = -(a*x1 + b*y1 + c*z1);

        for(auto iter = pts_3d.begin();iter != pts_3d.end(); ++iter)
        {
            double dis = fabs(a*iter->x 
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

//  test boost

extern "C"
{
    void GetBoost()
    {
        ofstream out_boost;
        out_boost.open("/storage/emulated/0/4DAR/boost.txt");
        out_boost <<"get:"   <<BOOST_LIB_VERSION <<endl;
        out_boost.close();


    }
}

extern "C"
{
    void SlamShutDown()
    {
        // save atlas
        SLAM.Shutdown();
    }
}