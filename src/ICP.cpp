//
// Created by sophda on 11/29/23.
//
#include <Eigen/Eigen>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
void ReadPcd(std::string filename, std::vector<Eigen::Vector3f>& p1){

    Eigen::Vector3f point;

    std::ifstream fin(filename);
    std::string line;
    for(int i=0;i<13;i++)
    {
        getline(fin,line);
    }
    while (getline(fin, line)) {
        std::stringstream ss(line);

        ss >> point.x();
        ss >> point.y();
        ss >> point.z();
//        std::cout<<point.x();
//        p2.push_back(point);
        p1.push_back(point);
    }
    fin.close();
}


void ICP(const std::vector<Eigen::Vector3f>& p1, const std::vector<Eigen::Vector3f>& p2,
         Eigen::Matrix3f& R_12, Eigen::Vector3f& t_12)
{

//    assert(p1.size() == p2.size());

    // center of mass
    size_t N = p1.size();
    Eigen::Vector3f p1_center, p2_center;
    p1_center = Eigen::Vector3f(0,0,0);
    p2_center = Eigen::Vector3f(0,0,0);
    for (int i = 0; i < N; ++i) {
        p1_center += p1.at(i);
        p2_center += p2.at(i);
    }
    p1_center /= N;
    p2_center /= N;

    // remove the center
    std::vector<Eigen::Vector3f> q1(N), q2(N);
    for (int i = 0; i < N; ++i) {
        q1[i] = p1.at(i) - p1_center;
        q2[i] = p2.at(i) - p2_center;
    }

    // compute q2*q1^T
    Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
    for (int i = 0; i < N; ++i) {
        H += q2.at(i) * q1.at(i).transpose();
    }

    // SVD on H
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f U = svd.matrixU();
    Eigen::Matrix3f V = svd.matrixV();

    R_12 = V * U.transpose();
    t_12 = p1_center - R_12 * p2_center;

}
