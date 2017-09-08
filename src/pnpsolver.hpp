/*
 * compute_pose_node.h
 *
 *  Created on: 2017年5月16日
 *      Author: dell
 */

#ifndef POSE_ESTIMATION_PKG_INCLUDE_POSE_ESTIMATION_PKG_COMPUTE_POSE_NODE_HPP_
#define POSE_ESTIMATION_PKG_INCLUDE_POSE_ESTIMATION_PKG_COMPUTE_POSE_NODE_HPP_
#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
struct Rotation{
  double rx;
  double ry;
  double rz;
};
struct Traslation{
  double tx;
  double ty;
  double tz;
};
class PointProcessor
{
public:
  virtual void solve(std::vector<std::vector<cv::Point> > & corners, Rotation & rt, Traslation & tl) = 0;
};

class PNPSolver : public PointProcessor{
public:
	//---PNP算法选择参数
	enum METHOD{
		ITERATIVE = CV_ITERATIVE,
		P3P = CV_P3P,
		EPNP = CV_EPNP
	};
  PNPSolver() : fx(167.156), fy(178.097), u0(155.89), v0(119.372), k_1(0), k_2(0), p_1(0), p_2(0), k_3(0), method(EPNP){

	}
	~PNPSolver(){

	}
  void solve(std::vector<std::vector<cv::Point> > & corners, Rotation & rt, Traslation & tl){
    setPoint2D(corners);
    setPoint3D();
    setCameraMatrix(fx, fy, u0, v0);
    setdistortionMatrix(k_1, k_2, p_1, p_2, k_3);
		//---数据校验
		if (camera_Matrix.cols == 0 || distortion_Matrix.cols == 0) {
      std::cout << "ErrCode:-1,摄像机内参数或畸变参数未设置" << std::endl;
		}
		if (Points2D.size() != Points3D.size()) {
      std::cout << "ErrCode:-2,3D点数量和2D点数量不一致" << std::endl;
		}
		if (method == ITERATIVE || method == P3P) {
			if (Points3D.size() != 4) {
        std::cout << "ErrCode:-2,使用CV_ITERATIVE或CV_P3P方法是特征点应为4" << std::endl;
			}
		}else
		{
			if (Points3D.size() < 4)
			{
        std::cout << "ErrCode:-2,输入的特征点数量应大于4！\r\n" << std::endl;
			}
		}
     cv:solvePnP(Points3D, Points2D,camera_Matrix, distortion_Matrix, rvec, tvec, false, method);
    //求解旋转角和坐标
    double rm[9];
    cv::Mat RoteM = cv::Mat(3, 3, CV_64FC1, rm);
    cv::Rodrigues(rvec, RoteM);
    double r11 = RoteM.ptr<double>(0)[0];
    double r12 = RoteM.ptr<double>(0)[1];
    double r13 = RoteM.ptr<double>(0)[2];
    double r21 = RoteM.ptr<double>(1)[0];
    double r22 = RoteM.ptr<double>(1)[1];
    double r23 = RoteM.ptr<double>(1)[2];
    double r31 = RoteM.ptr<double>(2)[0];
    double r32 = RoteM.ptr<double>(2)[1];
    double r33 = RoteM.ptr<double>(2)[2];
    //旋转角
    rt.rz = atan2(r21, r11) / CV_PI * 180;
    rt.ry = atan2(-1 * r31, sqrt(r32*r32 + r33*r33)) / CV_PI * 180;
    rt.rx = atan2(r32, r33) / CV_PI * 180;
    //坐标
    tl.tx = tvec.ptr<double>(0)[0];
    tl.ty = tvec.ptr<double>(0)[1];
    tl.tz = tvec.ptr<double>(0)[2];
  }
	//---设置摄像机内参数矩阵
	void setCameraMatrix(double fx, double fy, double u0, double v0){
		camera_Matrix = cv::Mat(3,3,CV_64FC1,cv::Scalar::all(0));
		camera_Matrix.ptr<double>(0)[0] = fx;
		camera_Matrix.ptr<double>(0)[2] = u0;
		camera_Matrix.ptr<double>(1)[1] = fy;
		camera_Matrix.ptr<double>(1)[2] = v0;
		camera_Matrix.ptr<double>(2)[2] = 1.0f;
	}
	//---设置摄像机畸变系数
	void setdistortionMatrix(double k_1, double k_2, double p_1, double p_2, double k_3){
		distortion_Matrix = cv::Mat(5,1,CV_64FC1,cv::Scalar::all(0));
		distortion_Matrix.ptr<double>(0)[0] = k_1;
		distortion_Matrix.ptr<double>(1)[0] = k_2;
		distortion_Matrix.ptr<double>(2)[0] = p_1;
		distortion_Matrix.ptr<double>(3)[0] = p_1;
		distortion_Matrix.ptr<double>(4)[0] = k_3;
	}
  void setPoint2D(const std::vector<std::vector<cv::Point> > &corners){
		for (int i = 0; i < corners.size(); ++i) {
			for (int j = 0; j < corners[i].size(); ++j) {
				Points2D.push_back(corners[i][j]);
			}
		}
	}
	void setPoint3D(){
		Points3D.push_back(cv::Point3f(0,0,0));
		Points3D.push_back(cv::Point3f(300,0,0));
		Points3D.push_back(cv::Point3f(300,300,0));
		Points3D.push_back(cv::Point3f(0,300,0));
		Points3D.push_back(cv::Point3f(600,225,0));
		Points3D.push_back(cv::Point3f(450,225,0));
		Points3D.push_back(cv::Point3f(450,75,0));
		Points3D.push_back(cv::Point3f(600,75,0));
		Points3D.push_back(cv::Point3f(75,450,0));
		Points3D.push_back(cv::Point3f(225,450,0));
		Points3D.push_back(cv::Point3f(225,600,0));
		Points3D.push_back(cv::Point3f(75,600,0));
	}
private:
  double fx, fy, u0, v0, k_1, k_2, p_1, p_2, k_3;
  cv::Mat rvec, tvec;
	//---图像角点坐标
  std::vector<cv::Point2f> Points2D;
	//---物体世界坐标
  std::vector<cv::Point3f> Points3D;
	//---摄像机内参数矩阵
	cv::Mat camera_Matrix;
	//---摄像机畸变参数矩阵
	cv::Mat distortion_Matrix;
  //方法枚举
  METHOD method;
};





#endif /* POSE_ESTIMATION_PKG_INCLUDE_POSE_ESTIMATION_PKG_COMPUTE_POSE_NODE_HPP_ */
