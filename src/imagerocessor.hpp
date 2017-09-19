/*
 * pose_estimation_node.h
 *
 *  Created on: 2017年5月15日
 *      Author: dell
 */

#ifndef POSE_ESTIMATION_PKG_INCLUDE_POSE_ESTIMATION_NODE_H_
#define POSE_ESTIMATION_PKG_INCLUDE_POSE_ESTIMATION_NODE_H_
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <iostream>
//图像帧处理类接口
class FrameProcessor
{
public:
  virtual bool process(cv::Mat & input, cv::Mat & output, std::vector<std::vector<cv::Point> > & corners) = 0;
};
class ImageProcessor : public FrameProcessor{
public:
	//---默认构造函数
  ImageProcessor(){

	}
	//---图像预处理
  bool process(cv::Mat & frame, cv::Mat & output, std::vector<std::vector<cv::Point> > & corners){
		//---原图转化为灰度图
    cv::cvtColor(frame,grayImage,CV_BGR2GRAY);
		//---中值滤波
    cv::medianBlur(grayImage,blurImage,7);
		//---边缘检测
    cv::Canny(blurImage,output,150,50,3);
    findRectangle(output);
    if (maxcontours.size() != 3) {
      return false;
    } else {
      cornerSort();
      findcorners();
      cornerSort();
      return true;
    }
  }
  void findRectangle(cv::Mat & output){
    double contArea = 0.0;
    double minAreaValue = 100.0;
    cv::findContours(output,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    //---轮廓筛选，获取大于阀值的轮廓
    for (int i = 0; i < contours.size(); ++i) {
      contArea = cv::contourArea(contours[i]);
      if (contArea > minAreaValue) {
        maxcontours.push_back(contours[i]);
      }
    }
  }
  void contourSort(){
    cv::Point2f mc0 = centerOfContours(maxcontours[0]);
    for (int i = 0; i < maxcontours.size(); ++i) {
      for (int j = i + 1; j < maxcontours.size(); ++j) {
        if(contourArea(maxcontours[i]) < contourArea(maxcontours[j])){
          maxcontours[i].swap(maxcontours[j]);
        }
      }
    }
    for (int i = 0; i < maxcontours.size(); ++i) {
      for (int j = i + 1; j < maxcontours.size(); ++j) {
        cv::Point2f mc1 = centerOfContours(maxcontours[i]);
        cv::Point2f mc2 = centerOfContours(maxcontours[j]);
        int det = (mc1.x - mc0.x) * (mc2.y - mc0.y) - (mc2.x - mc0.x) * (mc1.y - mc0.y);
        if(det < 0){
          (maxcontours[i]).swap(maxcontours[j]);
        }
      }
    }
  }
//  void drawImage(cv::Mat & output){
//    output = cv::Mat::zeros( output.size(), CV_8UC1 );
//    cv::drawContours(output, maxcontours, -1, cv::Scalar(255), 1);
//    for (int i = 0; i < maxcontours.size(); ++i) {
//      cv::Point2f center  = centerOfContours(maxcontours[i]);
//      cv::circle(output,center,2,cv::Scalar(80,80,120),-1);
//      cv::putText(output,"O",center,1,1,cv::Scalar(120,120,80),1);
//    }
//  }
//  void drawcorner(cv::Mat & output){
//    std::string label[3][4] = {{"1","2","3","4"},{"5","6","7","8"},{"9","10","11","12"}};
//    for (int i = 0; i < corners.size(); ++i) {
//      for (int j = 0; j < corners[i].size(); ++j) {
//        cv::circle(output,corners[i][j],2,cv::Scalar(0,0,255),-1);
//        cv::putText(output,label[i][j],corners[i][j],1,1,cv::Scalar(255,0,0),1);
//      }
//    }
//  }
  void findcorners(){
    double distance = 0.0;  double maxDistance = 0.0;
    double leftArea = 0.0;  double maxLeftArea = 0.0;
    double rightArea = 0.0; double maxRightArea = 0.0;
    double kk = 0.0;
    double bb = 0.0;
    //---寻找对角点
    for (int i = 0; i < maxcontours.size(); ++i) {
      distance = 0.0;  maxDistance = 0.0;
      leftArea = 0.0;  maxLeftArea = 0.0;
      rightArea = 0.0; maxRightArea = 0.0;
      corners.push_back(std::vector<cv::Point>(4,cv::Point(0,0)));
      for (int j = 0; j < maxcontours[i].size(); ++j) {
        for (int k = j + 1; k < maxcontours[i].size(); ++k) {
          distance = pointToPointDist(maxcontours[i][j],maxcontours[i][k]);
          if (distance > maxDistance) {
            maxDistance = distance;
            corners[i][0] = maxcontours[i][j];
            corners[i][1] = maxcontours[i][k];
          } else { ; }
        }
      }
      //---求两点直线方程
      if (corners[i][1].x == corners[i][0].x) {
        kk = 0;
      } else {
        kk = (double)(corners[i][1].y - corners[i][0].y) / (corners[i][1].x - corners[i][0].x);
      }
      bb = corners[i][1].y - kk * corners[i][1].x;
      //---求其他对角点
      for (int j = 0; j < maxcontours[i].size(); ++j) {
        if ((double)maxcontours[i][j].x * kk + bb > maxcontours[i][j].y) {
          rightArea = threePointArea(corners[i][0],corners[i][1],maxcontours[i][j]);
          if (rightArea > maxRightArea) {
            maxRightArea = rightArea;
            corners[i][2] = maxcontours[i][j];
          }
        } else {
          leftArea = threePointArea(corners[i][0],corners[i][1],maxcontours[i][j]);
          if (leftArea > maxLeftArea) {
            maxLeftArea = leftArea;
            corners[i][3] = maxcontours[i][j];
          }
        }
      }
    }
  }
  void cornerSort(){
    for (int i = 0; i < corners.size(); ++i) {
      for (int j = 0; j < corners[i].size(); ++j) {
        for (int k = j + 1; k < corners[i].size(); ++k) {
          if(distanceSort(corners[i][j]) < distanceSort(corners[i][k])){
            cv::Point temp = corners[i][j];
            corners[i][j] = corners[i][k];
            corners[i][k] = temp;
          }
        }
      }
    }
    for (int i = 0; i < corners.size(); ++i) {
      for (int j = 0; j < corners[i].size(); ++j) {
        for (int k = j + 1; k < corners[i].size(); ++k) {
          int det = (corners[i][j].x - corners[i][0].x) * (corners[i][k].y - corners[i][0].y) -
                (corners[i][k].x - corners[i][0].x) * (corners[i][j].y - corners[i][0].y);
          if (det < 0) {
            cv::Point temp = corners[i][j];
            corners[i][j] = corners[i][k];
            corners[i][k] = temp;
          }
        }
      }
    }
  }
  cv::Point2f centerOfContours (const std::vector<cv::Point> &contour){
    cv::Moments mu;
    mu = cv::moments( contour, false );
    //---计算中心矩:
    cv::Point2f mc;
    mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
    return mc;
  }
  double distanceSort(const cv::Point &point1){
    double distance = 0.0;
    double sum = 0.0;
    for (int i = 0; i < corners.size(); ++i) {
      for(int j = 0; j < corners[i].size(); ++j)
      distance = pointToPointDist(point1,corners[i][j]);
      sum = sum + distance;
    }
    return sum;
  }
  double pointToPointDist(const cv::Point &first,const cv::Point &second){
    double distance = 0.0;
    distance = (double)(second.x - first.x) * (double)(second.x - first.x) +
           (double)(second.y - first.y) * (double)(second.y - first.x);
    return distance;
  }
  double threePointArea(const cv::Point &first,const cv::Point &second,const cv::Point &third){
    double a = sqrt((double)(second.x-first.x)*(double)(second.x-first.x)+(double)(second.y-first.y)*(double)(second.y-first.y));
    double b = sqrt((double)(third.x-first.x)*(double)(third.x-first.x)+(double)(third.y-first.y)*(double)(third.y-first.y));
    double c = sqrt((double)(third.x-second.x)*(double)(third.x-second.x)+(double)(third.y-second.y)*(double)(third.y-second.y));
    double s = (a+b+c)/2.0;
    double area = sqrt(s * (s-a) * (s-b) * (s-c));
    return area;
  }
private:
  //---灰度图像修改
  cv::Mat grayImage;
  //---滤波图像
  cv::Mat blurImage;
  //轮廓向量
  std::vector<std::vector<cv::Point> > contours;
  //按轮廓面积排序
  std::vector<std::vector<cv::Point> > maxcontours;
  //角点
  std::vector<std::vector<cv::Point> > corners;

};
#endif /* POSE_ESTIMATION_PKG_INCLUDE_POSE_ESTIMATION_NODE_H_ */
