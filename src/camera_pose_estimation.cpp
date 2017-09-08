#include <ros/ros.h>
#include "imagerocessor.hpp"
#include "pnpsolver.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pose_estimation");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
  geometry_msgs::PoseStamped pose;
  ImageProcessor imageproc;
  PNPSolver pnpsolver;
  cv::VideoCapture capture(0);
  if (!capture.isOpened()) {
    return 1;
  }
  bool stop = false;
  cv::Mat frame, output;
  cv::Mat rvec, tvec;
  Rotation rt;
  Traslation tl;
  std::vector<std::vector<cv::Point>> features;
  cv::namedWindow("原图");
  int delay = 1000/ (double)capture.get(CV_CAP_PROP_FPS);
  ros::Rate rate(10);
  while (!stop) {
    if (!capture.read(frame)) {
      break;
    }
    cv::imshow("原图", frame);
    imageproc.process(frame, output, features);
    pnpsolver.solve(features, rt, tl);
    pose.pose.position.x = tl.tx;
    pose.pose.position.y = tl.ty;
    pose.pose.position.z = tl.tz;
    double qx = sin(rt.rz/2) * cos(rt.ry/2) * cos(rt.rz/2) - cos(rt.rz/2) * sin(rt.ry/2) * sin(rt.rz/2);
    double qy = cos(rt.rz/2) * sin(rt.ry/2) * cos(rt.rz/2) + sin(rt.rz/2) * cos(rt.ry/2) * sin(rt.rz/2);
    double qz = cos(rt.rz/2) * cos(rt.ry/2) * sin(rt.rz/2) - sin(rt.rz/2) * sin(rt.ry/2) * cos(rt.rz/2);
    double qw = cos(rt.rz/2) * cos(rt.ry/2) * cos(rt.rz/2) + sin(rt.rz/2) * sin(rt.ry/2) * sin(rt.rz/2);
    pose.pose.orientation.w = qw;
    pose.pose.orientation.x = qx;
    pose.pose.orientation.y = qy;
    pose.pose.orientation.z = qz;
    pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
  return 0;
}
