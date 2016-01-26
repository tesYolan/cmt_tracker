#ifndef FACE_LOCATOR_H
#define FACE_LOCATOR_H
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//CMT_TRACKER name structure;
#include <cmt_tracker_msgs/Face.h>
#include <cmt_tracker_msgs/Faces.h>
#include <cmt_tracker_msgs/Trackers.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>

//dlib libraries to check for better results
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
//=====
#define NO_MAKEFILE
#define ENABLE_ASSERTS
#define SSTR( x ) dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x ) ).str()

namespace face_detect {
class Face_Detection {

public:
  Face_Detection();
  //~Face_Detection();

  void imageCb(const sensor_msgs::ImageConstPtr& msg); 

private:
  cv::Mat conversion_mat_;
  int counter;
  geometry_msgs::Point face_points;

  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;

  dlib::frontal_face_detector detector; 

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //List of face values in space.
  ros::Publisher faces_locations;
  std::vector<cv::Rect> faces;
  std::vector<cv::Rect> eyes;
  cmt_tracker_msgs::Faces cmt_face_locations;
  std::string subscribe_topic;
  int time_sec;
  std_msgs::String tracking_method;
  std_msgs::String face_detection_method; 
  bool setup;
};
//static class that may be called by implementing class. 
namespace {
  std::vector<cv::Rect> facedetect(cv::Mat frame_gray);
  cmt_tracker_msgs::Trackers convert(std::vector<cv::Rect> faces); 
  cmt_tracker_msgs::Trackers returnOverlapping(std::vector<cv::Rect> cmt_locations, cmt_tracker_msgs::Faces facelocs); 
}
}
#endif // FACE_LOCATOR_H