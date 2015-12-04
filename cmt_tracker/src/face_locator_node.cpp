//This is the fucntion that is responsible to
//making face detections and tracking decisions for cmt_tracker_node.

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//CMT_TRACKER name structure;
#include <cmt_tracker/Face.h>
#include <cmt_tracker/Faces.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <iostream>
static const std::string OPENCV_WINDOW = "Image window";
class ImageConverter
{
  cv::Mat conversion_mat_;
  int counter;
  geometry_msgs::Point face_points;
  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //List of face values in space.
  ros::Publisher faces_locations;
  std::vector<cv::Rect> faces;
  std::vector<cv::Rect> eyes; 
  cmt_tracker::Faces cmt_face_locations;
  int time_sec;
  std_msgs::String tracking_method;
  bool setup;
public:
  ImageConverter()
    : it_(nh_)
  {
    counter = 0;
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1,
                               &ImageConverter::imageCb, this);
    // image_pub_ = it_.advertise("/image_converter/output_video", 1);
    faces_locations = nh_.advertise<cmt_tracker::Faces>("face_locations", 10);
    //Later refactor this to load from the xml documents of the file.
    if ( !face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml" ) || !eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml" ))
    { setup = false;  };
    setup = true;
    // cv::namedWindow(OPENCV_WINDOW);
    //Get time it takes to set the tracking values.
    nh_.getParam("tracker_set_time", time_sec);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    if (setup)
    {

      try
      {
        // First let cv_bridge do its magic
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        conversion_mat_ = cv_ptr->image;
      }
      catch (cv_bridge::Exception& e)
      {
        try
        {
          // If we're here, there is no conversion that makes sense, but let's try to imagine a few first
          cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
          if (msg->encoding == "CV_8UC3")
          {
            // assuming it is rgb
            conversion_mat_ = cv_ptr->image;
          } else if (msg->encoding == "8UC1") {
            // convert gray to rgb
            cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
          }  else {
            // qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
            std::cerr << "Error in the conversion of the iamge" << std::endl;
            // ui_.image_frame->setImage(QImage());
            return;
          }
        }
        catch (cv_bridge::Exception& e)
        {
          std::cerr << "Error in the conversion of the iamge" << std::endl;
          // qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
          // ui_.image_frame->setImage(QImage());
          return;
        }
      }

      // Draw an example circle on the video stream
      // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //   cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

      cv::Mat frame_gray;
      cv::cvtColor(conversion_mat_, frame_gray, cv::COLOR_BGR2GRAY);
      cv::equalizeHist( frame_gray, frame_gray );

      face_cascade.detectMultiScale( frame_gray, faces, 1.05, 4, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(40, 40) );


      //TODO: namespace mapping to the system.
      std::string tracking;
      nh_.getParam("tracking_method", tracking);
      //Process tracking_method.data
      if (tracking.compare("handtracking") == 0)
      {
        ROS_DEBUG("Using Methods of Hand Tracking Methods");
        std::cout<<"Using Methods of Hand Tracking Methods"<<std::endl; 
        for (size_t i = 0; i < faces.size(); i++)
        {

          cmt_tracker::Face face_description;
          face_description.pixel_lu.x = faces[i].x;
          face_description.pixel_lu.y = faces[i].y;
          //Now place coordinates to the value Z from the
          //depth camera.
          face_description.pixel_lu.z = 0;
          face_description.height.data = faces[i].height;
          face_description.width.data = faces[i].width;
          face_description.quality.data = true;
          face_description.id.data = counter;
          counter++;
          face_description.name.data = "Name";
          face_description.emotion_states.data = "Neutral";

          cmt_face_locations.faces.push_back(face_description);

          ros::Rate rate(2);

        }
        faces_locations.publish(cmt_face_locations);
        cmt_face_locations.faces.clear();
      }
      else if(tracking.compare("eyes") == 0)
      {
        ROS_DEBUG("Using Include Eye Results to stand in locations");
        std::cout<<"Eyes Detection: Using"<<std::endl; 
        for (size_t i = 0; i < faces.size(); i++)
        {

          // Now here what is different is to take
          //
          cv::Mat face_Frame(frame_gray(faces[i])); 
          // cv::cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
          // cv::equalizeHist( frame_gray, frame_gray );
          eyes_cascade.detectMultiScale( frame_gray, eyes, 1.1, 2, 0| cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );

          //Now if the above doesn't bring up two eyes then skip 
          //it 
          size_t eye_num= 2; 
          if(eyes.size() == eye_num)
          {
          cmt_tracker::Face face_description;

          face_description.pixel_lu.x = faces[i].x;
          face_description.pixel_lu.y = faces[i].y;
          //Now place coordinates to the value Z from the
          //depth camera.
          face_description.pixel_lu.z = 0;
          face_description.height.data = faces[i].height;
          face_description.width.data = faces[i].width;
          face_description.quality.data = true;
          face_description.id.data = counter;
          counter++;
          face_description.name.data = "Name";
          face_description.emotion_states.data = "Neutral";

          cmt_face_locations.faces.push_back(face_description);
          }
          ros::Rate rate(2);

        }
        faces_locations.publish(cmt_face_locations);
        cmt_face_locations.faces.clear();

      }
      else
      {
        std::cout<<"Please Choose a method of tracking"<<std::endl; 
      }
      // Update GUI Window
      // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
      // cv::waitKey(3);

      // Output modified video stream
      // image_pub_.publish(cv_ptr->toImageMsg());
    }
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "face_locator");
  ImageConverter ic;
  ros::spin();
  return 0;
}