//CMT libraryies
#include "CMT.h"
#include "gui.h"
#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif
//CMT Libraries End

//ROS and OpenCV Related Libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;
using ::atof;
static Scalar randomColor( cv::RNG& rng );

cv::Point face_location; 
cv::RNG rng( 0xFFFFFFFF );
void CallBackFunc(int event, int x, int y, int flags, void* click_points)
{
    // cv::Point click_point; 
     cv::Point *click_point= (cv::Point*)click_points;
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {  
            face_location.x= x; 
            face_location.y= y;
            click_point->x = x;
            click_point->y= y;
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          // return click_point;
     }
     else
     {
        face_location.x=-1;
        face_location.y=-1 ;
        click_point->x = -1;
        click_point->y= -1; 
     }
     // return click_point;
}

int display(Mat im, CMT & cmt)
{
    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    for(size_t i = 0; i < cmt.points_active.size(); i++)
    {
        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
    }

    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        line(im, vertices[i], vertices[(i+1)%4],Scalar(255,0,0));
        //line(im, vertices[i], vertices[(i+1)%4], randomColor(rng));
    }

    imshow(WIN_NAME, im);

    return waitKey(5);
}

static Scalar randomColor( cv::RNG& rng )
{
  int icolor = (unsigned) rng;
  return Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}




void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}