//This is the base file that will handle all the relevant part of processing
#include <cmt_tracker/Tracker.h>
#include <cmt_tracker/Trackers.h>
#include <cmt_tracker/Face.h>
#include <cmt_tracker/Faces.h>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/ImageConstPtr.h>

// CMT libraryies
#include "CMT.h"
#include "gui.h"
#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif
// CMT Libraries End

/**
The requirements of the this class:
1. This the function spawns the CMT instances.
  1.1 This is the fucntion that initalized.
  1.2 This is the one that publishes the location of the tracker results to the system.
  1.3 This is the fucntion that makes the decision on what to track and what not to track.

2. This is the fucntion that holds parameters for the tracker configuration and the one that would be modified when changes are made the plugin.
  2.1 When a person changes the configuration in the parameter server; this fucntion must change the way it performs it's tasks.
  2.2 When the CMT is initialized it's  must get the parameters from the systme so that it would perfrom in house calculation to maintain decent results.
  2.3 The tracking results are maintained here and updated correspondingly.

3. THIS FUNCTION MUST BE THREADED.

How i should proceed is this;

1st. Takeout everything from the tracker_plugin and make it in the cmt_tracker_node
  This step includes  1. Make the CMT_Tracker Listen to parameter settings and start processing the elements.
                      2. Make a detailed description of the output to rely to the other system.
                      3. Evaluate the performance of the system.
*/

using namespace cmt; 
class TrackerCMT
{

//to hold the value of the image
  cv::Mat conversion_mat_;
  cv::Mat frame_gray;
  std::vector<cv::Mat> tracked_images;


  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher tracker_results_pub;
  ros::Subscriber face_results;
  ros::Subscriber tracker_subscriber; 

  cmt_tracker::Tracker tracker_set;
  cmt_tracker::Faces face_locs;
  cmt_tracker::Trackers trackers_results;

  std::vector<CMT> cmt;
  std::vector<int> quality_of_tracker;

  std::vector<cv::Rect> locations_of_trackers; //for setting the tracking to higher levels.
  std::string subscribe_topic; 


  //Parameters for the CMT part of the code.
public:
  TrackerCMT() : it_(nh_)
  {
    //Here lies the essential parameters that we would set up to track the files.
    //Initally till things get to function pretty well let's do this nothing. Just subscribe to the images and publishing topics.
    nh_.getParam("camera_topic", subscribe_topic);

    //To acquire the image that we will be doing processing on
    image_sub_ = it_.subscribe(subscribe_topic, 1, &TrackerCMT::imageCb, this);

    //To acquire the list of faces currently in track. 
    // face_subscriber = (nh).subscribe("face_locations", 1, &list_of_faces_update, this);

    //To acquire commands from this and other nodes to what to set to track. 
    tracker_subscriber = (nh_).subscribe("tracking_locations", 1, &TrackerCMT::set_tracker, this);
    image_pub_ = it_.advertise("/transformed/images", 1);
    tracker_results_pub = nh_.advertise<cmt_tracker::Trackers>("tracker_results", 10);

  }
  /*
  This  function is a callback fucntion that happens when an image update occurs.
  */
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
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
        std::cerr << "Error in the conversion of the image" << std::endl;
        // qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
        // ui_.image_frame->setImage(QImage());
        return;
      }
    }
    //Do convert the image to gray scale as it's don't multiple times across other functions in the system.


      cv::cvtColor(conversion_mat_, frame_gray, CV_BGR2GRAY);

        cv::Mat im_gray = frame_gray.clone();
    std::cout<<"The number of tracker active is: "<<cmt.size()<<std::endl; 
    
    for (std::vector<CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
    {
      //Clear all the previous results relating to the tracker.
      cmt_tracker::Tracker tracker;
      if ((*v).initialized == true && !im_gray.empty())
      {
      (*v).processFrame(im_gray);
      cv::Rect rect = (*v).bb_rot.boundingRect();
      FILE_LOG(logDEBUG) << "Area ouptut is: "<<rect.area();
      tracker.pixel_lu.x = rect.x;
      tracker.pixel_lu.y = rect.y;
      tracker.pixel_lu.z = 0;

      tracker.width.data = rect.width;
      tracker.height.data = rect.height;
      tracker.inital_points.data = (*v).num_initial_keypoints;
      tracker.active_points.data= (*v).num_active_keypoints;  
      tracker.tracker_name.data = (*v).name;


      if(rect.area() > 50)
      {
        if ( 0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= im_gray.cols && 0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= im_gray.rows)
        {

      FILE_LOG(logDEBUG) <<"Returned results with key point: "<<cmt.back().num_active_keypoints;
      tracker.quality_results.data = true; 
      trackers_results.tracker_results.push_back(tracker);
    }
    else {
      FILE_LOG(logDEBUG) <<"No suitable result"; 
      tracker.quality_results.data = false; 
      trackers_results.tracker_results.push_back(tracker); 
    }
      }
      }

      //Now let's publish the results of the tracker.
    }
    cv::Mat area_tobe;  

    // for (int i = 0 ; i < trackers_results.tracker_results.size(); i++)
    // {
    // area_tobe= im_gray(cv::Rect(trackers_results.tracker_results[i].pixel_lu.x,trackers_results.tracker_results[i].pixel_lu.y, 
    //   trackers_results.tracker_results[i].width.data, trackers_results.tracker_results[i].height.data )).clone(); 
    // cv::imshow("i", area_tobe ); 
    // cv::waitKey(3);
    // }
    tracker_results_pub.publish(trackers_results); 
    trackers_results.tracker_results.clear(); 
  }

  void set_tracker(const cmt_tracker::Tracker& tracker_location)
  {
    //A potentially high penality task is done here but it's to avoid latter dealing with uncorrectly set trackers.

    FILE_LOG(logDEBUG) << "Initalizing Started ";
    cv::Mat im_gray = frame_gray.clone(); //To avoid change when being run.
    cv::Rect rect(tracker_location.pixel_lu.x, tracker_location.pixel_lu.y, tracker_location.width.data, tracker_location.height.data );
    if (!im_gray.empty() && rect.area() > 50)
    {
      //Now there must be some way to hold back setting up new tracker;
      cmt.push_back(CMT());
      cmt.back().consensus.estimate_rotation = true;
      std::string tracker_name = "Tracker : " + tracker_location.tracker_name.data ; 
      cmt.back().initialize(im_gray, rect, tracker_name);
      FILE_LOG(logDEBUG) <<"initialized with intial key point: "<<cmt.back().num_initial_keypoints;
    }
    else
    {
      FILE_LOG(logDEBUG) <<"Not initialized";
      // std::cout << "Not initialized" << std::endl;
    }


  }

  // void process()
  // {
  //   std::cout << "Entered Processing" << std::endl;
  //   cv::Mat im_gray = frame_gray().clone();
  //   for (std::vector<CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
  //   {
  //     //Clear all the previous results relating to the tracker.
  //     (*v).processFrame(im_gray);
  //     cv::Rect rect = (*v).bb_rot.boundingRect();
  //     cmt_tracker::Tracker tracker;
  //     tracker.pixel_lu.x = rect.x;
  //     tracker.pixel_lu.y = rect.y;
  //     tracker.pixel_lu.z = 0;

  //     tracker.width.data = rect.width;
  //     tracker.height.data = rect.height;

  //     tracker.name.data = (*v).name;

  //     trackers_results.tracker_results.push_back(tracker);

  //     //Now let's publish the results of the tracker.
  //   }
  //   tracker_results_pub.publish(trackers_results); 
  //   //This is a fucniton that performs that does the elements. 

  //   std::cout<<"Outputed results of processing"<<std::endl; 
  //   //Now publish the results of the tracking.

  // }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmt_tracker");
  TrackerCMT ic;
  ros::spin();
  return 0;
}