//This is the base file that will handle all the relevant part of processing
#include <cmt_tracker/Tracker.h>
#include <cmt_tracker/Face.h>
#include <cmt_tracker/Faces.h>

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
#include <cmt_tracker/Faces.h>

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

  cmt_tracker::Tracker tracker_set;
  cmt_tracker::Faces face_locs;
  cmt_tracker::Trackers trackers_results;

  std::vector<CMT> cmt;
  std::vector<int> quality_of_tracker;

  std::vector<cv::Rect> locations_of_trackers; //for setting the tracking to higher levels.


  //Parameters for the CMT part of the code.
public:
  TrackerCMT() : it_(nh_)
  {
    //Here lies the essential parameters that we would set up to track the files.
    //Initally till things get to function pretty well let's do this nothing. Just subscribe to the images and publishing topics.
    nh_.getParam("camera_topic", subscribe_topic);
    image_sub_ = it_.subscribe(subscribe_topic, 1, &TrackerCMT::imageCb, this);
    face_subscriber = (nh).subscribe("face_locations", 1, &list_of_faces_update, this);
    tracker_subscriber = (nh).subscribe("track_locations", 1, &set_tracker, this);
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
    cv::cvtColor(conversion_mat_, frame_gray, cv::COLOR_BGR2GRAY);
  }

  void set_tracker(const cmt_tracker::Tracker& tracker_location)
  {
    //A potentially high penality task is done here but it's to avoid latter dealing with uncorrectly set trackers.

    std::cout << "Initalizing tracker with values" << std::endl;
    cv::Mat im_gray = frame_gray.clone(); //To avoid change when being run.
    cv::Rect rect(tracker_location.pixel_lu.x, tracker_location.pixel_lu.y, tracker_location.width.data, tracker_location.height.data );
    if (!im_gray.empty() && rect.area() > 50)
    {
      //Now there must be some way to hold back setting up new tracker;
      cmt.push_back(CMT());
      cmt.back().consensus.estimate_rotation = true;
      cmt.back().initialize(im_gray, rect, tracker_location.tracker_name.data);
    }
    else
    {
      std::cout << "Not initialized" << std::endl;
    }
    std::cout << "Initalizing Done in the system. " << std::endl;

  }

  void process()
  {
    std::cout << "Entered Processing" << std::endl;
    cv::Mat im_gray = frame_gray().clone();
    for (std::vector<CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
    {
      //Clear all the previous results relating to the tracker.
      (*v).processFrame(im_gray);
      cv::Rect rect = (*v).bb_rot.boundingRect();
      cmt_tracker::Tracker tracker;
      tracker.pixel_lu.x = rect.x;
      tracker.pixel_lu.y = rect.y;
      tracker.pixel_lu.z = 0;

      tracker.width.data = rect.width;
      tracker.height.data = rect.height;

      tracker.name.data = (*v).name;

      trackers_results.tracker_results.push_back(tracker);

      //Now let's publish the results of the tracker.
    }
    tracker_results_pub.publish(trackers_results); 
    //This is a fucniton that performs that does the elements. 

    std::cout<<"Outputed results of processing"<<std::endl; 
    //Now publish the results of the tracking.

  }

  void rules()
  {

  }

  virtual void spinOnce ()
  {
    process()
    spinOnce();
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmt_tracker");
  TrackerCMT ic;
  ic.spin();
  return 0;
}