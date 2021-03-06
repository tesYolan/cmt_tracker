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

#include <cmt_tracker/Clear.h>

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

  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;
  std::vector<cv::Rect> faces;
  std::vector<cv::Rect> eyes;

  ros::NodeHandle nh_;
  cmt_tracker::Tracker track_location;
  ros::ServiceServer clear_service;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher tracker_locations_pub;
  ros::Publisher tracker_results_pub;
  ros::Subscriber face_results;
  ros::Subscriber tracker_subscriber;
  ros::Subscriber face_subscriber;
  cmt_tracker::Tracker tracker_set;
  cmt_tracker::Faces face_locs;
  cmt_tracker::Trackers trackers_results;
  std::vector<CMT> cmt;
  std::vector<int> quality_of_tracker;
  std::string tracking_method;
  bool setup;
  int last_count; 
  int jitterness; 

  std::vector<cv::Rect> locations_of_trackers; //for setting the tracking to higher levels.
  std::string subscribe_topic;
  sensor_msgs::ImagePtr masked_image;
  int frame_counters;
  int frame_previous;


  //Parameters for the CMT part of the code.
public:
  TrackerCMT() : it_(nh_)
  {
    //Here lies the essential parameters that we would set up to track the files.
    //Initally till things get to function pretty well let's do this nothing. Just subscribe to the images and publishing topics.
    jitterness=-1; 
    if ( !face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml" ) || !eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml" ))
    { setup = false;  };
    setup = true;
    nh_.getParam("camera_topic", subscribe_topic);
    last_count=0; 
    frame_counters = 0;
    frame_previous = 0;
    //To acquire the image that we will be doing processing on
    image_sub_ = it_.subscribe(subscribe_topic, 1, &TrackerCMT::imageCb, this);
    clear_service = nh_.advertiseService("clear", &TrackerCMT::clear, this);
    //To acquire the list of faces currently in track.
    face_subscriber = (nh_).subscribe("face_locations", 1, &TrackerCMT::list_of_faces_update, this);
    tracker_locations_pub = (nh_).advertise<cmt_tracker::Tracker>("tracking_locations", 10);

    //To acquire commands from this and other nodes to what to set to track.
    // masked_image = cv_bridge::CvImage(std_msgs::Header(), "mono", image_roi).toImageMsg();
    tracker_subscriber = (nh_).subscribe("tracking_locations", 1, &TrackerCMT::set_tracker, this);
    image_pub_ = it_.advertise("/transformed/images", 1);
    tracker_results_pub = nh_.advertise<cmt_tracker::Trackers>("tracker_results", 10);

  }

  bool clear(cmt_tracker::Clear::Request &req, cmt_tracker::Clear::Response &res)
  {
    cmt.clear();
    quality_of_tracker.clear();
    if (cmt.size() == 0)
    {
      // res.clear= true;
    }
    else {
      // res.clear = false;
    }
    return true;
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
    std::cout << "The number of tracker active is: " << cmt.size() << std::endl;
    int quality_update = 0;
    for (std::vector<CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
    {
      //Clear all the previous results relating to the tracker.

      cmt_tracker::Tracker tracker;
      if ((*v).initialized == true && !im_gray.empty())
      {
        (*v).processFrame(im_gray);
        cv::Rect rect = (*v).bb_rot.boundingRect();
        quality_of_tracker[quality_update] = (*v).num_active_keypoints;
        quality_update++;
        FILE_LOG(logDEBUG) << "Area ouptut is: " << rect.area();
        rect = rect & cv::Rect(0, 0, im_gray.size().width, im_gray.size().height);
        tracker.pixel_lu.x = rect.x;
        tracker.pixel_lu.y = rect.y;
        tracker.pixel_lu.z = 0;


        FILE_LOG(logDEBUG) << "Returned results with key point: " << cmt.back().num_active_keypoints;


        tracker.width.data = rect.width;
        tracker.height.data = rect.height;
        tracker.inital_points.data = (*v).num_initial_keypoints;
        tracker.active_points.data = (*v).num_active_keypoints;
        tracker.tracker_name.data = (*v).name;

        if (tracker.active_points.data > tracker.inital_points.data / 5)
        {
          FILE_LOG(logDEBUG) << "No suitable result";
          tracker.quality_results.data = true;
          trackers_results.tracker_results.push_back(tracker);
        }
        else {
          tracker.quality_results.data = false;
          trackers_results.tracker_results.push_back(tracker);
        }
      }
    }



    tracker_results_pub.publish(trackers_results);


    nh_.getParam("tracking_method", tracking_method);
    if (tracking_method.compare("sucessiveMA") == 0)
    {

      //Initally steps to setup the trackers.

      //First initialize the tracker with the the first face.

      //Here there seems to be a problem with the number of key points initally set make the system hard.
      for (std::vector<cmt_tracker::Face>::iterator v = face_locs.faces.begin(); v != face_locs.faces.end() ; ++v)
      {

        track_location.pixel_lu.x = (*v).pixel_lu.x;
        track_location.pixel_lu.y = (*v).pixel_lu.y;
        track_location.width.data = (*v).width.data;
        track_location.height.data = (*v).height.data;
        track_location.tracker_name.data = (*v).id.data;

//To Set the first Tracker--- FOr initial testing
        if (cmt.size() == 0)
          tracker_locations_pub.publish(track_location);
//Now let's do some processing on the image mat.


      }
      cv::Mat imageROI = im_gray.clone();
      for (std::vector<cmt_tracker::Tracker>::iterator v = trackers_results.tracker_results.begin(); v != trackers_results.tracker_results.end() ; ++v)
      {
        cv::Rect rect = cv::Rect((*v).pixel_lu.x, (*v).pixel_lu.y, (*v).width.data, (*v).height.data);
        cv::Rect normailze = rect & cv::Rect(0, 0, imageROI.size().width, imageROI.size().height);
        cv::Mat mask(normailze.size(), CV_16UC1);
        if (((*v).active_points.data)  >  ((*v).inital_points.data) * 0.4 )//If not start counter to the object for the nth frame
        {
          mask.setTo(cv::Scalar(5));
          mask.copyTo(imageROI(normailze));
        }
        
          //Mark the quality of this and wait to 5 frames for it. Then if the quality of this tracker doesn't reach good levels set's new tracker.

        
      }
      //Now the mask is small.

          cv::Mat hist_val; 
          cv::equalizeHist( imageROI, hist_val ); 
          face_cascade.detectMultiScale( hist_val, faces, 1.05, 4, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(40, 40) );
        
          for (size_t i = 0; i < faces.size(); i++)
        {
         cv::Mat face_Frame(hist_val(faces[i])); 
          // cv::cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
          // cv::equalizeHist( frame_gray, frame_gray );
          eyes_cascade.detectMultiScale( face_Frame, eyes, 1.1, 2, 0| cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );

          //Now if the above doesn't bring up two eyes then skip 
          //it 
          size_t eye_num= 2; 
          if(eyes.size() > 0)
          {
        track_location.pixel_lu.x = faces[i].x;
        track_location.pixel_lu.y = faces[i].y;
        track_location.width.data = faces[i].width;
        track_location.height.data = faces[i].height;
        track_location.tracker_name.data = "New Values";
        if(last_count==10)
        {
          tracker_locations_pub.publish(track_location);
          last_count=0; 
        }
        else //found an face wait for it 5. 
        last_count++; 
          }
          else //Not found a face. 
          {
            //Also for jitterness
            if(jitterness == -1)
            {
              jitterness =0;
            }
            else if(jitterness ==0)
            {
              jitterness = 1; 
            }
            else{
              jitterness=-1;
              last_count=0; 
            }
            
          }
        }
      masked_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageROI).toImageMsg();

      image_pub_.publish(masked_image);
      //Now let's visualize the results for a couple of seconds and if it's doesn't reach half
      //the inital active points we set a new tracker to it.

      //Now the rules are simple we overwrite the results of the image in the system.

    }
    trackers_results.tracker_results.clear();
  }

  void list_of_faces_update(const cmt_tracker::Faces& faces_info)
  {
    ROS_DEBUG("It get's here in the faces update");
    face_locs.faces.clear();
    //May be better to use an iterator to handle the function.
    for (int i = 0; i < faces_info.faces.size(); i++)
    {
      face_locs.faces.push_back(faces_info.faces[i]);
    }
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
      FILE_LOG(logDEBUG) << "initialized with intial key point: " << cmt.back().num_initial_keypoints;
      quality_of_tracker.push_back(cmt.back().num_initial_keypoints);
    }
    else
    {
      FILE_LOG(logDEBUG) << "Not initialized";
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