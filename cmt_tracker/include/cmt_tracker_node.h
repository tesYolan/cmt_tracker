#ifndef CMT_TRACKER_H
#define CMT_TRACKER_H

#include <cmt_tracker_msgs/Tracker.h>
#include <cmt_tracker_msgs/Trackers.h>
#include <cmt_tracker_msgs/Face.h>
#include <cmt_tracker_msgs/Faces.h>
#include <cmt_tracker_msgs/TrackedImages.h>

//OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//ROS - OpenCV libraries
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/ImageConstPtr.h>

//Service call to reset the values of the images in the system
#include <cmt_tracker_msgs/Clear.h>
#include <cmt_tracker_msgs/Update.h>
#include <pi_face_tracker/Face.h>
#include <pi_face_tracker/Faces.h>
#include <pi_face_tracker/FaceEvent.h>
#include <cmt_tracker_msgs/TrackerConfig.h>
#include <dynamic_reconfigure/server.h>

#include <stdlib.h>
#include <time.h>

// CMT libraryies
#include "CMT.h"
#include "gui.h"
#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif
// CMT Libraries End

//Threading Libraries
#include <boost/thread.hpp>
#include <string>
#include <sstream>
#include <cmath> 
#define SSTR( x ) dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x ) ).str()

//This is the base file that will handle all the relevant part of processing

//Messages for holding tracker and face locations


/**
This a boiler plate function for the cmt wrapping. Now the way this is constructed is to be utilized by any ROS node that wants to track 
a certain element in space. 
*/

namespace cmt_wrap {

struct camera_properties
{
	int width; 
	int height; 
	double fov; 
	int offset; 
} camera_config;

class TrackerCMT
{

public: 
	//constructor
	TrackerCMT();


	//Services
	//Remove all the tracking instances of the system. 
	bool clear(cmt_tracker_msgs::Clear::Request &req, cmt_tracker_msgs::Clear::Response &res); 

	//Check if there are changes in what is being tracke in cmt arrays. 
	bool updated(cmt_tracker_msgs::Update::Request &req, cmt_tracker_msgs::Update::Response &res);

	//Get the list of images from the cmt instances that we are tracking
	bool getTrackedImages(cmt_tracker_msgs::TrackedImages::Request &req,cmt_tracker_msgs::TrackedImages::Response &res); 


	//Callbacks for subscribers

	//image subscriber
	void imageCb(const sensor_msgs::ImageConstPtr& msg);  

	//dynamic reconfigure subscriber
	void list_of_faces_update(const cmt_tracker_msgs::Faces& faces_info);

	//Subscriber to set the cmt to location specified by the tracker_location
	void set_tracker(const cmt_tracker_msgs::Tracker& tracker_location); 

	//Subscriber to set the cmt to locations specified by the tracker locations
	void set_trackers(const cmt_tracker_msgs::Trackers& tracker_location); 

	//A function that removes the ith element from the vector of trackers. 
	void remove_tracker(int index); 

	void callback(cmt_tracker_msgs::TrackerConfig &config, uint32_t level); 

	//Rules
	void deleteOnLost(std::vector<cv::Rect> tracked); 



private: 


	ros::NodeHandle nh_;

	//To hold the images of from image callback;
	cv::Mat conversion_mat_;
	cv::Mat frame_gray;

	//To hold the values of image returns from internal states of cmt. 
	std::vector<cv::Mat> tracked_images;

	//To listen to dynamic reconfigure elements. 
	dynamic_reconfigure::Server<cmt_tracker_msgs::TrackerConfig> server;
	dynamic_reconfigure::Server<cmt_tracker_msgs::TrackerConfig>::CallbackType f;


	//Need to be removed
	cv::CascadeClassifier face_cascade;
	cv::CascadeClassifier eyes_cascade;
	std::vector<cv::Rect> faces;
	std::vector<cv::Rect> eyes;

	
	cmt_tracker_msgs::Tracker track_location;
	cmt_tracker_msgs::Trackers trackers_results;
	cmt_tracker_msgs::Tracker tracker_set;
	cmt_tracker_msgs::Faces face_locs;


	ros::ServiceServer clear_service;
	ros::ServiceServer image_service;
	ros::ServiceServer update_service;

	image_transport::ImageTransport it_;

	image_transport::Subscriber image_sub_;

	//Probably need to be enabled for debuggin puproses. 
	image_transport::Publisher image_pub_;

	image_transport::Publisher image_face_pub;

	//tracker results locations. 
	ros::Publisher tracker_locations_pub;
	ros::Publisher tracker_results_pub;
	ros::Publisher pi_vision_results; 
	ros::Publisher pi_events; 

	//Now this is face results; Moving to another function. 
	ros::Subscriber face_results;

	//Now this is how to set the locator functions. For single tracker and multiple set
	ros::Subscriber tracker_subscriber;
	ros::Subscriber trackers_subscriber;

	//what is this
	ros::Subscriber face_subscriber;

	
	//an array of that holds the functions of the value. 
	std::vector<cmt::CMT> cmt;
	// std::vector<int> quality_of_tracker;

	//Holds what item has lost tracking state. 
	std::vector<int> poorly_tracked;

	//premove
	std::string tracking_method;
	bool setup;
	int last_count;

	//the threshold value that is passed to the cmt instance. 
	double factor;

	//for setting the tracking to higher levels.
	std::vector<cv::Rect> locations_of_trackers; 
	std::string subscribe_topic;

	//for debugging purposes. 
	sensor_msgs::ImagePtr masked_image;

	//for debugging purposes. 
	int frame_counters;
	int frame_previous;

	//value of the system. 
	bool update_ui;
	bool update_ui_wait; 

	bool clear_cmt; 


};
namespace {
  std::vector<cv::Rect> facedetect(cv::Mat frame_gray);
  cmt_tracker_msgs::Trackers convert(std::vector<cv::Rect> faces); 
  cmt_tracker_msgs::Trackers returnOverlapping(std::vector<cv::Rect> cmt_locations, cmt_tracker_msgs::Faces facelocs); 
  pi_face_tracker::Face returnPiMessage(cmt_tracker_msgs::Tracker loc, camera_properties camera_config); 
  pi_face_tracker::Faces returnPiMessages(cmt_tracker_msgs::Trackers locs, camera_properties camera_config); 
  pi_face_tracker::FaceEvent returnPiEvents(std::string event, std::string face_id);
  cmt_tracker_msgs::Trackers returnOverlappingEmotion(cmt_tracker_msgs::Trackers locs, cmt_tracker_msgs::Faces facelocs);
}
}
#endif // CMT_TRACKER_H