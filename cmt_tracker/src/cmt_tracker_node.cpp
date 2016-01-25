#include "cmt_tracker_node.h"
#include "face_locator_node.h"
namespace cmt_wrap {

TrackerCMT::TrackerCMT() : it_(nh_)
{

  if ( !face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml" ) || !eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml" ))
  { setup = false;  };
  setup = true;
  nh_.getParam("camera_topic", subscribe_topic);
  last_count = 0;
  frame_counters = 0;
  frame_previous = 0;
  update_ui = false;
  //To acquire the image that we will be doing processing on
  image_sub_ = it_.subscribe(subscribe_topic, 1, &cmt_wrap::TrackerCMT::imageCb, this);


  //Service to deal with the image systems.
  clear_service = nh_.advertiseService("clear", &cmt_wrap::TrackerCMT::clear, this);
  image_service = nh_.advertiseService("get_cmt_rects", &cmt_wrap::TrackerCMT::getTrackedImages, this);
  update_service = nh_.advertiseService("update", &cmt_wrap::TrackerCMT::updated, this);

  //To acquire the list of faces currently in track.
  face_subscriber = (nh_).subscribe("face_locations", 1, &cmt_wrap::TrackerCMT::list_of_faces_update, this);

  //this is the one that creates what add to track based on the overlay of previous cmt tracking results and faces in face_locator_node
  tracker_locations_pub = (nh_).advertise<cmt_tracker_msgs::Trackers>("tracking_locations", 10);


  //To acquire commands from this and other nodes to what to set to track.
  // masked_image = cv_bridge::CvImage(std_msgs::Header(), "mono", image_roi).toImageMsg();
  tracker_subscriber = (nh_).subscribe("tracking_location", 1, &cmt_wrap::TrackerCMT::set_tracker, this);
  trackers_subscriber = (nh_).subscribe("tracking_locations", 1, &cmt_wrap::TrackerCMT::set_trackers, this);

  
  //image_pub_ = it_.advertise("/transformed/images", 1);
  //image_face_pub = it_.advertise("/transformed/faces", 1);

  tracker_results_pub = nh_.advertise<cmt_tracker_msgs::Trackers>("tracker_results", 10);
  
  pi_vision_results = nh_.advertise<pi_face_tracker::Faces>("pi_results", 10); 
  pi_events = (nh_).advertise<pi_face_tracker::FaceEvent>("pi_events", 10); 
  f = boost::bind(&cmt_wrap::TrackerCMT::callback, this, _1, _2);

  server.setCallback(f);

  //Now let's read the camera pictures form the system. 
  camera_config.width = 640; 
  camera_config.height = 480; 
  camera_config.fov = 0.625; 
}

bool TrackerCMT::clear(cmt_tracker_msgs::Clear::Request &req, cmt_tracker_msgs::Clear::Response &res)
{
  cmt.clear();
  quality_of_tracker.clear();
  if (cmt.size() == 0)
  {
    res.cleared = true;
  }
  else
  {
    res.cleared = false;
  }
  //update ui. 
  nh_.setParam("tracker_updated", 2); 
  return true;
}
//this is a service that indicates wheteher new elements are going to be tracked or note.
bool TrackerCMT::updated(cmt_tracker_msgs::Update::Request &req, cmt_tracker_msgs::Update::Response &res)
{
  res.update.data = false;
  if (update_ui)
  {
    res.update.data = true;
    update_ui = false;
  }
  return true;
}
/*
Request the images in the system.
*/
bool TrackerCMT::getTrackedImages(cmt_tracker_msgs::TrackedImages::Request &req,
                                  cmt_tracker_msgs::TrackedImages::Response &res)
{
  //Now this get's the name of elements in one iteration in a single copy method.
  for (std::vector<cmt::CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
  {

    res.names.push_back((*v).name);

    sensor_msgs::ImagePtr masked_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", (*v).imArchive).toImageMsg();


    //Why not publish the image here and see it's corresponding response regarding how to implement the best form of the system.
    //image_face_pub.publish(masked_image);

    res.image.push_back(*masked_image);

  }
  return true;
}
/*
This  function is a callback function that happens when an image update occurs.
*/
void TrackerCMT::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  //update_ui = false;
  //TODO why not just subscribe to the MONO image and skip all this handling. A way must be included to do that functionality.
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
  //Now let's copy an additional mat to do additional removal.
  cv::Mat im_masked = frame_gray.clone();
  std::cout << "The number of tracker active is: " << cmt.size() << std::endl;
  int quality_update = 0;
  int count = 0;
  poorly_tracked.clear();
  std::vector<cv::Rect> tracked;
  for (std::vector<cmt::CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
  {
    cmt_tracker_msgs::Tracker tracker;
    if ((*v).initialized == true && !im_gray.empty())
    {
      tracker.inital_points.data = (*v).num_initial_keypoints;
      tracker.active_points.data = (*v).num_active_keypoints;
      tracker.tracker_name.data = (*v).name;
      if (!(*v).tracker_lost)
      {
        (*v).processFrame(im_gray, factor);
        cv::Rect rect = (*v).bb_rot.boundingRect();
        tracked.push_back(rect);
        quality_of_tracker[quality_update] = (*v).num_active_keypoints;
        quality_update++;
        //FILE_LOG(logDEBUG) << "Area ouptut is: " << rect.area();
        rect = rect & cv::Rect(0, 0, im_gray.size().width, im_gray.size().height);
        tracker.pixel_lu.x = rect.x;
        tracker.pixel_lu.y = rect.y;
        tracker.pixel_lu.z = 0;
        tracker.width.data = rect.width;
        tracker.height.data = rect.height;
        tracker.quality_results.data = true;
        trackers_results.tracker_results.push_back(tracker);
      }
      else
      {
        tracker.pixel_lu.x = 0;
        tracker.pixel_lu.y = 0;
        tracker.pixel_lu.z = 0;
        tracker.width.data = 0;
        tracker.height.data = 0;
        tracker.quality_results.data = false;
        trackers_results.tracker_results.push_back(tracker);
        poorly_tracked.push_back(count);
      }
    }
    count++;
  }

  pi_face_tracker::Faces pi_results= returnPiMessages(trackers_results, camera_config); 

  tracker_results_pub.publish(trackers_results);
  pi_vision_results.publish(pi_results);
  //Now Let's do different methods based on what parameters are set:

  //if delete on lost is selected.
  std::string value;
  nh_.getParam("tracking_method", value);

  if (value.compare("mustbeface") == 0)
  {
    deleteOnLost(tracked);
  }
  else if (value.compare("FaceRecognition") == 0)
  {
    // Incoporate face tracking fucntinoality here.
  }

  trackers_results.tracker_results.clear();

}
//Now this function deltes all the poor tracked values and adds trackers if there are overlaps in the system.
void TrackerCMT::deleteOnLost(std::vector<cv::Rect> tracked)
{
  std::cout<<"enters delete on lost"<<std::endl; 
  int size  = poorly_tracked.size();
  if (size > 0)
  {
    //nh_.setParam("tracker_updated", true);
    for (int i = 0; i < poorly_tracked.size(); i++)
    {
      remove_tracker(poorly_tracked[i]);
    }
  }

  cmt_tracker_msgs::Trackers track_locs = returnOverlapping(tracked, face_locs);

  if ( track_locs.tracker_results.size() > 0 )
  {
    //now wait for update irregardless of of deletion; 
    
    tracker_locations_pub.publish(track_locs);
    update_ui_wait = true; //this would update the ui after this locations have been published in the system. 
  }
  else 
  {
    if(size > 0)
      nh_.setParam("tracker_updated", 2); 
  }
  std::cout<<"exits delete on lost"<<std::endl; 
}

void TrackerCMT::callback(cmt_tracker_msgs::TrackerConfig &config, uint32_t level)
{
  factor = config.factor;
}
void TrackerCMT::list_of_faces_update(const cmt_tracker_msgs::Faces& faces_info)
{
  face_locs.faces.clear();
  for (int i = 0; i < faces_info.faces.size(); i++)
  {
    face_locs.faces.push_back(faces_info.faces[i]);
  }
  //nh_.setParam("tracker_updated",true);
}
void TrackerCMT::set_tracker(const cmt_tracker_msgs::Tracker& tracker_location)
{
  //A potentially high penality task is done here but it's to avoid latter dealing with uncorrectly set trackers.
  cv::Mat im_gray = frame_gray.clone(); //To avoid change when being run.
  cv::Rect rect(tracker_location.pixel_lu.x, tracker_location.pixel_lu.y, tracker_location.width.data, tracker_location.height.data );
  if (!im_gray.empty() && rect.area() > 50)
  {
    //Now there must be some way to hold back setting up new tracker;
    cmt.push_back(cmt::CMT());
    cmt.back().consensus.estimate_rotation = true;
    srand(time(NULL));
    int tracker_num;
    //check tracker id is unique;
    tracker_num = rand() % 100000;
    std::string tracker_name = SSTR(tracker_num) ;
    cmt.back().initialize(im_gray, rect, tracker_name);
    quality_of_tracker.push_back(cmt.back().num_initial_keypoints);

    //Here is where one needs to publish new face event
    pi_face_tracker::FaceEvent m = returnPiEvents("new_face", tracker_name); 

    pi_events.publish(m); 

  }
  else
  {
    //FILE_LOG(logDEBUG) << "Not initialized";
  }
  std::string value;
  nh_.getParam("tracking_method", value);
  if(value.compare("handtracking") == 0) nh_.setParam("tracker_updated",2);
}
void TrackerCMT::set_trackers(const cmt_tracker_msgs::Trackers& tracker_location)
{
  for (int i = 0; i < tracker_location.tracker_results.size(); i++)
  {
    set_tracker(tracker_location.tracker_results[i]);
  }
  if(update_ui_wait)
    {
      nh_.setParam("tracker_updated", 2); 
      update_ui_wait= false; 
    }


}
void TrackerCMT::remove_tracker(int index)
{

  pi_face_tracker::FaceEvent m = returnPiEvents("lost_face", cmt[index].name); 
  pi_events.publish(m); 
  cmt.erase(cmt.begin() + index);
  

}

namespace {
std::vector<cv::Rect> facedetect(cv::Mat frame_gray)
{
  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;
  std::vector<cv::Rect> faces;
  bool setup;
  if ( !face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml" ) || !eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml" ))
  { setup = false;  };
  setup = true;
  if (setup)
  {

    face_cascade.detectMultiScale( frame_gray, faces, 1.05, 4, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(40, 40) );
    //Now add eyes if one desires

  }
  return faces;
}

cmt_tracker_msgs::Trackers convert(std::vector<cv::Rect> faces)
{
  
  cmt_tracker_msgs::Trackers tracker_description;
  for (size_t i = 0; i < faces.size(); i++)
  {

    cmt_tracker_msgs::Tracker face_description;
    face_description.pixel_lu.x = faces[i].x;
    face_description.pixel_lu.y = faces[i].y;
    //Now place coordinates to the value Z from the
    //depth camera.
    face_description.pixel_lu.z = 0;
    face_description.height.data = faces[i].height;
    face_description.width.data = faces[i].width;


    tracker_description.tracker_results.push_back(face_description);
  }
  
  return tracker_description;
}
cmt_tracker_msgs::Trackers returnOverlapping(std::vector<cv::Rect> cmt_locations, cmt_tracker_msgs::Faces facelocs)
{
  
  std::vector<cv::Rect> not_overlapped;
  //This is wrote to avoid calling face detect algorithm multiple times in this application. So, we check for overlap between the cmt_locations
  //and the facelocs (found from the face_locator node) and then if there are new values we push it note the rect and hold it in a new rect.
  for (int i = 0; i < facelocs.faces.size(); i++)
  {
    cv::Rect area_overlap(facelocs.faces[i].pixel_lu.x, facelocs.faces[i].pixel_lu.y, facelocs.faces[i].width.data, facelocs.faces[i].height.data);
    bool no_overlap = false;
    for (int j = 0; j < cmt_locations.size(); j++)
    {
      //if it overlaps with the one then break.
      no_overlap = (area_overlap & cmt_locations[j]).area() > 0;
      if (no_overlap)
        break;
    }
    if (!no_overlap)
      not_overlapped.push_back(area_overlap);
    //if doesn't overlap then break.
  }
  
  return convert(not_overlapped);
}

pi_face_tracker::Face returnPiMessage(cmt_tracker_msgs::Tracker locs, camera_properties camera_config)
{
  pi_face_tracker::Face msg; 

  
  msg.id = std::atoi(locs.tracker_name.data.c_str()); 
  //Now let's convert the point to a 3d representation from a  pi definition
  /*
        p = Point()
        # same FOV for both, so calculate the relative distance of one pixel
        dp = 0.22 / float(self.bounding_size) # It should be same in both axis
        # logger.warn("bbox size=" + str(self.bounding_size))
        w = self.camera_width/2
        h = self.camera_height/2
        # Y is to the left in camera image, Z is to top
        p.x = dp *  (h / tan(self.camera_fov_x/2.0))
        p.y = dp * (w-(self.pt2[0]+self.pt1[0])/2)
        p.z = dp * (h-(self.pt2[1]+self.pt1[1])/2)

            self.pt1 = (x,y)
            self.pt2 = (x+w, y+h)
  */


  //Here definition relating to  bounding_size

  //self.bounding_size = self.pt2[1] - self.pt1[1]
  //While in initilzation it's considered as x location
  // self.bounding_size = pt2[0] - pt1[0]

  double dp; 
  double width; 
  double height; 

  dp = 0.22 / locs.height.data; //This is how it's defined as above
  width = camera_config.width / 2; 
  height = camera_config.height / 2; 



  msg.point.x = dp * (height / tan(camera_config.fov/2)); 
  msg.point.y = dp * (width - ((locs.pixel_lu.x + locs.width.data) + locs.pixel_lu.x) / 2 ); 
  msg.point.z = dp * (width - ((locs.pixel_lu.y + locs.height.data) + locs.pixel_lu.y) / 2 ); 

  return msg; 
}

pi_face_tracker::Faces returnPiMessages(cmt_tracker_msgs::Trackers locs, camera_properties camera_config)
{
  pi_face_tracker::Faces msgs; 
    for (int i = 0; i < locs.tracker_results.size(); i++)
    {
      msgs.faces.push_back(returnPiMessage(locs.tracker_results[i], camera_config)); 
    }
  return msgs; 
}

pi_face_tracker::FaceEvent returnPiEvents(std::string evt, std::string face_id)
{
  pi_face_tracker::FaceEvent event; 
  event.face_id = std::atoi(face_id.c_str()); 
  event.face_event = evt; 
  return event; 
}

}

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmt_tracker");
  cmt_wrap::TrackerCMT ic;
  ros::spin();
  return 0;
}