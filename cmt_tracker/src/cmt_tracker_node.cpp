

using namespace cmt;


class TrackerCMT
{

//to hold the value of the image
  cv::Mat conversion_mat_;
  cv::Mat frame_gray;

  dynamic_reconfigure::Server<cmt_tracker::TrackerConfig> server;
  dynamic_reconfigure::Server<cmt_tracker::TrackerConfig>::CallbackType f;

  std::vector<cv::Mat> tracked_images;

  cv::CascadeClassifier face_cascade;
  cv::CascadeClassifier eyes_cascade;
  std::vector<cv::Rect> faces;
  std::vector<cv::Rect> eyes;

  ros::NodeHandle nh_;

  cmt_tracker::Tracker track_location;

  ros::ServiceServer clear_service;
  ros::ServiceServer image_service;
  ros::ServiceServer update_service;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_face_pub;
  ros::Publisher tracker_locations_pub;
  ros::Publisher tracker_results_pub;

  ros::Subscriber face_results;

  ros::Subscriber tracker_subscriber;
  ros::Subscriber trackers_subscriber;

  ros::Subscriber face_subscriber;

  cmt_tracker::Tracker tracker_set;
  cmt_tracker::Faces face_locs;
  cmt_tracker::Trackers trackers_results;

  std::vector<CMT> cmt;
  std::vector<int> quality_of_tracker;

  std::vector<int> poorly_tracked;

  std::string tracking_method;
  bool setup;
  int last_count;

  double factor;
  

  std::vector<cv::Rect> locations_of_trackers; //for setting the tracking to higher levels.
  std::string subscribe_topic;
  sensor_msgs::ImagePtr masked_image;

  int frame_counters;
  int frame_previous;

  bool update_ui;
  //Parameters for the CMT part of the code.
public:
  TrackerCMT() : it_(nh_)
  {
    
    if ( !face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml" ) || !eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml" ))
    { setup = false;  };
    setup = true;
    nh_.getParam("camera_topic", subscribe_topic);
    last_count=0; 
    frame_counters = 0;
    frame_previous = 0;
    update_ui= false;
    //To acquire the image that we will be doing processing on
    image_sub_ = it_.subscribe(subscribe_topic, 1, &TrackerCMT::imageCb, this);


    //Service to deal with the image systems.
    clear_service = nh_.advertiseService("clear", &TrackerCMT::clear, this);
    image_service = nh_.advertiseService("get_cmt_rects", &TrackerCMT::getTrackedImages, this);
    update_service = nh_.advertiseService("update", &TrackerCMT::updated, this);

    //To acquire the list of faces currently in track.
    face_subscriber = (nh_).subscribe("face_locations", 1, &TrackerCMT::list_of_faces_update, this);
    tracker_locations_pub = (nh_).advertise<cmt_tracker::Trackers>("tracking_locations", 10);


    //To acquire commands from this and other nodes to what to set to track.
    // masked_image = cv_bridge::CvImage(std_msgs::Header(), "mono", image_roi).toImageMsg();
    tracker_subscriber = (nh_).subscribe("tracking_location", 1, &TrackerCMT::set_tracker, this);
    trackers_subscriber = (nh_).subscribe("tracking_locations", 1, &TrackerCMT::set_trackers, this);
    image_pub_ = it_.advertise("/transformed/images", 1);
    image_face_pub = it_.advertise("/transformed/faces", 1);
    
    tracker_results_pub = nh_.advertise<cmt_tracker::Trackers>("tracker_results", 10);

    f = boost::bind(&TrackerCMT::callback, this, _1, _2);

    server.setCallback(f);
  }

  bool clear(cmt_tracker::Clear::Request &req, cmt_tracker::Clear::Response &res)
  {
    cmt.clear();
    quality_of_tracker.clear();
    if (cmt.size() == 0)
    {
       res.cleared= true;
    }
    else
    {
       res.cleared = false;
    }
    return true;
  }
  bool updated(cmt_tracker::Update::Request &req, cmt_tracker::Update::Response &res)
  {
    res.update.data = false;
    if (update_ui)
    {
    res.update.data = true;
    update_ui= false;

    }
    return true;
  }
  /*
  Request the images in the system.
  */
  bool getTrackedImages(cmt_tracker::TrackedImages::Request &req,
                        cmt_tracker::TrackedImages::Response &res)
  {
  //Now this get's the name of elements in one iteration in a single copy method.
      for (std::vector<CMT>::iterator v = cmt.begin(); v!= cmt.end(); ++v)
      {

        res.names.push_back((*v).name);

        sensor_msgs::ImagePtr masked_image= cv_bridge::CvImage(std_msgs::Header(), "mono8", (*v).imArchive).toImageMsg();


        //Why not publish the image here and see it's corresponding response regarding how to implement the best form of the system.
        image_face_pub.publish(masked_image);

        res.image.push_back(*masked_image);

      }
      return true;
  }
  /*
  This  function is a callback function that happens when an image update occurs.
  */
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
  update_ui = false;
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
    cv::Mat im_masked= frame_gray.clone();
    std::cout << "The number of tracker active is: " << cmt.size() << std::endl;
    int quality_update = 0;
    int count=0;
    poorly_tracked.clear();
    std::vector<cv::Rect> tracked; 
    for (std::vector<CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
    {
      cmt_tracker::Tracker tracker;
      if ((*v).initialized == true && !im_gray.empty())
      {
        tracker.inital_points.data = (*v).num_initial_keypoints;
        tracker.active_points.data = (*v).num_active_keypoints;
        tracker.tracker_name.data = (*v).name;
        if(!(*v).tracker_lost)
        {
        (*v).processFrame(im_gray,factor);
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


    tracker_results_pub.publish(trackers_results);

    //Now Let's do different methods based on what parameters are set:

    //if delete on lost is selected.
    std::string value;
    nh_.getParam("tracking_method",value);

    if(value.compare("DisappearingFace") == 0)
    {
      deleteOnLost(tracked);
    }
    else if(value.compare("FaceRecognition") == 0)
    {
      // Incoporate face tracking fucntinoality here.
    }

  trackers_results.tracker_results.clear();

  }
  void deleteOnLost(std::vector<cv::Rect> tracked)
  {
    if(poorly_tracked.size() > 0) update_ui=true;

    for (int i=0; i< poorly_tracked.size(); i++)
    {
      remove_tracker(poorly_tracked[i]);
    }
    //Now what we do now here check if there are no overlapping rects between the cmt tracker results and the face detection results
    //if there are add them to tracker. No one problem is what are teh tracked 
    cmt_tracker::Trackers track_locs= Face_Detection::returnOverlapping(tracked,face_locs);

    if(track_locs.tracker_results.size() > 0) update_ui=true;

    tracker_locations_pub.publish(track_locs);

  }

  void callback(cmt_tracker::TrackerConfig &config, uint32_t level)
  {
    factor = config.factor;
  }
  void list_of_faces_update(const cmt_tracker::Faces& faces_info)
  {
    face_locs.faces.clear();
    for (int i = 0; i < faces_info.faces.size(); i++)
    {
      face_locs.faces.push_back(faces_info.faces[i]);
    }
    //nh_.setParam("tracker_updated",true);
  }
  void set_tracker(const cmt_tracker::Tracker& tracker_location)
  {
    //A potentially high penality task is done here but it's to avoid latter dealing with uncorrectly set trackers.
    cv::Mat im_gray = frame_gray.clone(); //To avoid change when being run.
    cv::Rect rect(tracker_location.pixel_lu.x, tracker_location.pixel_lu.y, tracker_location.width.data, tracker_location.height.data );
    if (!im_gray.empty() && rect.area() > 50)
    {
      //Now there must be some way to hold back setting up new tracker;
      cmt.push_back(CMT());
      cmt.back().consensus.estimate_rotation = true;
      srand(time(NULL));
      int tracker_num;
      //check tracker id is unique;
      tracker_num = rand() % 100000;
      std::string tracker_name = SSTR(tracker_num) ;
      cmt.back().initialize(im_gray, rect, tracker_name);
      quality_of_tracker.push_back(cmt.back().num_initial_keypoints);
    }
    else
    {
      //FILE_LOG(logDEBUG) << "Not initialized";
    }
    //nh_.setParam("tracker_updated","true");
  }
  void set_trackers(const cmt_tracker::Trackers& tracker_location)
  {
    for (int i=0; i< tracker_location.tracker_results.size(); i++)
    {
      set_tracker(tracker_location.tracker_results[i]);
    }

  }
  void remove_tracker(int index)
  {
    cmt.erase(cmt.begin()+index);
  }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmt_tracker");
  TrackerCMT ic;
  ros::spin();
  return 0;
}