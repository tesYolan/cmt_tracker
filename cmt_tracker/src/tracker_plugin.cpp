#include "tracker_plugin.h"
#include <QDebug>
#include <QList>
#include <QImage>
#include <QPixmap>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <cmt_tracker/Clear.h>
#include <cmt_tracker/Face.h>
#include <cmt_tracker/Faces.h>
#include <iostream>
#include <sstream>


#define SSTR( x ) dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x ) ).str()

namespace rqt_tracker_view {

tracker_plugin::tracker_plugin()
  : rqt_gui_cpp::Plugin(),
    widget_(0)
{

  setObjectName("TrackerView");
  //ui.setupUi(this);
}

void tracker_plugin::initPlugin(qt_gui_cpp::PluginContext& context)
{

  widget_ = new QWidget();
  ui.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    // widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    // widget_->setWindowTitle("Tray's ")
  }
  context.addWidget(widget_);

  /**

  Now here the methods and rules to run the scirpts are entirely run.

  So it's assuming that there are rules that are placed in the notion of the scirpts to maintain the desired elements in the overall system.

  also if the person has given in the parameters that are run with the script that is running then we would be able to run the system.
  */


  // frame = 0;
  tracker_updated = true;
  tracking_results_updated = false;
  ui.face_choice_method->addItem("Hand Selection Trackings");
  ui.face_choice_method->addItem("Remove on LOST");
  ui.face_choice_method->addItem("Face Recognition Method");
  image_transport::ImageTransport it(nh);

  nh.getParam("camera_topic", subscribe_topic);

  face_subscriber = (nh).subscribe("face_locations", 1, &rqt_tracker_view::tracker_plugin::list_of_faces_update, this);
  image_subscriber = it.subscribe(subscribe_topic, 1, &rqt_tracker_view::tracker_plugin::imageCb, this);
  tracked_locations = nh.subscribe("tracker_results", 10 , &rqt_tracker_view::tracker_plugin::tracker_resultsCb, this);
  
  // // image_publisher = it.advertise("/transformed/images", 1);

  //This is a publisher to check initally by setting trackers in the rqt plugin.
  tracker_locations_pub = (nh).advertise<cmt_tracker::Tracker>("tracking_location", 10);

  client = nh.serviceClient<cmt_tracker::Clear>("clear");
  image_client= nh.serviceClient<cmt_tracker::TrackedImages>("get_cmt_rects");


  //This is subscribed here because of other nodes outside this rqt plugin  set tracker location and thus this extension
  //must show the ability to show different elements in the process.
  tracker_locations_sub = (nh).subscribe("tracking_location", 10 , &rqt_tracker_view::tracker_plugin::trackerCb, this);
  nh.setParam("tracking_method", "handtracking");

  connect(ui.face_choice_method, SIGNAL(currentIndexChanged(int)), this, SLOT(on_MethodChanged(int)));
  connect(ui.face_output_list, SIGNAL(itemPressed(QListWidgetItem *)), this, SLOT(on_addToTrack_clicked(QListWidgetItem *)));
  connect(ui.removeAllTracked, SIGNAL(pressed()), this, SLOT(on_removeAllTracked_clicked()));
  connect(ui.removeTracked, SIGNAL(pressed()), this, SLOT(on_removeTracked_clicked()));
  connect(this, SIGNAL(updatefacelist()), this, SLOT(updateVisibleFaces()));

  f = boost::bind(&rqt_tracker_view::tracker_plugin::callback, this , _1, _2);
  server.setCallback(f);
  

}

void tracker_plugin::callback(cmt_tracker::TrackerConfig &config, uint32_t level)
{
  //Code to handle the threshold values of the function. 
}
void tracker_plugin::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
        qWarning("ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)", msg->encoding.c_str(), e.what());
        // ui_.image_frame->setImage(QImage());
        return;
      }
    }
    catch (cv_bridge::Exception& e)
    {
      qWarning("ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)", msg->encoding.c_str(), e.what());
      // ui_.image_frame->setImage(QImage());
      return;
    }
  }
  //now update the results of the elements in the GUI thread. It's done here because the GUI thread and the call back thread
  //are two different entities and since ros::spin() handles (unless specified otherwise) callbacks serially it's best to
  //get the data's here sequentially.
  mat_images.clear();
  face_images.clear();
  for (std::vector<cmt_tracker::Face>::iterator v = face_locs.faces.begin(); v != face_locs.faces.end() ; ++v)
  {
    mat_images.push_back(conversion_mat_(cv::Rect((*v).pixel_lu.x, (*v).pixel_lu.y, (*v).width.data, (*v).height.data)).clone());
    face_images.push_back(QImage((uchar*) mat_images.back().data, mat_images.back().cols, mat_images.back().rows,
                                 mat_images.back().step[0], QImage::Format_RGB888));
  }

  //The logic for subscribed works like this; (if there has been change in the value then we add it to the list other wise we let it be)

  if (tracker_updated)
  {
  //Now let get the service call to the system.

      //Now let's iterate over the results and rename the systems.

    tracked_images.push_back(conversion_mat_(cv::Rect(track_published.pixel_lu.x, track_published.pixel_lu.y, track_published.width.data,
                             track_published.height.data)).clone());
    tracked_faces.push_back(QImage((uchar*) tracked_images.back().data, tracked_images.back().cols, tracked_images.back().rows,
                                   tracked_images.back().step[0], QImage::Format_RGB888));
  }
  tracked_image_mats.clear();
  tracked_image_results.clear();
  tracked_image_information.clear();
  for (std::vector<cmt_tracker::Tracker>::iterator v = tracking_results.tracker_results.begin(); v != tracking_results.tracker_results.end() ; ++v)
  {
    std::string quality;
    if((*v).quality_results.data)
    {
    quality =  "true";
    }
    else
    {
    quality = "false";
    }
    std::string value = (*v).tracker_name.data + "\n" + SSTR((*v).inital_points.data) + "\n" + SSTR((*v).active_points.data)  + "\n" +  quality;
    tracked_image_information.push_back( value );

    //Now here if the tracker results is positive then output this as a result of the image other wise update the results.
    if ((*v).quality_results.data)
    {
      tracked_image_mats.push_back(conversion_mat_(cv::Rect((*v).pixel_lu.x, (*v).pixel_lu.y, (*v).width.data, (*v).height.data)).clone());
      tracked_image_results.push_back(QImage((uchar*) tracked_image_mats.back().data, tracked_image_mats.back().cols, tracked_image_mats.back().rows,
                                             tracked_image_mats.back().step[0], QImage::Format_RGB888));
    }
    else {
      cv::Mat img(100, 100, CV_8UC3);
      img.setTo(cv::Scalar(5));
      tracked_image_mats.push_back(img.clone());
      tracked_image_results.push_back(QImage((uchar*) tracked_image_mats.back().data, tracked_image_mats.back().cols, tracked_image_mats.back().rows,
                                             tracked_image_mats.back().step[0], QImage::Format_RGB888));

    }
  }

  emit updatefacelist();
}

/**
This function is the one that update hte UI of all things related to the system.
*/
void tracker_plugin::updateVisibleFaces()
{
  ui.face_output_list->clear();
  ui.tracker_output_list->clear();


  for (std::vector<QImage>::iterator v = face_images.begin(); v != face_images.end(); ++v)
  {
    ui.face_output_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(*v)), "Faces"));
  }

  //Update the last element to the list
  //Sends a service to get all images from the cmt_node.
  //nh.getParam("tracker_updated", tracker_updated);
  if (tracker_updated)
  {
    cmt_tracker::TrackedImages results;

    if(image_client.call(results))
     //Query the images from the CMT tracker.
    {
    ui.tracker_initial_list->clear();

    //CVMAT list of the faces.
    tracked_images.clear();

    //QIMage list of tracked results
    tracked_faces.clear();
    //Now how do we get the items from the list of indexes.

    for (int i = 0; i < results.response.image.size(); i++)
    {
        sensor_msgs::Image im = results.response.image[i];

        sensor_msgs::ImagePtr  r= boost::shared_ptr<sensor_msgs::Image>(boost::make_shared<sensor_msgs::Image>(im));
        //r = boost::shared_ptr<sensor_msgs::Image>(im);
        cv_bridge::CvImageConstPtr cv_ptr;
        cv::Mat image;
        try{
        cv_ptr = cv_bridge::toCvShare(r);
        image= cv_ptr->image;

        //cv::imshow("H", image);
        }
        catch(cv_bridge::Exception& e)
        {
        std::cout<<"Error"<<std::endl;
        return;
        }
        tracked_images.push_back(image);
        tracked_faces.push_back(QImage((uchar*) tracked_images.back().data, tracked_images.back().cols, tracked_images.back().rows,tracked_images.back().step[0], QImage::Format_Indexed8));

        ui.tracker_initial_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(tracked_faces.back())), QString::fromUtf8(results.response.names[i].c_str())));
    }

    //tracked_faces.push_back(QImage((uchar*) tracked_images.back().data, tracked_images.back().cols, tracked_images.back().rows,tracked_images.back().step[0], QImage::Format_RGB888));

    //Now let's add all the values by iterating over a list of items in space

//    for (int i = 0; i < results.response.names.size(); i++)
//    {
//      ui.tracker_initial_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(tracked_faces[i])), QString::fromUtf8(results.response.names[i].c_str())));
//    }

    }
    else
    {
      //DISPLAY ERROR in the SYSTEM.
    }

  }
  tracker_updated = false;

  int count_info = 0 ;
  for (std::vector<QImage>::iterator v = tracked_image_results.begin(); v != tracked_image_results.end(); ++v)
  {
    ui.tracker_output_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(*v)), QString::fromStdString(tracked_image_information[count_info])));
    count_info++;
  }


}
/**
The one is the one that update the value of the funciton .
*/
void tracker_plugin::list_of_faces_update(const cmt_tracker::Faces& faces_info)
{
  face_locs.faces.clear();
  //May be better to use an iterator to handle the function.
  for (int i = 0; i < faces_info.faces.size(); i++)
  {
    face_locs.faces.push_back(faces_info.faces[i]);
  }
}

void tracker_plugin::trackerCb(const cmt_tracker::Tracker& tracker_locs)
{
  //track_published = tracker_locs;

  //This is the condition we update the system.
  track_published.pixel_lu.x = tracker_locs.pixel_lu.x;
  track_published.pixel_lu.y = tracker_locs.pixel_lu.y;
  track_published.width.data = tracker_locs.width.data;
  track_published.height.data = tracker_locs.height.data;
  track_published.tracker_name.data = tracker_locs.tracker_name.data;

  tracker_updated = true;

}
void tracker_plugin::tracker_resultsCb(const cmt_tracker::Trackers& tracker_results)
{
  //Check whether this is invalided when the loop exits.
  // = tracker_results;
  tracking_results.tracker_results.clear();
  for (int i = 0; i < tracker_results.tracker_results.size(); i++)
  {
    tracking_results.tracker_results.push_back(tracker_results.tracker_results[i]);
  }
  tracking_results_updated = true;
}

void tracker_plugin::shutdownPlugin()
{
  //Do shutdown objects here.
  face_subscriber.shutdown();
  image_subscriber.shutdown();
}

void tracker_plugin::on_MethodChanged(int index)
{
  // QString topic = ui.face_choice_method->itemData(ui.face_choice_method->currentIndex());
  // tracking_method = index;
  // std::cout << "The Index is:" << index <<std::endl;
  if (index == 0)
  {
    nh.setParam("tracking_method", "handtracking");
  }
  else if (index == 1) {
    nh.setParam("tracking_method", "DisappearingFace");
  }
  else
  {
    nh.setParam("tracking_method", "FaceRecognition");
  }

}
/**
 * @brief tracker_plugin::on_addToTrack_clicked
 * Is a function that set's the parameters for the CMT tracker and also displays it in the viewframe trackedView
 */
void tracker_plugin::on_addToTrack_clicked(QListWidgetItem *item)
{

  int last_selected_item = ui.face_output_list->currentRow();


  //Now here one publishes the last selected item in the list.

  track_location.pixel_lu.x = face_locs.faces[last_selected_item].pixel_lu.x;
  track_location.pixel_lu.y = face_locs.faces[last_selected_item].pixel_lu.y;
  track_location.width.data = face_locs.faces[last_selected_item].width.data;
  track_location.height.data = face_locs.faces[last_selected_item].height.data;
  track_location.tracker_name.data= face_locs.faces[last_selected_item].name.data;
  //Let's create here a name by which it's random. 


  std::string name;
  //name = "tracker: " + SSTR(tracker_num);
  track_location.tracker_name.data = name; 

  tracker_locations_pub.publish(track_location);
}
/**
 * @brief tracker_plugin::on_removeAllTracked_clicked
 *
 * is a fucntion that removes all the CMT tracking objects and clears the tracking objects in the scene.
 *
 */
void tracker_plugin::on_removeAllTracked_clicked()
{
  //Here All Items need only be removed
  // ui.tracker_initial_list->clear();
  // ui.tracker_output_list->clear();
  // cmt.clear();
  cmt_tracker::Clear srv; 
  if(client.call(srv))
  {
    std::cout<<"Cleared"<<std::endl; 
  }
  else{
    std::cout<<"Not Cleared"<<std::endl; 
  }
  ui.tracker_initial_list->clear(); 
}
/**
 * @brief tracker_plugin::on_removeTracked_clicked
 *
 * Removes selected tracking elements in the trackerView ELements.
 */
void tracker_plugin::on_removeTracked_clicked()
{

}
/**
 * @brief tracker_plugin::on_removeAllElements_clicked
 *
 * This is the fucntion that is placed as inital rough draft of the funciton in the user interface.
 */
void tracker_plugin::on_removeAllElements_clicked()
{

}

}
PLUGINLIB_EXPORT_CLASS(rqt_tracker_view::tracker_plugin, rqt_gui_cpp::Plugin)