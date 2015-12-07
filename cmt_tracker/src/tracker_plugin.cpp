#include "tracker_plugin.h"
#include <QDebug>
#include <QList>
#include <QImage>
#include <QPixmap>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <ros/console.h>


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
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    // widget_->setWindowTitle("Tray's ")
  }
  context.addWidget(widget_);

  /**

  Now here the methods and rules to run the scirpts are entirely run.

  So it's assuming that there are rules that are placed in the notion of the scirpts to maintain the desired elements in the overall system.

  also if the person has given in the parameters that are run with the script that is running then we would be able to run the system.
  */

  last_selected_item = -1;
  frame = 0;
  ui.face_choice_method->addItem("Hand Slection Trackings");
  tracking_method = 0;
  ui.face_choice_method->addItem("Eyes and Shadowning");
  first_run_eyes = -1;
  ui.face_choice_method->addItem("Skeleton Based Tracking");

//Get a nodehandle to subscribe to nodes;

//Declare Subscribers here;
// nh = getNodeHandle();
  if ( !face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml" ) || !eyes_cascade.load("/usr/share/opencv/haarcascades/haarcascade_eye_tree_eyeglasses.xml" ))
  { setup = false;  };
  setup = true;
  nh.getParam("camera_topic", subscribe_topic);
  face_subscriber = (nh).subscribe("face_locations", 1, &rqt_tracker_view::tracker_plugin::list_of_faces_update, this);
  image_transport::ImageTransport it(nh);
  image_subscriber = it.subscribe(subscribe_topic, 1, &rqt_tracker_view::tracker_plugin::imageCb, this);
  image_publisher = it.advertise("/transformed/images", 1);
  tracker_locations_pub = (nh).advertise<cmt_tracker::Faces>("/tracker/tracking_locations", 10);
  nh.setParam("tracking_method", "handtracking");
  connect(ui.face_choice_method, SIGNAL(currentIndexChanged(int)), this, SLOT(on_MethodChanged(int)));
  connect(ui.face_output_list, SIGNAL(itemPressed(QListWidgetItem *)), this, SLOT(on_addToTrack_clicked(QListWidgetItem *)));
  connect(ui.removeAllTracked, SIGNAL(pressed()), this, SLOT(on_removeAllTracked_clicked()));
  connect(ui.removeTracked, SIGNAL(pressed()), this, SLOT(on_removeTracked_clicked()));
  connect(this, SIGNAL(updatefacelist()), this, SLOT(updateVisibleFaces()));



//Set the intial location fo faces to be zero.
// face_locs.faces

//Now connect the application.


//end of initPlugin
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
//now here do all the processing. 
  

}

/**
This function is the one that update hte UI of all things related to the system.
*/
void tracker_plugin::updateVisibleFaces()
{

//Update the Faces List.
  ui.face_output_list->clear();
  ui.tracker_output_list->clear();
  // ui.tracker_output_list->clear();
  int counter_name = 0;
  for (std::vector<QImage>::iterator it = qmap.begin(); it != qmap.end(); ++it)
  {
    //Update the Face ID with a complete description of the face()
    ui.face_output_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(*it)), QString::number(face_locs.faces[counter_name].id.data)));
    counter_name++;
  }

//Add to the Tracking in the system.
  if (hold_var != -1 || auto_emit > 0) //something is selected
  {
    //Move this to the next time when the GUI is updated.
    if (auto_emit < 0)
    {
      std::cout << "Updating previously selected item: " << hold_var << std::endl;
      //Replace with ability to store all values of a previous.
      QImage qimage = QImage(face_images[hold_var].data, face_images[hold_var].cols, face_images[hold_var].rows,
                             face_images[hold_var].step[0], QImage::Format_RGB888);
      ui.tracker_initial_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(qimage)), "Being Tracked"));
    }
    else
    {
      //Now this is a new set of images
      std::cout << "Updating previously selected item: " << hold_var << std::endl;
      //Replace with ability to store all values of a previous.
      QImage qimage = QImage(face_images[hold_var].data, face_images[hold_var].cols, face_images[hold_var].rows,
                             face_images[hold_var].step[0], QImage::Format_RGB888);
      ui.tracker_initial_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(qimage)), "Being Tracked"));
    }
  }
  hold_var = -1;
  auto_emit = auto_emit - 1;

//Visualize the Results of the system.
  int counter_id = 0;


  for (std::vector<QImage>::iterator it = tracked_image_results.begin(); it != tracked_image_results.end(); ++it)
  {
    ui.tracker_output_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(*it)),  QString::number(counter_id)));
    counter_id++;
  }

  ros::Rate rate(1);

}




/**
The one is the one that update the value of the funciton .
*/
void tracker_plugin::list_of_faces_update(const cmt_tracker::Faces& faces_info)
{
  ROS_DEBUG("It get's here in the faces update");
//This doesn't directly update the list of faces but sets the value of the faces in the cmt_tracker::Faces when
  //ever the next image comes whihc is handled by hte imageCb fucntion above.
  tracker_plugin::face_locs.faces.clear();
  //May be better to use an iterator to handle the function.
  for (int i = 0; i < faces_info.faces.size(); i++)
  {
    face_locs.faces.push_back(faces_info.faces[i]);
  }
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
  tracking_method = index;
  // std::cout << "The Index is:" << index <<std::endl;
  if (index == 0)
  {
    nh.setParam("tracking_method", "handtracking");
    first_run_eyes == -1;
  }
  else if (index == 1) {
    nh.setParam("tracking_method", "eyes");
    first_run_eyes = 0;
  }
  else
  {
    nh.setParam("tracking_method", "skeleton");
    first_run_eyes = -1;
  }




}
/**
 * @brief tracker_plugin::on_addToTrack_clicked
 * Is a function that set's the parameters for the CMT tracker and also displays it in the viewframe trackedView
 */
void tracker_plugin::on_addToTrack_clicked(QListWidgetItem *item)
{

  last_selected_item = ui.face_output_list->currentRow();

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
  ui.tracker_initial_list->clear();
  ui.tracker_output_list->clear();
  cmt.clear();
}
/**
 * @brief tracker_plugin::on_removeTracked_clicked
 *
 * Removes selected tracking elements in the trackerView ELements.
 */
void tracker_plugin::on_removeTracked_clicked()
{
//    qDebug() <<"Elements to Be Deleted: ";
  //Here elements to be removed on tracking.
//Remove Certain Tracking parameters.
  qDeleteAll(ui.tracker_output_list->selectedItems());
}
/**
 * @brief tracker_plugin::on_removeAllElements_clicked
 *
 * This is the fucntion that is placed as inital rough draft of the funciton in the user interface.
 */
void tracker_plugin::on_removeAllElements_clicked()
{
  //Clears List
  ui.face_output_list->clear();
}

}
PLUGINLIB_EXPORT_CLASS(rqt_tracker_view::tracker_plugin, rqt_gui_cpp::Plugin)