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
  if (frame % 5 == 0 || last_selected_item != -1)
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
//Now we have a file that is formated;

    //Now let's do the taking out of the essential locations of the faces.
    //Now this is where all the things happen. So this is the part that needs to be subed out when there
    //are conditions from the parameters.
    qmap.clear();
    face_images.clear();
    for (int i = 0; i < tracker_plugin::face_locs.faces.size(); i++)
    {

      //Here is where the list of faces updated in the system by using QImage.

      cv::Point ul ;
      ul.x = tracker_plugin::face_locs.faces[i].pixel_lu.x;
      ul.y =  tracker_plugin::face_locs.faces[i].pixel_lu.y;

      cv::Point ll ;
      ll.x = ul.x + tracker_plugin::face_locs.faces[i].width.data;
      ll.x = ul.y + tracker_plugin::face_locs.faces[i].height.data;

      //Now let's create an image area map of the ROI

      face_images.push_back(conversion_mat_(cv::Rect(tracker_plugin::face_locs.faces[i].pixel_lu.x,
                                            tracker_plugin::face_locs.faces[i].pixel_lu.y,
                                            tracker_plugin::face_locs.faces[i].width.data,
                                            tracker_plugin::face_locs.faces[i].height.data )).clone());

      //Now set this mat to the value of the QImage tag in the space.
      QImage qimage = QImage(face_images[i].data, face_images[i].cols, face_images[i].rows, face_images[i].step[0], QImage::Format_RGB888);
      // QImage qimage= QImage((const unsigned char*)(newfaces.data), newfaces.cols, newfaces.rows, QImage::Format_RGB888);
      // ui.face_output_list->addItem(new QListWidgetItem(QIcon(QPixmap::fromImage(qimage)),"NewPic"));
      // qmap.push_back(QPixmap(QPixmap::fromImage(qimage)));
      qmap.push_back(qimage);

    }

    // Before emiting the signal let's do some processing here.
    cv::Mat im_gray;
    if (conversion_mat_.channels() > 1) {
      cv::cvtColor(conversion_mat_, im_gray, CV_BGR2GRAY);
    } else {
      im_gray = conversion_mat_.clone();
    }


    tracked_image_results.clear();
    tracked_image_mats.clear();

//This is where the initalization must occur
    //Now how do i setup initiailzation after tracking has been done.

    //Now let's remove this part into hand and multiple face detection without breaking hand tracking method.


    if (tracking_method == 0)
    {
      if (last_selected_item != -1)
      {
        std::cout << "It's Initialization" << std::endl;
        cv::Rect rect(tracker_plugin::face_locs.faces[last_selected_item].pixel_lu.x,
                      tracker_plugin::face_locs.faces[last_selected_item].pixel_lu.y,
                      tracker_plugin::face_locs.faces[last_selected_item].width.data,
                      tracker_plugin::face_locs.faces[last_selected_item].height.data );

        if (!im_gray.empty() && rect.area() > 50)
        {
          cmt.push_back(CMT());
          cmt.back().consensus.estimate_rotation = true;
          cmt.back().initialize(im_gray, rect);
        }
        else {
          std::cerr << "It's not initialized" << std::endl;
        }
      }
      hold_var = last_selected_item;
      last_selected_item = -1;
      //End of Initialization
      //The Problem Areas;
      int i = 0;
      // tracked_image_mats.clear();
      //The Tracking Areas;
      //Also Where Published Occurs.
      for (std::vector<CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
      { //Nothing Wrong till here.
        //Check for if it's initialized;
        if ((*v).initialized == true && !im_gray.empty())
        { //Nothing wrong here to.
          //nothing wrong here to.

          (*v).processFrame(im_gray);
          std::cout << "Bounding Rect" << std::endl;
          cv::Rect rect = (*v).bb_rot.boundingRect();
          cmt_tracker::Face tracker_description;

//Set the values here
          tracker_description.pixel_lu.x = rect.x;
          tracker_description.pixel_lu.y = rect.y;
          //Now place coordinates to the value Z from the
          //depth camera.
          tracker_description.pixel_lu.z = 0;
          tracker_description.height.data = rect.height;
          tracker_description.width.data = rect.width;
          tracker_description.quality.data = false;
          tracker_description.id.data = i;
          tracker_description.name.data = "Tracker " + SSTR(i);
          tracker_description.emotion_states.data = "Neutral";

          if (rect.area() > 50)//This condition is case where when no results are detected.
          {
            // std::cout << "Passed this value" << std::endl;
            std::cout << "The area is: " << rect.area() << std::endl;
            if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= conversion_mat_.cols && 0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= conversion_mat_.rows)
            {

              tracked_image_mats.push_back(conversion_mat_(rect).clone());

              QImage qimage = QImage(tracked_image_mats[i].data, tracked_image_mats[i].cols, tracked_image_mats[i].rows, tracked_image_mats[i].step[0], QImage::Format_RGB888);
              tracked_image_results.push_back(qimage);
              tracker_description.quality.data = true;
            }
            else
            {
              std::cout << "No Suitable Results" << std::endl;
              cv::Mat img(100, 100, CV_8UC3);
              img.setTo(cv::Scalar(5));

              tracked_image_mats.push_back(img.clone());

              QImage qimage = QImage(tracked_image_mats[i].data, tracked_image_mats[i].cols, tracked_image_mats[i].rows, tracked_image_mats[i].step[0], QImage::Format_RGB888);
              tracked_image_results.push_back(qimage);
            }
            i++;
          }
          tracker_locations.faces.push_back(tracker_description);

        }
        else {
          std::cerr << "The isn't processed yet. " << std::endl;

        }

      }
      tracker_locations_pub.publish(tracker_locations);

      emit updatefacelist();
    }
    else if (tracking_method == 1)
    {
      //===========================================================================================================================================================================================

      if (first_run_eyes == 0)
      {
        std::cout << "First Run setting the faces for tracking" << std::endl;
        for (int i = 0; i < tracker_plugin::face_locs.faces.size(); i++)
        {
          cv::Rect rect(tracker_plugin::face_locs.faces[i].pixel_lu.x,
                        tracker_plugin::face_locs.faces[i].pixel_lu.y,
                        tracker_plugin::face_locs.faces[i].width.data,
                        tracker_plugin::face_locs.faces[i].height.data );

          if (!im_gray.empty() && rect.area() > 50)
          {
            //Now here update the user interface to include what is being tracked. 
            ui.tracker_output_list->selectAll(); 
            int selected_items_cout= ui.face_output_list->selectedItems().count(); 
            std::cout<<"SelectedItems Count"<<std::endl; 
            auto_emit= selected_items_cout; 
            //Now set the index of the items. 


            cmt.push_back(CMT());
            cmt.back().consensus.estimate_rotation = true;
            cmt.back().initialize(im_gray, rect);
          }
          else {
            std::cerr << "It's not initialized" << std::endl;
          }
        }
        std::cout << "Has done initializing with the faces" << std::endl;

        //Now it's essential that we have the at least a tracker set to stop at this stage.
        if (cmt.size() > 0)
        {
          first_run_eyes = -1;
        }
      }
      int i = 0;
      for (std::vector<CMT>::iterator v = cmt.begin(); v != cmt.end(); ++v)
      { //Nothing Wrong till here.
        //Check for if it's initialized;
        if ((*v).initialized == true && !im_gray.empty())
        {

          (*v).processFrame(im_gray);
          std::cout << "Bounding Rect" << std::endl;
          cv::Rect rect = (*v).bb_rot.boundingRect();
          cmt_tracker::Face tracker_description;

//Set the values here
          tracker_description.pixel_lu.x = rect.x;
          tracker_description.pixel_lu.y = rect.y;
          //Now place coordinates to the value Z from the
          //depth camera.
          tracker_description.pixel_lu.z = 0;
          tracker_description.height.data = rect.height;
          tracker_description.width.data = rect.width;
          tracker_description.quality.data = false;
          tracker_description.id.data = i;
          tracker_description.name.data = "Tracker " + SSTR(i);
          tracker_description.emotion_states.data = "Neutral";

          if (rect.area() > 50)//This condition is case where when no results are detected.
          {
            // std::cout << "Passed this value" << std::endl;
            std::cout << "The area is: " << rect.area() << std::endl;
            if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= conversion_mat_.cols && 0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= conversion_mat_.rows)
            {

              tracked_image_mats.push_back(conversion_mat_(rect).clone());
              QImage qimage = QImage(tracked_image_mats[i].data, tracked_image_mats[i].cols, tracked_image_mats[i].rows, tracked_image_mats[i].step[0], QImage::Format_RGB888);
              tracked_image_results.push_back(qimage);
              tracker_description.quality.data = true;
            }
            else
            {
              std::cout << "No Suitable Results" << std::endl;
              cv::Mat img(100, 100, CV_8UC3);
              img.setTo(cv::Scalar(5));

              tracked_image_mats.push_back(img.clone());

              QImage qimage = QImage(tracked_image_mats[i].data, tracked_image_mats[i].cols, tracked_image_mats[i].rows, tracked_image_mats[i].step[0], QImage::Format_RGB888);
              tracked_image_results.push_back(qimage);
            }
            i++;
            locations_of_trackers.push_back(rect);
          }
          tracker_locations.faces.push_back(tracker_description);

        }
        else {
          std::cerr << "The isn't processed yet. " << std::endl;

        }

      }
      //Now that we have finished the processing of the tracking areas; We can set masks
      //to them and thusly create new tracking nodes if necessary.

      //Now apply black masks to im_gray and pass it thru the tracker.

      //Iterate over the locations of trackers;
      cv::Mat imageROI = im_gray.clone();

      for (size_t i = 0; i < locations_of_trackers.size(); i++)
      {
        // cv::Mat imageTracker = imageROI(locations_of_trackers[i]);
        // std::cout<<"The Size of the mat is: "<<imageTracker.size()<<std::endl;
        cv::Mat mask(locations_of_trackers[i].size(), CV_16UC1);
        std::cout << "The size of the maskis: " << mask.size() << std::endl;

        mask.setTo(cv::Scalar(5));

        std::cout << "The error is triggered here" << std::endl;
        if (0 <= locations_of_trackers[i].x && 0 <= locations_of_trackers[i].width && locations_of_trackers[i].x + locations_of_trackers[i].width <= imageROI.cols
            && 0 <= locations_of_trackers[i].y && 0 <= locations_of_trackers[i].height && locations_of_trackers[i].y + locations_of_trackers[i].height <= imageROI.rows)
        {
          mask.copyTo(imageROI(locations_of_trackers[i]));
          std::cout << "Hopefully " << std::endl;
        }
      }
      locations_of_trackers.clear();
      //Now check the image;
      face_cascade.detectMultiScale( imageROI, candidate_faces , 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(40, 40) );
      for (size_t i = 0; i < candidate_faces.size(); i++)
      {
        cv::Mat face_Frame(imageROI(candidate_faces[i]));
        eyes_cascade.detectMultiScale( face_Frame, candidates_eyes, 1.1, 2, 0 | cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );
        size_t eyes = 2;
        if (candidates_eyes.size() == eyes)
        {
          if (!im_gray.empty() && candidate_faces[i].area() > 50)
          {
            //Now there must be some way to hold back setting up new tracker;
            cmt.push_back(CMT());
            cmt.back().consensus.estimate_rotation = true;
            cmt.back().initialize(im_gray, candidate_faces[i]);
          }
        }
      }

      //Now let's do some assigning for the system.
      image_published = cv_bridge::CvImage(std_msgs::Header(), "mono8", imageROI).toImageMsg();
      image_publisher.publish(image_published);

      tracker_locations_pub.publish(tracker_locations);

      emit updatefacelist();

      //===========================================================================================================================================================================================
    }
    //This where the definition needs to end.
  }
  else
  {
    std::cout << "Skipping Frame: " << frame << std::endl;
  }
  frame++;
  // conversion_mat_previous = conversion_mat_.clone();
  // image = QImage(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);

  // ros::Rate rate(1);



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
    if(auto_emit < 0)
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