#ifndef TRACKER_PLUGIN_H
#define TRACKER_PLUGIN_H

#include <QWidget>
#include <rqt_gui_cpp/plugin.h>
#include <ros/macros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cmt_tracker/Faces.h>
#include <cmt_tracker/Tracker.h>
#include <cmt_tracker/Trackers.h>
#include <vector>
//The Ui Element Definition
#include "ui_tracker_plugin.h"



namespace rqt_tracker_view {
class tracker_plugin
    : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    //To hold last value of face_locs;
    cmt_tracker::Faces face_locs;
    cmt_tracker::Tracker track_location;
    cmt_tracker::Tracker track_published;
    cmt_tracker::Trackers tracking_results; 
    // cmt_tracker::Faces tracker_locations;
    // //Subscribers for location of faces and the image_subscriber
    ros::Publisher tracker_locations_pub;
    ros::Subscriber tracker_locations_sub;
    ros::Subscriber face_subscriber;
    ros::Subscriber tracked_locations; 
    ros::ServiceClient client; 
    image_transport::Subscriber image_subscriber;
    // image_transport::Publisher image_publisher;
    // sensor_msgs::ImagePtr image_published;
    // std::string subscribe_topic;
    // //Nodehandle from the rqt plugin.
    ros::NodeHandle nh;

    //Constructor.
    tracker_plugin();


    //Plugin Implementation.
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);

    virtual void shutdownPlugin();

signals:
    void updatefacelist();
protected slots:

    void on_MethodChanged(int index);

    void on_addItems_clicked();

    void updateVisibleFaces();

    void on_addToTrack_clicked(QListWidgetItem * item);

    void on_removeAllTracked_clicked();

    void on_removeTracked_clicked();

    void on_removeAllElements_clicked();

    void list_of_faces_update(const cmt_tracker::Faces& faces_info);

    void imageCb(const sensor_msgs::ImageConstPtr& msg);

    void trackerCb(const cmt_tracker::Tracker& tracker_locs);

    void tracker_resultsCb(const cmt_tracker::Trackers& tracker_results); 

protected:
    std::vector<QImage> face_images;
    std::vector<QImage> tracked_faces;
    // std::vector<QImage> qmap;
    std::vector<QImage> tracked_image_results;
    std::vector<std::string> tracked_image_information; 
    std::vector<cv::Mat> tracked_image_mats;
    std::vector<cv::Mat> mat_images;
    std::vector<cv::Mat> tracked_images;


    std::string subscribe_topic;
    cv::Mat conversion_mat_;
    // cv::Mat im_gray;
    // cv::Mat conversion_mat_previous;
    Ui::tracker_plugin ui;
    QWidget* widget_;
    bool tracker_updated;
    bool tracking_results_updated; 


    //Doubling CMT on the running model.

};
}
#endif // TRACKER_PLUGIN_H
