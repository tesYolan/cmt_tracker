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
#include <vector>
//The Ui Element Definition
#include "ui_tracker_plugin.h"

// CMT libraryies
#include "CMT.h"
#include "gui.h"
#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif
// CMT Libraries End

//Thread Saftey of the Application
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>


using cmt::CMT;
namespace rqt_tracker_view {
class tracker_plugin
    : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

public:
    //To hold last value of face_locs;
    cv::CascadeClassifier face_cascade;
    cv::CascadeClassifier eyes_cascade;
    std::vector<cv::Rect> candidate_faces; 
    std::vector<cv::Rect> candidates_eyes; 
    bool setup; 
    cmt_tracker::Faces face_locs;
    cmt_tracker::Faces tracker_locations;
    //Subscribers for location of faces and the image_subscriber
    ros::Publisher tracker_locations_pub;
    ros::Subscriber face_subscriber;
    image_transport::Subscriber image_subscriber;
    image_transport::Publisher image_publisher;
    sensor_msgs::ImagePtr image_published;
    std::string subscribe_topic; 
    //Nodehandle from the rqt plugin.
    ros::NodeHandle nh;

    //Constructor.
    tracker_plugin();

    void handbasedtracking_rules();
    void eyesbasedtracking_rules();

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

protected:
    std::vector<QImage> face_image;
    std::vector<QImage> qmap;
    std::vector<QImage> tracked_image_results;
    std::vector<cv::Mat> tracked_image_mats;
    std::vector<cv::Mat> face_images;
    std::vector<int> quality_of_tracker;
    std::vector<cv::Rect> locations_of_trackers; //for setting the tracking to higher levels.


    cv::Mat conversion_mat_;
    cv::Mat im_gray;
    cv::Mat conversion_mat_previous;
    Ui::tracker_plugin ui;
    QWidget* widget_;

    boost::mutex mat_mutex;
    //Doubling CMT on the running model.

    std::vector<CMT> cmt;
    cv::Rect selected_rect;
    int last_selected_item;
    int hold_var;
    int frame;
    int tracking_method;
    int first_run_eyes;
    int auto_emit; 
};
}
#endif // TRACKER_PLUGIN_H
