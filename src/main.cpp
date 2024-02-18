#include <ros/package.h>
#include <ros/ros.h>
#include <iostream>
#include <ctime>

#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;
using namespace message_filters;

#define default_rgb_topic "/camera/rgb/image_raw"
#define default_depth_topic "/camera/depth/image_raw"
#define default_timer_time 3

string img_path = ros::package::getPath("rgbd_saver") + "/img/";
string rgb_topic;
string depth_topic;
string timer_topic;

Mat rgb_frame, dpt_frame;
cv_bridge::CvImagePtr rgb_ptr;
cv_bridge::CvImagePtr dpt_ptr;

void RGBDcallback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth) {  
  
  try { 
    rgb_ptr = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::BGR8); }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert '%s' format", e.what()); }
 
  try{
    dpt_ptr = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1); }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert '%s' format", e.what()); }

  time_t now = time(0);
  string now_str = to_string(now);
  rgb_frame = rgb_ptr->image;
  dpt_frame = dpt_ptr->image;
  dpt_frame.convertTo(dpt_frame, CV_16UC1);

  cout << "Writing     ..." << endl;
  cout << "Say cheese  ..." << endl;
  imwrite(img_path + "rgb_" + now_str + ".png", rgb_ptr->image);
  imwrite(img_path + "depth_" + now_str + ".tiff", dpt_ptr->image);
  sleep(default_timer_time);
  cout << "Done!       ..." << endl;
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rgbd_saver");
  ros::NodeHandle n;

  if (rgb_topic.size()==0) rgb_topic=default_rgb_topic;
  n.getParam("/rgbd/rgb",rgb_topic);

  if (depth_topic.size()==0) depth_topic=default_depth_topic;
  n.getParam("/rgbd/depth",depth_topic);

  if (timer_topic.size()==0) timer_topic=default_timer_time;
  n.getParam("/rgbd/time",timer_topic);

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(n, rgb_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, depth_topic, 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&RGBDcallback, _1, _2));

  ros::spin();   
  return 0;
}