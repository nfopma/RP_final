# include <ros/ros.h>
# include <image_transport/image_transport.h>
# include <cv_bridge/cv_bridge.h>
// not sure if these two are necessary, but are in the tutorial (http://wiki.ros.org/image_transport/Tutorials/SubscribingToImages) so I'll include them anyway
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/videoio.hpp>
#include <iostream>
#include <iomanip>

class ImageToDetection
{

  cv::HOGDescriptor hog_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_subscriber_;

public:
  ImageToDetection(ros::NodeHandle& nh)
    : it_(nh), hog_()
    //image_subscriber_(nh.subscribe("/prius/front_camera/image_raw", 10, &ImageToDetection::detect_person, this))
  {
    image_subscriber_ = it_.subscribe("/prius/front_camera/image_raw", 1, &ImageToDetection::detect_person, this);
    hog_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
  }

  void detect_person(const sensor_msgs::ImageConstPtr& image)
  {
    ROS_INFO_STREAM("Image message received by opencv_solution_node");

    cv_bridge::CvImagePtr converted_image = cv_bridge::toCvShare(image, "8UC4");
    std::vector<cv::Rect> found_locations;
    // Need to find out what kind of image I can send to detectMultiscale...
    hog_.detectMultiScale(converted_image->image, found_locations, 0, cv::Size(8,8), cv::Size(), 1.05, 2);
    ROS_INFO_STREAM("hog.detectMultiscale called succesfully");
    // Draw the rectangles
    for(int i = 0; i<found_locations.size(); i++)
    {
      cv::rectangle(converted_image->image, found_locations[i], (0, 255, 255), 1, 9, 0);
    }
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "opencv_solution_node");
  ros::NodeHandle nh;

  cv::namedWindow("view");
  cv::startWindowThread();

  ImageToDetection detector(nh);

  ros::spin();
  cv::destroyWindow("view");
}
