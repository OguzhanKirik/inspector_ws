// #include "ros/ros.h"

// #include "std_msgs/String.h"
// #include "sensor_msgs/Image.h"

// #include <sstream>


// int main (int argc, char **argv){
//     ros::init(argc,argv,"t");
//     ros::NodeHandle n;

//     //ros::Publisher pub  = n.advertise<sensor_msgs::Image>("rangeImage",5);
//     ros::Publisher pub  = n.advertise<std_msgs::String>("rangeImage",1);

//     ros::Rate loop_rate(1);

//     int count = 0;
//     while (ros::ok()){
//     /**
//      * This is a message object. You stuff it with data, and then publish it.
//      */
//    std_msgs::String msg;

//     std::stringstream ss;
//     ss << "hello world " << count;
//     msg.data = ss.str();

//     ROS_INFO("%s", msg.data.c_str());
//     // ROS_INFO("image publihed");

//     pub.publish(msg);

  
//     ros::spinOnce();
//         loop_rate.sleep();

//     ++count;
//   }

  



//     return 0;
// }


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  
  //cv::Mat image = cv::imread(argv[1],cv::IMREAD_COLOR);
  cv::Mat image = cv::imread("/home/oguz/Desktop/parrot-picture.jpg", cv::IMREAD_COLOR );
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);

//  For ROS C++, ros::spin() is mandatory for subscribers. If you are subscribing messages, 
//  services or actions you must call ros::spin() to process the events. While spinOnce() 
//  handles the events and returns immediately,
//   spin() blocks until ROS invokes a shutdown. spinOnce() is used where other lines of 
//   code need to be executed along with processing arriving messages.


  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    //ros::spin();
    loop_rate.sleep();
  }


}