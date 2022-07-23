#include "ros/ros.h"



#include "sensor_msgs/Image.h"
#include<std_msgs/Int64.h>
#include<std_msgs/String.h>
#include <geometry_msgs/Pose.h>
// #include <string>
// #include <cv_bridge/cv_bridge.h> //to convert ros imamge to image in cv format
// #include <image_transport/image_transport.h>


// // #include <Eigen/Geometry>
// #include <Eigen/Core>
// #include <pcl/range_image/range_image.h>
// #include <pcl/io/pcd_io.h>

// #include <opencv2/core.hpp>
// #include "opencv2/opencv.hpp"
// #include <opencv2/highgui.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/core/affine.hpp>


#include "frank/GenerateRangeImage.h"




int main(int argc, char **argv){
  ros::init(argc, argv, "rangeimage_client");
  ros::NodeHandle n;
  ROS_INFO("Ready to generate range image");
  ros::ServiceClient client = n.serviceClient<frank::GenerateRangeImage>("/generate_range_image");
  frank::GenerateRangeImage srv;
  srv.request.req=  atoll(argv[1]);

  if(client.call(srv)){
    ROS_INFO("Sum: %d", (int)srv.response.res);
  }else{
    ROS_ERROR("Failed to call service generate range image");
     return 1;
  }

  ros::spin();

  return 0;
}