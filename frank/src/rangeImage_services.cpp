#include "ros/ros.h"



#include "sensor_msgs/Image.h"
#include<std_msgs/Int64.h>
#include<std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <cv_bridge/cv_bridge.h> //to convert ros imamge to image in cv format
#include <image_transport/image_transport.h>


#include <Eigen/Geometry>
#include <Eigen/Core>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>


#include "frank/GenerateRangeImage.h"


// void imageCb(const sensor_msgs::ImageConstPtr& msg){
//     cv_bridge::CvImagePtr cv_ptr;
//     try{
//       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception& e){
//       ROS_ERROR("cv_bridge exception: %s", e.what());
//       return;
//     }

//   }
 bool generateRangeImage(frank::GenerateRangeImage::Request &req,
                        frank::GenerateRangeImage::Response &res ){


        std::string dense_pc,dense_folder;
        ros::param::get("/FilePath/dense",dense_folder);
        dense_pc = dense_folder + "/PointCloud_0.pcd" ;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (dense_pc, *cloud_) == -1) {
          PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
          return (-1);
        } 



        float angularResolution_x = (float) (0.01 * (M_PI/180.0f));  // 0.01
        float angular_resolution_y = (float) (0.01 * (M_PI/180.0f)); // 0.01
        float maxAngleWidth     = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
        float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians

        Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);;
        pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        float noiseLevel=0.00;
        float minRange = 0.0f;
        int borderSize = 100;

   	    pcl::RangeImage rangeImage;      
           // Creating Range image
        rangeImage.createFromPointCloud(*cloud_, angularResolution_x,angular_resolution_y, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
        rangeImage.setUnseenToMaxRange();
      
        // normalize
        float min, max;
        rangeImage.getMinMaxRanges(min,max);

        cv::Mat range_map(rangeImage.height, rangeImage.width, CV_8UC3);

        // create range map or binary image
        for (int h = 0; h < rangeImage.height; h++){
            for (int w = 0; w < rangeImage.width; w++){   
                if(isinf(rangeImage.at(w,h).range) || rangeImage.at(w,h).range > 500 ){
                range_map.at<cv::Vec3b>(h,w)[0] = 0;
                range_map.at<cv::Vec3b>(h,w)[1] = 0;
                range_map.at<cv::Vec3b>(h,w)[2] = 0;

            }else{
                range_map.at<cv::Vec3b>(h,w)[0]= ((rangeImage.at(w,h).range) / max)*255; // Normalize for color image
                range_map.at<cv::Vec3b>(h,w)[1]= ((rangeImage.at(w,h).range) / max)*255;
                range_map.at<cv::Vec3b>(h,w)[2]= ((rangeImage.at(w,h).range) / max)*255;
          }
        }  
      }
      // // convert cv image to ros message
      // sensor_msgs::ImageConstPtr msg;
      // imageCb(msg);
      // res.image = *msg;


      std::string imagePath = dense_folder + "/rangeImage_0.png";
      res.error="Image is generated";
      res.res =1;
      cv::imwrite(imagePath,range_map);
      ROS_INFO("Image generated!");

      return res.res;
}


// bool check_ready(frank::rangeimage_ready::Request &req,
//                 frank::rangeimage_ready::Response &res){

//     ROS_INFO("Check if ready to create range image");
//     res.number_2 = req.number_1 + 1;
//     return true;
// }

// void chatterCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }

int main(int argc, char **argv){
  ros::init(argc, argv, "rangeimage_service");
  ros::NodeHandle n;
  ROS_INFO("Ready to generate range image");
  
  ros::ServiceServer ready_range = n.advertiseService("/generate_range_image", generateRangeImage);
  // ros::Subscriber sub = n.subscribe("asd",1000,imgCallback);

  ros::spin();

  return 0;
}