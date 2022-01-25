/**
 * Copyright (c) 2017, California Institute of Technology.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the California Institute of
 * Technology.
 */

#include "apriltag_ros/single_image_detector.h"

#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>

#include <apriltag_ros/HceSingleImage.h>

namespace apriltag_ros
{

SingleImageDetector::SingleImageDetector (ros::NodeHandle& nh,
                                          ros::NodeHandle& pnh) :
    tag_detector_(pnh)
{
  // Advertise the single image analysis service
  single_image_analysis_service_ =
      nh.advertiseService("single_image_tag_detection",
                          &SingleImageDetector::analyzeImage, this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
      // nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  // ROS_INFO_STREAM("Ready to do tag detection on single images");
    ROS_INFO_STREAM("\n <Apriltag_ros server> Ready to do tag detection");

}

bool SingleImageDetector::analyzeImage(
    apriltag_ros::HceSingleImage::Request& request,
    apriltag_ros::HceSingleImage::Response& response)
    // apriltag_ros::AnalyzeSingleImage::Request& request,
    // apriltag_ros::AnalyzeSingleImage::Response& response)
{
  ROS_INFO("\n <Apriltag_ros server> callback");
  // ROS_INFO("[ Summoned to analyze image ]");
  // ROS_INFO("Image load path: %s",
  //          request.full_path_where_to_get_image.c_str());
  // ROS_INFO("Image save path: %s",
  //          request.full_path_where_to_save_image.c_str());

  // Read the image
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(request.img0, sensor_msgs::image_encodings::BGR8);
  cv::Mat image = cv_ptr->image;

  // cv::Mat image = cv::imread(request.full_path_where_to_get_image,
  //                            cv::IMREAD_COLOR);
  if (image.data == NULL)
  {
    // Cannot read image
    // ROS_ERROR_STREAM("Could not read image " <<
    //                  request.full_path_where_to_get_image.c_str());
    ROS_ERROR_STREAM("No image in request of apriltag_ros::HceSingleImage"); 
    return false;
  }
  
  // Detect tags in the image
  cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::Header(),
                                                            "bgr8", image));
  loaded_image->header.frame_id = "camera";
  AprilTagDetectionArray tag_centers_tmp =
      tag_detector_.detectTags(loaded_image);
      // tag_detector_.detectTags(loaded_image,sensor_msgs::CameraInfoConstPtr(
      //     new sensor_msgs::CameraInfo(request.camera_info)));
 
  std::cout << " <Apriltag_ros server> How many Apriltags were found? " << tag_centers_tmp.detections.size() << std::endl;
  
  // success: over than 1 tag detected
  if (tag_centers_tmp.detections.size() > 0)
  {
    response.success = 1;
  }
  response.tag_detections.detections = tag_centers_tmp.detections;
  
  for (int i = 0; i < tag_centers_tmp.detections.size(); ++i)
    {
      std::cout<<" <Apriltag_ros server> ========== "<<"id = "<< response.tag_detections.detections.at(i).id.at(0) <<" =========="<<std::endl;
    }

  if (tag_centers_tmp.detections.size() != 0)


  // Publish detected tags (AprilTagDetectionArray, basically an array of
  // geometry_msgs/PoseWithCovarianceStamped)

  // tag_detections_publisher_.publish(response.detections);
  // tag_centers_tmp.header.stamp = ros::Time::now(); // time
  // tag_centers_tmp.header.stamp.sec = ros::Time::now().toSec();
  // tag_centers_tmp.header.stamp.nsec = ros::Time::now().toNSec();
    
  tag_detections_publisher_.publish(response.tag_detections);

  // Save tag detections image
  tag_detector_.drawDetections(loaded_image);
  cv::imwrite("/home/junhakim/service_test_results/result.png", loaded_image->image);

  ROS_INFO("\n <Apriltag_ros server> Done!\n");

  return true;
}

} // namespace apriltag_ros
