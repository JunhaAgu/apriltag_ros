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

namespace apriltag_ros
{

SingleImageDetector::SingleImageDetector (ros::NodeHandle& nh,
                                          ros::NodeHandle& pnh,
                                          std::string& dir_txt) :
    tag_detector_(pnh)
{
  // Advertise the single image analysis service
  single_image_analysis_service_ =
      nh.advertiseService("/service_dump_detector",
                          &SingleImageDetector::analyzeImage, this);
  tag_detections_publisher_ =
      nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
      // nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  // ROS_INFO_STREAM("Ready to do tag detection on single images");
    ROS_INFO_STREAM("\n <Apriltag_ros server> Ready to do tag detection");

  id_candi_.reserve(6);
  tag_trans_.reserve(6);

  readVertices(dir_txt);
}

void SingleImageDetector::readVertices(std::string& dir_txt)
{
  std::string a0_dir = dir_txt + "a0_vertices.txt";
  std::string a1_dir = dir_txt + "a1_vertices.txt";
  std::string a2_dir = dir_txt + "a2_vertices.txt";
  std::string a3_dir = dir_txt + "a3_vertices.txt";
  std::string a4_dir = dir_txt + "a4_vertices.txt";
  std::string a5_dir = dir_txt + "a5_vertices.txt";

  // std::cout <<a0_dir<<std::endl;

  double x;
  double y;
  double z;

  Point3D point_tmp;

  std::ifstream file0(a0_dir);
	if( file0.is_open() ){
		std::string line;

    int cnt = 0;
    getline(file0, line);
		while(getline(file0, line)){
      std::stringstream ss;
      ss << line;
      ss >> point_tmp[0];
      ss >> point_tmp[1];
      ss >> point_tmp[2];
      a0_vertices_.push_back(point_tmp);
      // std::cout << line << std::endl;
      // std::cout << x << std::endl;
      // std::cout << y << std::endl;
      // std::cout << z << std::endl;
      // exit(0);
      cnt += 1;
		}
		file0.close();
	}
  // std::cout << a0_vertices_[0] << std::endl;
  // std::cout << a0_vertices_[1] << std::endl;
  // std::cout << a0_vertices_[2] << std::endl;
  // std::cout << a0_vertices_[3] << std::endl;
  // std::cout << a0_vertices_[4] << std::endl;
  // std::cout << a0_vertices_[5] << std::endl;
  // std::cout << a0_vertices_[6] << std::endl;
  // std::cout << a0_vertices_[7] << std::endl;

  std::ifstream file1(a1_dir);
	if( file1.is_open() ){
		std::string line;

    int cnt = 0;
    getline(file1, line);
		while(getline(file1, line)){
      std::stringstream ss;
      ss << line;
      ss >> point_tmp[0];
      ss >> point_tmp[1];
      ss >> point_tmp[2];
      a1_vertices_.push_back(point_tmp);
      cnt += 1;
		}
		file1.close();
	}

  std::ifstream file2(a2_dir);
	if( file2.is_open() ){
		std::string line;

    int cnt = 0;
    getline(file2, line);
		while(getline(file2, line)){
      std::stringstream ss;
      ss << line;
      ss >> point_tmp[0];
      ss >> point_tmp[1];
      ss >> point_tmp[2];
      a2_vertices_.push_back(point_tmp);
      cnt += 1;
		}
		file2.close();
	}

  std::ifstream file3(a3_dir);
	if( file3.is_open() ){
		std::string line;

    int cnt = 0;
    getline(file3, line);
		while(getline(file3, line)){
      std::stringstream ss;
      ss << line;
      ss >> point_tmp[0];
      ss >> point_tmp[1];
      ss >> point_tmp[2];
      a3_vertices_.push_back(point_tmp);
      cnt += 1;
		}
		file3.close();
	}

  std::ifstream file4(a4_dir);
	if( file4.is_open() ){
		std::string line;

    int cnt = 0;
    getline(file4, line);
		while(getline(file4, line)){
      std::stringstream ss;
      ss << line;
      ss >> point_tmp[0];
      ss >> point_tmp[1];
      ss >> point_tmp[2];
      a4_vertices_.push_back(point_tmp);
      cnt += 1;
		}
		file4.close();
	}

  std::ifstream file5(a5_dir);
	if( file5.is_open() ){
		std::string line;

    int cnt = 0;
    getline(file5, line);
		while(getline(file5, line)){
      std::stringstream ss;
      ss << line;
      ss >> point_tmp[0];
      ss >> point_tmp[1];
      ss >> point_tmp[2];
      a5_vertices_.push_back(point_tmp);
      cnt += 1;
		}
		file5.close();
	}
}

bool SingleImageDetector::analyzeImage(
    hce_msgs::CallDumpDetector::Request& request,
    hce_msgs::CallDumpDetector::Response& response)
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

  tag_detector_.c_fx = request.fx;
  tag_detector_.c_fy = request.fy;
  tag_detector_.c_cx = request.cx;
  tag_detector_.c_cy = request.cy;

  tag_detector_.c_dist_k1 = request.dist_k1;
  tag_detector_.c_dist_k2 = request.dist_k2;
  tag_detector_.c_dist_p1 = request.dist_p1;
  tag_detector_.c_dist_p2 = request.dist_p2;
    
  // Detect tags in the image
  cv_bridge::CvImagePtr loaded_image(new cv_bridge::CvImage(std_msgs::Header(),
                                                            "bgr8", image));
  loaded_image->header.frame_id = "camera";
  AprilTagDetectionArray tag_centers_tmp =
      tag_detector_.detectTags(loaded_image);
      // tag_detector_.detectTags(loaded_image,sensor_msgs::CameraInfoConstPtr(
      //     new sensor_msgs::CameraInfo(request.camera_info)));
 
  std::cout << " <Apriltag_ros server> How many Apriltags were found? " << tag_centers_tmp.detections.size() << std::endl;
  
  // success: over than 2 tag detected
  if (tag_centers_tmp.detections.size() > 1)
  {
    response.success = 1;
  }
  
  // response.tag_detections.detections = tag_centers_tmp.detections;

  for (int i = 0; i < tag_centers_tmp.detections.size(); ++i)
  {
    response.tag_pose.push_back(tag_centers_tmp.detections[i].pose.pose.pose);
    response.tag_id.push_back(tag_centers_tmp.detections[i].id[0]);

    std::cout << " <Apriltag_ros server> ========== "
              << "id = " << tag_centers_tmp.detections[i].id[0] << " ==========" << std::endl;
    // std::cout << tag_centers_tmp.detections[i].pose.pose.pose.position << std::endl;
    printf("id = %d, {x, y, z} = {%.8f, %.8f, %.8f}\n",
     tag_centers_tmp.detections[i].id[0], 
     tag_centers_tmp.detections[i].pose.pose.pose.position.x, 
     tag_centers_tmp.detections[i].pose.pose.pose.position.y, 
     tag_centers_tmp.detections[i].pose.pose.pose.position.z );

     printf("quat: , {x, y, z, w} = {%.8f, %.8f, %.8f, %.8f}\n",
     tag_centers_tmp.detections[i].pose.pose.pose.orientation.x, 
     tag_centers_tmp.detections[i].pose.pose.pose.orientation.y, 
     tag_centers_tmp.detections[i].pose.pose.pose.orientation.z,
     tag_centers_tmp.detections[i].pose.pose.pose.orientation.w
      );
  }

  id_candi_.resize(0);
  tag_trans_.resize(0);
  tag_quat_.resize(0);

  double x;
  double y;
  double z;
  geometry_msgs::Point point_out_tmp;

  if (tag_centers_tmp.detections.size() > 1)
  {
    this->calculateBBVertex(tag_centers_tmp);
    for (int i=0; i<8; ++i)
    {
      point_out_tmp.x = vertices_out_[i][0];
      point_out_tmp.y = vertices_out_[i][1];
      point_out_tmp.z = vertices_out_[i][2];
      response.bb_vertices.push_back(point_out_tmp);
    }
    
  }
  // if (tag_centers_tmp.detections.size() != 0)


  // Publish detected tags (AprilTagDetectionArray, basically an array of
  // geometry_msgs/PoseWithCovarianceStamped)

  // tag_detections_publisher_.publish(response.detections);
  // tag_centers_tmp.header.stamp = ros::Time::now(); // time
  // tag_centers_tmp.header.stamp.sec = ros::Time::now().toSec();
  // tag_centers_tmp.header.stamp.nsec = ros::Time::now().toNSec();
    
  // tag_detections_publisher_.publish(tag_centers_tmp.detections);

  // Save tag detections image
  tag_detector_.drawDetections(loaded_image);
  cv::imwrite("/home/junhakim/service_test_results/result.png", loaded_image->image);

  // cv::namedWindow("detected tags");
  // cv::imshow("detected tags", loaded_image->image);
  // cv::waitKey(0);

  ROS_INFO("\n <Apriltag_ros server> Done!\n");

  return true;
}

void SingleImageDetector::calculateBBVertex(AprilTagDetectionArray& tag_centers)
{ 
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 0.0;

  double dist = 0.0;
  double dist_min = 1e5;
  double dist_min_idx = 100;
  int dist_min_id = -1;
  Pose T_c0aX;

  trans trans_tmp;
  quat quat_tmp;
  Rot rot;

  Rot rot_pi;
  rot_pi << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;

  for (int i = 0; i < tag_centers.detections.size(); ++i)
  {
    id_candi_.push_back(tag_centers.detections[i].id[0]);

    geometry_msgs::Pose& pose_tmp = tag_centers.detections[i].pose.pose.pose;
    x = pose_tmp.position.x;
    y = pose_tmp.position.y;
    z = pose_tmp.position.z;
    trans_tmp << x , y ,z ;
    tag_trans_.push_back(trans_tmp);
    
    qx = pose_tmp.orientation.x;
    qy = pose_tmp.orientation.y;
    qz = pose_tmp.orientation.z;
    qw = pose_tmp.orientation.w;
    quat_tmp << qx, qy, qz, qw;
    tag_quat_.push_back(quat_tmp);

    dist = SQ_SUM(x,y,z);
    if (dist < dist_min)
    {
      dist_min = dist;
      dist_min_id = id_candi_[i];
      dist_min_idx = i;
    }
  }
  std::cout << "dist_min id = " << "[" <<dist_min_id << "]" << std::endl;
  // calculateMinDistance
  
  qx = tag_quat_[dist_min_idx][0];
  qy = tag_quat_[dist_min_idx][1];
  qz = tag_quat_[dist_min_idx][2];
  qw = tag_quat_[dist_min_idx][3];
  rot << 2*(qw*qw+qx*qx)-1,   2*(qx*qy-qw*qz),    2*(qx*qz+qw*qy),
          2*(qx*qy+qw*qz),    2*(qw*qw+qy*qy)-1,  2*(qy*qz-qw*qx),
          2*(qx*qz-qw*qy),    2*(qy*qz+qw*qx),    2*(qw*qw+qz*qz)-1;
  
  //for matlab orientation (rotate 180 degree in z-axis direction)
  T_c0aX.block(0,0,3,3) = rot*rot_pi;
  T_c0aX.block(0,3,3,1) = tag_trans_[dist_min_idx];
  T_c0aX(3,3) = 1;
  std::cout << T_c0aX <<std::endl;

  switch(dist_min_id)
  {
  case 0:
    for (int i = 0; i < 8; ++i)
    {
      vertices_out_.push_back(T_c0aX.block(0, 0, 3, 3) * a0_vertices_[i] + T_c0aX.block(0, 3, 3, 1));
    }

  case 1:
    for (int i = 0; i < 8; ++i)
    {
      vertices_out_.push_back(T_c0aX.block(0, 0, 3, 3) * a1_vertices_[i] + T_c0aX.block(0, 3, 3, 1));
    }

  case 2:
    for (int i = 0; i < 8; ++i)
    {
      vertices_out_.push_back(T_c0aX.block(0, 0, 3, 3) * a2_vertices_[i] + T_c0aX.block(0, 3, 3, 1));
    }

  case 3:
    for (int i = 0; i < 8; ++i)
    {
      vertices_out_.push_back(T_c0aX.block(0, 0, 3, 3) * a3_vertices_[i] + T_c0aX.block(0, 3, 3, 1));
    }

  case 4:
    for (int i = 0; i < 8; ++i)
    {
      vertices_out_.push_back(T_c0aX.block(0, 0, 3, 3) * a4_vertices_[i] + T_c0aX.block(0, 3, 3, 1));
    }

  case 5:
    for (int i = 0; i < 8; ++i)
    {
      vertices_out_.push_back(T_c0aX.block(0, 0, 3, 3) * a5_vertices_[i] + T_c0aX.block(0, 3, 3, 1));
    }
  }
}

} // namespace apriltag_ros
