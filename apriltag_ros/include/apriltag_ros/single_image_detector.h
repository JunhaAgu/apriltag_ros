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
 *
 ** single_image_detector.h ****************************************************
 *
 * Wrapper class of TagDetector class which calls TagDetector::detectTags on a
 * an image stored at a specified load path and stores the output at a specified
 * save path.
 *
 * $Revision: 1.0 $
 * $Date: 2017/12/17 13:33:40 $
 * $Author: dmalyuta $
 *
 * Originator:        Danylo Malyuta, JPL
 ******************************************************************************/

#ifndef APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H
#define APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H

#include "apriltag_ros/common_functions.h"
#include <apriltag_ros/AnalyzeSingleImage.h>
#include <apriltag_ros/HceSingleImage.h>

#include "hce_msgs/CallDumpDetector.h"

#include <fstream> // ifstream header

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>

typedef Eigen::Matrix4d   Pose; 
typedef Eigen::Matrix3d   Rot;
typedef Eigen::Vector3d   trans;
typedef Eigen::Vector4d   quat;
typedef Eigen::Vector3d   Euler;
typedef Eigen::Vector3d   Point3D;

#define SQ_SUM(x, y, z) (((x) * (x) + (y) * (y) + (z) * (z)))

namespace apriltag_ros
{

  class SingleImageDetector
  {
  public:
  std::vector<int> id_candi_;
  std::vector<trans> tag_trans_;
  std::vector<quat> tag_quat_;

  std::vector<Point3D> a0_vertices_;
  std::vector<Point3D> a1_vertices_;
  std::vector<Point3D> a2_vertices_;
  std::vector<Point3D> a3_vertices_;
  std::vector<Point3D> a4_vertices_;
  std::vector<Point3D> a5_vertices_;

  std::vector<Point3D> vertices_out_;

  pcl::PointCloud<pcl::PointXYZ> bb_pcl_a0_;
  pcl::PointCloud<pcl::PointXYZ> bb_pcl_a1_;
  pcl::PointCloud<pcl::PointXYZ> bb_pcl_a2_;
  pcl::PointCloud<pcl::PointXYZ> bb_pcl_a3_;
  pcl::PointCloud<pcl::PointXYZ> bb_pcl_a4_;
  pcl::PointCloud<pcl::PointXYZ> bb_pcl_a5_;

  pcl::PointCloud<pcl::PointXYZ> bb_pcl_out_;

  private:
    TagDetector tag_detector_;
    ros::ServiceServer single_image_analysis_service_;

    ros::Publisher tag_detections_publisher_;

  public:
    SingleImageDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::string& dir_txt);

    // The function which provides the single image analysis service
    //   bool analyzeImage(apriltag_ros::AnalyzeSingleImage::Request& request,
    //                      apriltag_ros::AnalyzeSingleImage::Response& response);
    bool analyzeImage(hce_msgs::CallDumpDetector::Request &request,
                      hce_msgs::CallDumpDetector::Response &response);

    void calculateBBVertex(AprilTagDetectionArray &tag_centers);

    void readVertices(std::string& dir_txt);

    void readTruckPCL(std::string& dir_txt);
  };

} // namespace apriltag_ros

#endif // APRILTAG_ROS_SINGLE_IMAGE_DETECTOR_H
