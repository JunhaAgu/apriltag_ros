//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Server 
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

#include "apriltag_ros/common_functions.h"
#include <apriltag_ros/AnalyzeSingleImage.h>
#include "apriltag_ros/single_image_detector.h"

#include "apriltag_ros/hce_apriltag.h"

using namespace INTRINSIC;

// bool getRosParameter (ros::NodeHandle& pnh, std::string name, double& param)
// {
//   // Write parameter "name" from ROS Parameter Server into param
//   // Return true if successful, false otherwise
//   if (pnh.hasParam(name.c_str()))
//   {
//     pnh.getParam(name.c_str(), param);
//     ROS_INFO_STREAM("Set camera " << name.c_str() << " = " << param);
//     return true;
//   }
//   else
//   {
//     ROS_ERROR_STREAM("Could not find " << name.c_str() << " parameter!");
//     return false;
//   }
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltag_ros_hce");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  apriltag_ros::SingleImageDetector hce_tag_detector(nh, pnh);
  
  ros::spin();
}
