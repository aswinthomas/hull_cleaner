
#ifndef _ROBOT_UTIL_HH
#define _ROBOT_UTIL_HH

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

namespace RobotUtil {

  geometry_msgs::Quaternion getQuatFromEuler(double roll,double pitch,double yaw) {
    tf::Quaternion quatTf=tf::createQuaternionFromRPY(roll,pitch,yaw);
    //tf::Quaternion quatTf(0,0,0,1);
    geometry_msgs::Quaternion quat;
    //tf::Matrix3x3(quatTf).setRPY(roll,pitch,yaw);
    tf::quaternionTFToMsg(quatTf,quat);
    return quat;
  }

  double getYawFromQuat(geometry_msgs::Quaternion quat) {
    double roll,pitch,yaw;
    tf::Quaternion quatTf;
    tf::quaternionMsgToTF(quat,quatTf);
    tf::Matrix3x3(quatTf).getRPY(roll,pitch,yaw);
    return angles::normalize_angle(yaw);
  }

  void rotateAboutZAxis(geometry_msgs::Quaternion &quat,double yaw) {
    tf::Quaternion quatTf;
    tf::quaternionMsgToTF(quat,quatTf);
    quatTf.setRotation(tf::Vector3(0,0,1),yaw);
    tf::quaternionTFToMsg(quatTf,quat);
  }
}

#endif
