#ifndef _ROBOT_HH
#define _ROBOT_HH

#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include <std_msgs/String.h>
#include <angles/angles.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <hull_cleaner/RobotUtil.hh>

using namespace std;
using namespace RobotUtil;

//* Robot
/**
 * \brief This class describes the simulation robot and executes low level motor controls
 *
 * 1. Performs forward motion to a certain distance \n
 * 2. Performs Angular rotation to a certain yaw angle
 *
 * Parameters:
 * 1. kp_POS:           P parameter of the position control \n
 * 2. MAX_FWD_VEL       Maximum forward velocity in m/s \n
 * 3. MAX_ANG_RATE      Maximum Angular velocity in degree/s
 * 4. DIST_TOLERANCE    Tolerance in m for forward control \n
 * 5. ANG_TOLERANCE     Tolerance in degree for angular control \n
 * 6. RADIUS            Robot radius
 *
 */
class Robot {
private:
  static const double kp_POS = 0.03;
  static const double Kp_ANG = 5.0;
  static const double MAX_FWD_VEL = 1.0;
  static const double MAX_ANG_RATE = 60.0;
  static const double DIST_TOLERANCE = 0.1;
  static const double ANG_TOLERANCE = 0.01;
  static const double RADIUS = 0.15;

  //robot
  geometry_msgs::Pose pose;
  double maxFwdVel,maxAngVel,radius;

  //control
  double yawTarget;
  geometry_msgs::Point positionTarget;
  enum controlMode {move_fwd,turn,stop};
  controlMode mode;
  bool inControlLoop;
  void forwardControl();
  void turningControl();

  //simulation
  double freq;

  //subscribe and advertise


public:
  ///Construct and Destroy!
  /**
   * \brief Constructs the robot with a simulation speed input
   */
  Robot(double simSpeed);
  /**
   * \brief   Destroys this robot instance.
   */
  ~Robot(){ }

  /*!
   * \brief Main task
   *
   * This method comprises of the control loop
   *
   */
  void task();


  ///subscribe and advertise
  ros::Publisher pRobotMarker,pDirMarker,pStatus;
  /*!
   * \brief Planner command callback
   *
   * This method updates the planner command
   * \param msg ROS message string
   */
  void setCommand(const std_msgs::String::ConstPtr& msg);
  /*!
   * \brief Publisher
   *
   * This method publishes all non-display messages
   */
  void publishMsgs();



  ///getter and setter functions
  /**
   * \brief Returns robot radius
   */
  inline double getRadius() { return RADIUS;}
  /**
   * \brief Returns sim frequency
   */
  inline double getFreq() { return freq;}
  /**
   * \brief Sets sim frequency
   * \param frq simulation frequency
   */
  inline void setFreq(double frq) { freq=frq;}



  ///display related
  vector<geometry_msgs::Point> posHistory;
  /**
   * \brief This method publishes robot position marker
   */
  void drawRobot();
  /**
   * \brief This method publishes robot yaw marker
   */
  void drawDirection();
};

#endif
