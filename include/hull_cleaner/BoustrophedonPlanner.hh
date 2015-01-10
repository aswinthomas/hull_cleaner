
#ifndef _BOUSTROPHEDON_PLANNER_HH
#define _BOUSTROPHEDON_PLANNER_HH

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <stdio.h>
#include <angles/angles.h>
#include <math.h>
#include <geometry_msgs/PolygonStamped.h>
#include <hull_cleaner/Map.hh>
#include <hull_cleaner/RobotUtil.hh>

using namespace std;
using namespace RobotUtil;

//* Robot
/**
 * \brief This class plans a path based on the boustrophedon criterion
 *
 * Parameters:
 * 1. MAP_LENGTH:       Length of the map
 * 2. MAP_HEIGHT:       Height of the map
 * 3. SEC_TO_HOUR:      Conversion from seconds to hour
 *
 */
class BoustrophedonPlanner {
private:
  static const double SMALL_POS_INCREMENT = 0.01;
  static const double OVERLAP_PERC = 20.0;
  static const double MAP_LENGTH = 10.0;
  static const double MAP_HEIGHT = 10.0;
  static const double SEC_TO_HOUR = 0.000277778;

  ///map
  Map *map;
  double filledPerc;

  //robot
  geometry_msgs::Pose robotPose;
  double robotRadius;
  bool robotInControlLoop;

  //planner
  enum planMode {fwd_long,fwd_short,right_turn90,left_turn90,stop};
  planMode mode;

  double timeLapsed,startTime;
  double simSpeed;

  void sendCommand(std::string cmd,double val=0.0);
  void turnLeft();
  void turnRight();
  void calculateMode();
  double updateMap();

public:
  ///Construct and Destroy!
  /**
   * \brief Constructs the planner with a simulation speed input
   */
  BoustrophedonPlanner(double sim_speed);
  /**
   * \brief   Destroys this robot instance.
   */
  ~BoustrophedonPlanner(){ }

  ///subscribe and advertise
  ros::Publisher pBoundary,pPercMarker,pCommand;
  /*!
   * \brief Robot status callback
   *
   * This method updates the robot status
   * \param msg ROS message string
   */
  void setRobotStatus(const std_msgs::String::ConstPtr& msg);


  ///getter and setter functions
  /**
   * \brief Returns robot radius
   */
  inline double getRobotRadius() { return robotRadius;}


  /**
   * \brief Initializes the map
   */
  void initializeMap();
  /*!
   * \brief Main task
   *
   * This method comprises of the planner loop
   *
   */
  void task();


  ///drawing methods
  /**
   * \brief This method publishes robot position marker
   */
  void drawPercentage();
  /**
   * \brief This method publishes robot position marker
   */
  void drawBoundary();


};

#endif
