
#include <hull_cleaner/BoustrophedonPlanner.hh>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/LinearMath/Matrix3x3.h>



#define PRINT_FREQ 1

std::string nodeName("[Planner]");

using namespace angles;

BoustrophedonPlanner::BoustrophedonPlanner(double sim_speed) {
  simSpeed=sim_speed;
  filledPerc=robotRadius=timeLapsed=0;
  mode=fwd_long;
  startTime=ros::Time::now().toSec();
  map=NULL;
  robotInControlLoop=false;
  robotPose=geometry_msgs::Pose();
  //robotPose.position=geometry_msgs::Point();
  //robotPose.orientation=geometry_msgs::Quaternion();
}

void BoustrophedonPlanner::initializeMap() {
  double cellSize = robotRadius*2.0;
  int rows = (int)(MAP_LENGTH/cellSize);
  int cols = (int)(MAP_HEIGHT/cellSize);
  ROS_INFO("%20s Initializing map with %d rows, %d columns",nodeName.c_str(),rows,cols);
  map = new Map(rows,cols,cellSize);
}

double BoustrophedonPlanner::updateMap() {
  tf::Point vector1,vector2;
  int fillCount=0;
  for(unsigned int i=0; i<map->grid.size(); i++) {
    for(unsigned int j=0; j<map->grid[i].size(); j++) {
      tf::pointMsgToTF(map->grid[i][j].pos,vector1);
      tf::pointMsgToTF(robotPose.position,vector2);
      if(vector1.distance(vector2)<=robotRadius) map->grid[i][j].filled=true;
      if(map->grid[i][j].filled) fillCount++;
    }
  }
  return ((double)fillCount)/((double)map->grid.size()*(double)map->grid[0].size());
}

void BoustrophedonPlanner::calculateMode() {
  if(robotInControlLoop) return;
  double distToTop = fabs(robotPose.position.y-MAP_HEIGHT);
  double distToBottom = robotPose.position.y;
  double distToRight = fabs(robotPose.position.x-MAP_LENGTH);

  if(mode==stop) return;
  if(distToRight<=robotRadius && (distToTop<=robotRadius || distToBottom<=robotRadius)) {
    mode=stop;
    return;
  }
  double yaw = getYawFromQuat(robotPose.orientation);
  if(distToTop<=robotRadius && ((fabs(shortest_angular_distance(yaw,from_degrees(0)))<from_degrees(10) && mode==fwd_long) ||
      (fabs(shortest_angular_distance(yaw,from_degrees(90)))<from_degrees(10) && mode==fwd_short))) {
    mode=right_turn90;
  } else if(fabs(shortest_angular_distance(yaw,from_degrees(90)))<from_degrees(10) && ((distToTop<=robotRadius && mode==right_turn90) ||
      (distToBottom<=robotRadius && mode==left_turn90))) {
    mode=fwd_short;
  } else if(distToBottom<=robotRadius && ((fabs(shortest_angular_distance(yaw,from_degrees(180)))<from_degrees(10) && mode==fwd_long) ||
      (fabs(shortest_angular_distance(yaw,from_degrees(90)))<from_degrees(10) && mode==fwd_short))) {
    mode=left_turn90;
  } else mode=fwd_long;
}

void BoustrophedonPlanner::sendCommand(std::string cmd,double val) {
  std_msgs::String msg;
  std::stringstream ss;
  ss.precision(10);
  ss.str("");
  ss<<cmd<<" "<<val;
  msg.data = ss.str();
  pCommand.publish(msg);
}

void BoustrophedonPlanner::task() {
  timeLapsed=fabs(startTime-ros::Time::now().toSec());
  calculateMode();
  filledPerc=updateMap()*100;
  switch(mode) {
    case fwd_long:
      ROS_INFO_THROTTLE(PRINT_FREQ,"%20s movefwd long",nodeName.c_str());
      sendCommand("move_fwd",MAP_HEIGHT);
      break;
    case fwd_short:
      ROS_INFO_THROTTLE(PRINT_FREQ,"%20s movefwd short",nodeName.c_str());
      sendCommand("move_fwd",robotRadius*2.0);
      break;
    case left_turn90:
      ROS_INFO_THROTTLE(PRINT_FREQ,"%20s turn left 90",nodeName.c_str());
      sendCommand("turn",-angles::from_degrees(90.0));
      break;
    case right_turn90:
      ROS_INFO_THROTTLE(PRINT_FREQ,"%20s turn right 90",nodeName.c_str());
      sendCommand("turn",angles::from_degrees(90.0));
      break;
    case stop:
      ROS_INFO_THROTTLE(PRINT_FREQ,"%20s stop",nodeName.c_str());
      sendCommand("stop");
      break;
  }
}


void BoustrophedonPlanner::setRobotStatus(const std_msgs::String::ConstPtr& msg) {
  std::stringstream ss(msg->data);
  geometry_msgs::Pose tempPose;
  while(!ss.eof()) {
    ss>>robotRadius>>tempPose.position.x>>tempPose.position.y>>tempPose.position.z;
    ss>>tempPose.orientation.x>>tempPose.orientation.y>>tempPose.orientation.z;
    ss>>tempPose.orientation.w>>robotInControlLoop;
  }
  robotPose=tempPose;
  ROS_INFO_THROTTLE(PRINT_FREQ,"%20s InLoop:%d,Radius:%.2f Pose:x=%.2f,y=%.2f,yaw=%.2f",nodeName.c_str(),robotInControlLoop,
                    robotRadius,robotPose.position.x,robotPose.position.y,angles::to_degrees(getYawFromQuat(robotPose.orientation)));
}

void BoustrophedonPlanner::drawPercentage() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/planner_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "path";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;

  marker.scale.z = 2;
  geometry_msgs::Point pos;
  std_msgs::ColorRGBA color;
  std::stringstream text;
  marker.id=0;
  marker.pose.position.x=MAP_LENGTH/2.0;
  marker.pose.position.y=-2;//-10;
  marker.color.a = 1.0; marker.color.r=1.0f; marker.color.g=1.0f; marker.color.b=1.0f;
  text.str(std::string());
  text<<filledPerc<<"% in "<<timeLapsed*simSpeed*SEC_TO_HOUR<<" hours";
  marker.text = text.str();
  pPercMarker.publish(marker);
}

void BoustrophedonPlanner::drawBoundary() {
  geometry_msgs::PolygonStamped rect;
  rect.header.frame_id = "/planner_frame";
  rect.header.stamp = ros::Time::now();

  geometry_msgs::Point32 topLeft,topRight,bottomLeft,bottomRight;
  topLeft.x=0.0; topLeft.y=MAP_HEIGHT; topRight.x=MAP_LENGTH; topRight.y=MAP_HEIGHT;
  bottomLeft.x=0.0; bottomLeft.y=0.0; bottomRight.x=MAP_LENGTH; bottomRight.y=0.0;
  rect.polygon.points.push_back(topLeft);
  rect.polygon.points.push_back(topRight);
  rect.polygon.points.push_back(bottomRight);
  rect.polygon.points.push_back(bottomLeft);

  pBoundary.publish(rect);
}





//* Planner node
/**
 * \brief Main function for robot node
 *
 * Params:
 * 1. sim_speed - simulation speed multiplier
 * 2. (node_name)/freq - simulation speed
 *
 * Publishes:
 * 1. (node_name)/command - commands from the planner
 * 2. /perc_marker        - Percentage covered text for Rviz
 * 3. /boundary           - PolygonStamped Map boundaryfor Rviz
 *
 * Subscribes to:
 * 1. robot/status        - robot status
 *
 *
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "BoustrophedonPlanner");
  ros::NodeHandle n;
  ros::NodeHandle np("~");


  double freq;
  if(!(np.hasParam("freq"))) ROS_INFO_THROTTLE(PRINT_FREQ,"%20s 'freq' parameter not set. Defaulting to 5Hz",nodeName.c_str());
  np.param<double>("freq",freq, 5);
  ROS_INFO_THROTTLE(PRINT_FREQ,"%20s freq Param:%.2f",nodeName.c_str(),freq);

  double simSpeed;
  if(!(n.hasParam("sim_speed"))) ROS_INFO_THROTTLE(PRINT_FREQ,"%20s 'sim_speed' parameter not set. Defaulting to 1",nodeName.c_str());
  n.param<double>("sim_speed",simSpeed, 1);
  ROS_INFO_THROTTLE(PRINT_FREQ,"%20s SimSpeed Param:%.2f",nodeName.c_str(),simSpeed);

  BoustrophedonPlanner bp(simSpeed);

  //subscribers
  ros::Subscriber sub1 = n.subscribe("robot/status", 1000, &BoustrophedonPlanner::setRobotStatus, &bp);
  //publishers
  bp.pBoundary = n.advertise<geometry_msgs::PolygonStamped>("boundary", 1);
  bp.pPercMarker = n.advertise<visualization_msgs::Marker>("perc_marker", 1);
  bp.pCommand = np.advertise<std_msgs::String>("command", 1);

  ROS_INFO_THROTTLE(PRINT_FREQ,"%20s Final SimSpeed:%.2f",nodeName.c_str(),simSpeed*freq);
  ros::Rate loop_rate(freq*simSpeed);
  while (ros::ok()) {
    ros::spinOnce();
    if(bp.getRobotRadius()!=0) break;
    ROS_INFO_THROTTLE(PRINT_FREQ,"%20s Waiting for robot radius...",nodeName.c_str());
    loop_rate.sleep();
  }
  bp.initializeMap();
  while (ros::ok()) {
    ros::spinOnce();
    bp.task();
    bp.drawBoundary();
    bp.drawPercentage();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}
