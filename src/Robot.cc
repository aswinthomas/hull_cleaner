
#include <hull_cleaner/Robot.hh>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define PRINT_FREQ 1.0
std::string nodeName("[Robot]");

Robot::Robot(double simSpeed) {
  pose=geometry_msgs::Pose();
  pose.position.x=RADIUS;
  //pose.position.y=pose.position.z=0.0;
  //pose.orientation=getQuatFromEuler(0.0,0.0,0.0);
  ROS_INFO("%20s Starting Pos x:%.2f y:.2%f z:.2%f yaw:.2%f",nodeName.c_str(),
           pose.position.x,pose.position.y,pose.position.z,
           angles::to_degrees(getYawFromQuat(pose.orientation)));
  positionTarget=pose.position;
  yawTarget=0.0;

  inControlLoop=false;
  freq=0;
  mode=stop;
  radius=RADIUS;
  maxFwdVel=MAX_FWD_VEL*simSpeed;
  maxAngVel=MAX_ANG_RATE*simSpeed;
}

void Robot::forwardControl() {
  tf::Point pos,target;
  tf::pointMsgToTF(pose.position,pos);
  tf::pointMsgToTF(positionTarget,target);
  double distToGoal=pos.distance(target);
  ROS_INFO_THROTTLE(PRINT_FREQ,"%20s DistToGoal:%.2f",nodeName.c_str(),distToGoal);
  if(distToGoal<=DIST_TOLERANCE) { inControlLoop=false; return;}
  inControlLoop=true;
  double prefVel=kp_POS*distToGoal;
  double maxVel=maxFwdVel*(1/freq);
  double yaw = getYawFromQuat(pose.orientation);
  if(prefVel>maxVel) prefVel=maxVel; if(prefVel<-maxVel) prefVel=-maxVel;
  pose.position.x+=prefVel*sinf(yaw);
  pose.position.y+=prefVel*cosf(yaw);
}

void Robot::turningControl() {
  double yaw = getYawFromQuat(pose.orientation);
  double angDist = fabs(angles::shortest_angular_distance(yaw,yawTarget));
  ROS_INFO_THROTTLE(PRINT_FREQ,"AngDist:%.2f",angDist);
  if(angDist<angles::from_degrees(ANG_TOLERANCE)) {
    inControlLoop=false; return;
  }
  inControlLoop=true;
  double prefAngRateDiscrete=angles::from_degrees(Kp_ANG)*angles::shortest_angular_distance(yaw,yawTarget);
  double maxAngRateDiscrete = angles::from_degrees(maxAngVel)*(1/freq);
  if(prefAngRateDiscrete>maxAngRateDiscrete) prefAngRateDiscrete=maxAngRateDiscrete;
  else if(prefAngRateDiscrete<-maxAngRateDiscrete) prefAngRateDiscrete=-maxAngRateDiscrete;
  rotateAboutZAxis(pose.orientation,yaw+prefAngRateDiscrete);
  ROS_INFO_THROTTLE(PRINT_FREQ,"yaw:%f",angles::to_degrees(getYawFromQuat(pose.orientation)));
}

void Robot::task() {
  posHistory.push_back(pose.position);
  switch(mode) {
    case move_fwd:
      ROS_INFO_THROTTLE(PRINT_FREQ,"%20s move",nodeName.c_str());
      forwardControl();
      break;
    case turn:
      ROS_INFO_THROTTLE(PRINT_FREQ,"%20s turn",nodeName.c_str());
      turningControl();
      break;
    case stop:
      ROS_INFO_THROTTLE(PRINT_FREQ,"%20s stop",nodeName.c_str());
      inControlLoop=false;
      break;
  }
}

void Robot::setCommand(const std_msgs::String::ConstPtr& msg) {
  std::stringstream ss(msg->data);
  std::string cmd;
  double value;
  double yaw = getYawFromQuat(pose.orientation);
  ss>>cmd>>value;

  if(mode!=move_fwd) {
    positionTarget.x=pose.position.x+value*sinf(yaw);
    positionTarget.y=pose.position.y+value*cosf(yaw);
  }
  if(mode!=turn) yawTarget=angles::normalize_angle(yaw+value);

  if((cmd.compare("move_fwd"))==0) mode=move_fwd;
  else if((cmd.compare("stop"))==0) mode=stop;
  else if((cmd.compare("turn"))==0) mode=turn;

  ROS_INFO_THROTTLE(PRINT_FREQ,"%20s Cmd:%s %f",nodeName.c_str(),cmd.c_str(),value);
}

void Robot::publishMsgs() {
  std_msgs::String msg;
  std::stringstream ss;
  ss.precision(10);
  ss<<radius<<" "<<pose.position.x<<" "<<pose.position.y<<" "<<pose.position.z<<" ";
  ss<<pose.orientation.x<<" "<<pose.orientation.y<<" "<<pose.orientation.z<<" "<<pose.orientation.w<<" ";
  ss<<inControlLoop;
  msg.data = ss.str();
  pStatus.publish(msg);
}

void Robot::drawRobot() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/planner_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "path";
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.id = 0;
  marker.scale.x = radius*2;
  marker.scale.y = radius*2;
  marker.scale.z = radius*2;
  std_msgs::ColorRGBA color;
  for(unsigned int i=0; i<posHistory.size(); i++) {
    marker.points.push_back(posHistory[i]);
    color.a=0.6;
    if(i==(posHistory.size()-1)) { color.r=1.0f; color.g=0.0f; color.b=0.0f;}
    else { color.r=1.0f; color.g=1.0f; color.b=0.0f;}
    marker.colors.push_back(color);
  }
  pRobotMarker.publish(marker);
}

void Robot::drawDirection() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/planner_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "path";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.id = 0;
  // Set the scale of the marker
  marker.scale.x = 0.1;
  geometry_msgs::Point pos;
  std_msgs::ColorRGBA color;
  pos.x=pose.position.x; pos.y=pose.position.y;
  marker.points.push_back(pose.position); //start point
  double yaw = getYawFromQuat(pose.orientation);
  pos.x+=radius*2*sin(yaw); pos.y+=radius*2*cos(yaw);
  marker.points.push_back(pos); //end point
  color.a = 1.0; color.r=1.0f; color.g=1.0f; color.b=1.0f;
  marker.colors.push_back(color);
  marker.colors.push_back(color);
  pDirMarker.publish(marker);
}





//* Robot node
/**
 * \brief Main function for robot node
 *
 * Params:
 * 1. sim_speed - simulation speed multiplier
 * 2. (node_name)/freq - simulation speed
 *
 * Publishes:
 * 1. (node_name)/status - robot pose and control loop status
 * 2. /robot_marker      - MarkerArray robot position history for Rviz
 * 3. /dir_marker        - MarkerArray robot yaw for Rviz
 *
 * Subscribes to:
 * 1. planner/command    - commands from path planner
 *
 *
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "robot");
  ros::NodeHandle n;
  ros::NodeHandle np("~");

  double simSpeed,rate;
  if(!(n.hasParam("sim_speed"))) ROS_INFO_THROTTLE(PRINT_FREQ,"%20s 'sim_speed' parameter not set. Defaulting to 1",nodeName.c_str());
  n.param<double>("sim_speed",simSpeed, 1);
  if(!(np.hasParam("freq"))) ROS_INFO_THROTTLE(PRINT_FREQ,"%20s 'freq' parameter not set. Defaulting to 200Hz",nodeName.c_str());
  np.param<double>("freq",rate, 100);
  ROS_INFO("%20s SimSpeed Param:%.2f freq Param:%.2f Final freq:%.2f",nodeName.c_str(),simSpeed,rate,rate*simSpeed);

  Robot rbt(simSpeed);
  rbt.setFreq(rate);

  //subscribers
  ros::Subscriber sub1 = n.subscribe("planner/command", 1000, &Robot::setCommand, &rbt);
  //publishers
  rbt.pRobotMarker = n.advertise<visualization_msgs::Marker>("robot_marker", 1);
  rbt.pDirMarker = n.advertise<visualization_msgs::Marker>("dir_marker", 1);
  rbt.pStatus = np.advertise<std_msgs::String>("status", 1);

  ros::Rate loop_rate(rate*simSpeed);
  while (ros::ok()) {
    ros::spinOnce();
    rbt.task();
    rbt.publishMsgs();
    rbt.drawRobot();
    rbt.drawDirection();
    loop_rate.sleep();
  }
  ros::spin();

  return 0;
}










