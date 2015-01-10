
#include <hull_cleaner/BoustrophedonPlanner.hh>
#include <hull_cleaner/Map.hh>
#include <hull_cleaner/RobotUtil.hh>
#include <hull_cleaner/Robot.hh>

#include <gtest/gtest.h>

using namespace std;

class createQuatFromEulerTest : public testing::Test {
public:
  struct Euler {
    double roll,pitch,yaw;
  };
  vector<Euler> eulerSet;
  vector<geometry_msgs::Quaternion> quatSet;

  virtual void SetUp(){
    Euler euler;
    euler.roll=angles::from_degrees(88.0); euler.pitch=angles::from_degrees(88.0); euler.yaw=angles::from_degrees(88.0);
    eulerSet.push_back(euler);
    geometry_msgs::Quaternion quat;
    quat.x=0.0123331973097206; quat.y=0.70656840065847526; quat.z=0.0123331973097206; quat.w=0.70742977013917319;
    quatSet.push_back(quat);
  }
  virtual void TearDown(){}
};

TEST_F(createQuatFromEulerTest, testCreateQuat) {
  for(unsigned int i=0; i<eulerSet.size(); i++) {
    geometry_msgs::Quaternion quat=getQuatFromEuler(eulerSet[i].roll,eulerSet[i].pitch,eulerSet[i].yaw);
    EXPECT_DOUBLE_EQ(quat.x,quatSet[i].x);
    EXPECT_DOUBLE_EQ(quat.y,quatSet[i].y);
    EXPECT_DOUBLE_EQ(quat.z,quatSet[i].z);
    EXPECT_DOUBLE_EQ(quat.w,quatSet[i].w);
  }
}

class QuatToYawTest : public testing::Test {
public:
  vector<geometry_msgs::Quaternion> quats;
  vector<double> yawSet;

  virtual void SetUp(){
    yawSet.push_back(angles::from_degrees(88.0)); yawSet.push_back(angles::from_degrees(171.0));
    yawSet.push_back(angles::from_degrees(-82.0)); yawSet.push_back(angles::from_degrees(-156.0));
    for(unsigned int i=0; i<yawSet.size(); i++) {
      quats.push_back(getQuatFromEuler(0.0,0.0,yawSet[i]));
    }
  }
  virtual void TearDown(){}
};

TEST_F(QuatToYawTest, testYaw) {
  for(unsigned int i=0; i<quats.size(); i++) {
    EXPECT_DOUBLE_EQ(getYawFromQuat(quats[i]),yawSet[i]);
  }
}

class RotateAboutZTest : public testing::Test {
public:
  geometry_msgs::Quaternion quat;
  double yaw;
  vector<double> yawChangeSet;

  virtual void SetUp(){
    yaw=angles::from_degrees(88.0);
    quat=getQuatFromEuler(0.0,0.0,yaw);
    yawChangeSet.push_back(angles::from_degrees(5.0)); yawChangeSet.push_back(angles::from_degrees(25.0));
    yawChangeSet.push_back(angles::from_degrees(250.0)); yawChangeSet.push_back(angles::from_degrees(-280.0));
  }
  virtual void TearDown(){}
};

TEST_F(RotateAboutZTest, testRotate) {
  geometry_msgs::Quaternion tempQuat;
  double expectedYaw,calculatedYaw;
  for(unsigned int i=0; i<yawChangeSet.size(); i++) {
    tempQuat=quat;
    expectedYaw=angles::normalize_angle(getYawFromQuat(quat)+yawChangeSet[i]);
    rotateAboutZAxis(tempQuat,getYawFromQuat(quat)+yawChangeSet[i]);
    calculatedYaw=angles::normalize_angle(getYawFromQuat(tempQuat));
    EXPECT_NEAR(calculatedYaw,expectedYaw,0.0000001);
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}






