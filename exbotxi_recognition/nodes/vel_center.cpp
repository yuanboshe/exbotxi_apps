#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <exbotxi_recognition/TwistWithMask.h>
#include <math.h>

#define BIAS 0.001;

// params
double paramMinLinear, paramMinAngular;
double paramMaxLinear, paramMaxAngular;
double paramAccelerateLinear, paramAccelerateAngular;

geometry_msgs::Twist goalVel;
geometry_msgs::Twist currentVel;

bool isDiff(geometry_msgs::Twist gVel, geometry_msgs::Twist cVel)
{
  float min = BIAS
  ;
  return fabs(gVel.linear.x - cVel.linear.x) > min || fabs(gVel.linear.y - cVel.linear.y) > min || fabs(gVel.linear.z - cVel.linear.z) > min
      || fabs(gVel.angular.x - cVel.angular.x) > min || fabs(gVel.angular.y - cVel.angular.y) > min || fabs(gVel.angular.z - cVel.angular.z) > min;
}

void verifyGoalLinear(double& gVel)
{
  if (fabs(gVel) < paramMinLinear)
    gVel = 0;
  else if (fabs(gVel) > paramMaxLinear)
    gVel = gVel > 0 ? paramMaxLinear : -paramMaxLinear;
}

void verifyGoalAngular(double& gVel)
{
  if (fabs(gVel) < paramMinAngular)
    gVel = 0;
  else if (fabs(gVel) > paramMaxAngular)
    gVel = gVel > 0 ? paramMaxAngular : -paramMaxAngular;
}

void verifyGoalTwist(geometry_msgs::Twist& gVel)
{
  verifyGoalLinear(gVel.linear.x);
  verifyGoalLinear(gVel.linear.y);
  verifyGoalLinear(gVel.linear.z);
  verifyGoalAngular(gVel.angular.x);
  verifyGoalAngular(gVel.angular.y);
  verifyGoalAngular(gVel.angular.z);
}

void smoothLinear(float gVel, double& cVel)
{
  if (cVel - gVel > paramAccelerateLinear)
    cVel -= paramAccelerateLinear;
  else if (gVel - cVel > paramAccelerateLinear)
    cVel += paramAccelerateLinear;
  else
    cVel = gVel;
}

void smoothAngular(float gVel, double& cVel)
{
  if (cVel - gVel > paramAccelerateAngular)
    cVel -= paramAccelerateAngular;
  else if (gVel - cVel > paramAccelerateAngular)
    cVel += paramAccelerateAngular;
  else
    cVel = gVel;
}

void smoothTwist(geometry_msgs::Twist gVel, geometry_msgs::Twist& cVel)
{
  smoothLinear(gVel.linear.x, cVel.linear.x);
  smoothLinear(gVel.linear.y, cVel.linear.y);
  smoothLinear(gVel.linear.z, cVel.linear.z);
  smoothAngular(gVel.angular.x, cVel.angular.x);
  smoothAngular(gVel.angular.y, cVel.angular.y);
  smoothAngular(gVel.angular.z, cVel.angular.z);
}

// global speed vars
void goalVelCallback(const geometry_msgs::TwistConstPtr& msg)
{
  goalVel = *msg;
  verifyGoalTwist(goalVel);
}

/**
 * mask == true means ignore the item
 */
void goalVelWithMaskCallback(const exbotxi_recognition::TwistWithMaskConstPtr& msg)
{
  double tmp = 0;
  if (!msg->mask[0])
  {
    tmp = msg->twist.linear.x;
    verifyGoalLinear(tmp);
    goalVel.linear.x = tmp;
  }
  if (!msg->mask[1])
  {
    tmp = msg->twist.linear.y;
    verifyGoalLinear(tmp);
    goalVel.linear.y = tmp;
  }
  if (!msg->mask[2])
  {
    tmp = msg->twist.linear.z;
    verifyGoalLinear(tmp);
    goalVel.linear.z = tmp;
  }
  if (!msg->mask[3])
  {
    tmp = msg->twist.angular.x;
    verifyGoalAngular(tmp);
    goalVel.angular.x = tmp;
  }
  if (!msg->mask[4])
  {
    tmp = msg->twist.angular.y;
    verifyGoalAngular(tmp);
    goalVel.angular.y = tmp;
  }
  if (!msg->mask[5])
  {
    tmp = msg->twist.angular.z;
    verifyGoalAngular(tmp);
    goalVel.angular.z = tmp;
  }
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "vel_center");

  // get params
  ros::NodeHandle ph("~");
  ph.param("minLinear", paramMinLinear, 0.03);
  ph.param("minAngular", paramMinAngular, 0.15);
  ph.param("maxLinear", paramMaxLinear, 0.5);
  ph.param("maxAngular", paramMaxAngular, 2.5);
  ph.param("accelerateLinear", paramAccelerateLinear, 0.25); // num / s
  ph.param("accelerateAngular", paramAccelerateAngular, 1.0); // num / s
  int rate;
  ph.param("rate", rate, 10);
  std::string vel_out;
  ph.param<std::string>("vel_out", vel_out, "vel_out");


  ros::NodeHandle nh;
  ros::Subscriber goalVelSub = nh.subscribe("/goal_vel", 1, goalVelCallback);
  ros::Subscriber goalVelWithMaskSub = nh.subscribe("/goal_vel_mask", 1, goalVelWithMaskCallback);
  ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>(vel_out, 1);

  // start pub loop
  ros::Rate loopRate(rate);
  paramAccelerateLinear /= rate; // accelerate limit per loop
  paramAccelerateAngular /= rate; // accelerate limit per loop
  while (ros::ok())
  {
    //if (isDiff(goalVel, currentVel))

    smoothTwist(goalVel, currentVel);
    // pub msg
    cmdVelPub.publish(currentVel);

//    ROS_INFO("goal: l[%.3f][%.3f][%.3f] a[%.3f][%.3f][%.3f]", goalVel.linear.x, goalVel.linear.y, goalVel.linear.z, goalVel.angular.x, goalVel.angular.y, goalVel.angular.z);
//    ROS_INFO("real: l[%.3f][%.3f][%.3f] a[%.3f][%.3f][%.3f]", currentVel.linear.x, currentVel.linear.y, currentVel.linear.z, currentVel.angular.x, currentVel.angular.y,
//             currentVel.angular.z);

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
