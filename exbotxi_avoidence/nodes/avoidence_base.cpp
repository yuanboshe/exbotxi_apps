#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"

ros::Publisher pubvel;

// Timer
double startCommand;
int direction = 0; // count last direction + left, - right

// ROS Param Variables
std::vector<double> limit_ranges(2, 0);
std::vector<int> limit_points(2, 0);
double angle_speed, min_angle_speed;

std::string sensor_2d = "";

geometry_msgs::Twist velinTwist;
std::vector<std::vector<double> > regionRates;
std::vector<std::vector<int> > regionCounts(2);
std::vector<std::vector<double> > regionAvRanges(2);

void velInCallback(const geometry_msgs::TwistConstPtr& msg)
{
  if (regionRates.size() == 0)
    return;

  geometry_msgs::Twist tw;
  tw.linear.x = msg->linear.x;
  tw.angular.z = msg->angular.z;

  if (msg->linear.x > 0)
  {
    if (regionCounts[0][1] > limit_points[0] && regionCounts[0][2] > limit_points[0])
    {
      tw.linear.x = 0;
      tw.angular.z = 0;
      ROS_INFO("[1] regionCounts[0][1] = %d, regionCounts[0][2] = %d", regionCounts[0][1], regionCounts[0][2]);
    }
    else if (regionCounts[0][1] > limit_points[0])
    {
      tw.angular.z = angle_speed;
      ROS_INFO("[2] regionCounts[0][1] = %d", regionCounts[0][1]);
    }
    else if (regionCounts[0][2] > limit_points[0])
    {
      tw.angular.z = -angle_speed;
      ROS_INFO("[3] regionCounts[0][2] = %d", regionCounts[0][2]);
    }
    else if (regionCounts[1][1] > limit_points[1])
    {
      tw.angular.z = angle_speed;
      ROS_INFO("[4] regionCounts[1][1] = %d", regionCounts[1][1]);
    }
    else if (regionCounts[1][2] > limit_points[1])
    {
      tw.angular.z = -angle_speed;
      ROS_INFO("[5] regionCounts[1][2] = %d", regionCounts[1][2]);
    }
    startCommand = ros::Time::now().toSec();
  }
  else if (abs(msg->angular.z) > min_angle_speed)
  {
    if (msg->angular.z > 0 && regionCounts[0][3] > limit_points[0])
    {
      tw.angular.z = 0;
    }
    else if (msg->angular.z < 0 && regionCounts[0][0] > limit_points[0])
    {
      tw.angular.z = 0;
    }
  }

  pubvel.publish(tw);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  const double PI = 3.1415;

  if (regionRates.size() == 0)
  {
    if (sensor_2d == "rplidar" && msg->angle_min >= 0)
    {
      double angles[] = {5.673, 4.533, 1.57, 0.61}; // degrees right 35 90 180 90 35 left
      double startAngle = msg->angle_max;
      double rangeLen = msg->angle_max - msg->angle_min;
      int loopi = sizeof(angles) / sizeof(double);
      for (int i = 0; i < loopi; i++)
      {
        angles[i] = (startAngle - angles[i]) / rangeLen;
      }
      std::vector<double> regionRateR1(2);
      regionRateR1[0] = 0;
      regionRateR1[1] = angles[0];
      std::vector<double> regionRateR2(2);
      regionRateR2[0] = angles[0];
      regionRateR2[1] = angles[1];
      std::vector<double> regionRateL2(2);
      regionRateL2[0] = angles[2];
      regionRateL2[1] = angles[3];
      std::vector<double> regionRateL1(2);
      regionRateL1[0] = angles[3];
      regionRateL1[1] = 1;
      regionRates.push_back(regionRateR2);
      regionRates.push_back(regionRateR1);
      regionRates.push_back(regionRateL1);
      regionRates.push_back(regionRateL2);
      ROS_INFO("It's rplidar");
    }
    else
    {
      double angles[] = {-1.57, -0.61, 0, 0.61, 1.57}; // degrees right 90 35 0 35 90 left
      double startAngle = msg->angle_min;
      double rangeLen = msg->angle_max - msg->angle_min;
      int loopi = sizeof(angles) / sizeof(double);
      for (int i = 0; i < loopi; i++)
      {
        angles[i] = (angles[i] - startAngle) / rangeLen;
      }
      std::vector<double> regionRateR2(2);
      regionRateR2[0] = angles[0];
      regionRateR2[1] = angles[1];
      std::vector<double> regionRateR1(2);
      regionRateR1[0] = angles[1];
      regionRateR1[1] = angles[2];
      std::vector<double> regionRateL1(2);
      regionRateL1[0] = angles[2];
      regionRateL1[1] = angles[3];
      std::vector<double> regionRateL2(2);
      regionRateL2[0] = angles[3];
      regionRateL2[1] = angles[4];
      regionRates.push_back(regionRateR2);
      regionRates.push_back(regionRateR1);
      regionRates.push_back(regionRateL1);
      regionRates.push_back(regionRateL2);
      ROS_INFO("It's hokuyo");
    }
  }

  int beamsNum = msg->ranges.size();
  std::vector<std::vector<int> > regions;
  int loopi = regionRates.size();
  for (int i = 0; i < loopi; i++)
  {
    std::vector<int> region(2);
    region[0] = regionRates[i][0] * beamsNum;
    region[1] = regionRates[i][1] * beamsNum;
    regions.push_back(region);

  }

//  //view test info
//  ROS_INFO("R2: %d~%d, R1: %d~%d, L1: %d~%d, L2: %d~%d", regions[0][0], regions[0][1], regions[1][0], regions[1][1], regions[2][0], regions[2][1], regions[3][0], regions[3][1]);

  regionCounts[0] = std::vector<int>(regions.size(), 0);
  regionCounts[1] = std::vector<int>(regions.size(), 0);
  regionAvRanges[0] = std::vector<double>(regions.size(), 0);
  regionAvRanges[1] = std::vector<double>(regions.size(), 0);
  for (int i = 0; i < beamsNum; i++)
  {
    if (msg->ranges[i] < msg->range_min || msg->ranges[i] > limit_ranges[1])
      continue;

    for (int j = 0, loop = regions.size(); j < loop; j++)
    {
      if (i >= regions[j][0] && i <= regions[j][1])
      {
        regionCounts[1][j]++;
        regionAvRanges[1][j] += msg->ranges[i];

        if (msg->ranges[i] < limit_ranges[0])
        {
          regionCounts[0][j]++;
          regionAvRanges[0][j] += msg->ranges[i];
        }
      }
    }
  }

  for (int i = 0, loop = regions.size(); i < loop; i++)
  {
    regionAvRanges[0][i] = regionAvRanges[0][i] / regionCounts[0][i];
    regionAvRanges[1][i] = regionAvRanges[1][i] / regionCounts[1][i];
  }

//  // view test info
//  ROS_INFO("regions: %d - %f, %d - %f, %d - %f, %d - %f", regionCounts[0], regionAvRanges[0], regionCounts[1], regionAvRanges[1], regionCounts[2], regionAvRanges[2], regionCounts[3], regionAvRanges[3]);

  //ROS_INFO("%f, %d", limit_max_range, limit_point_count);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "avoidence_base");

// Get ROS Params
  ros::NodeHandle ph("~");
  ros::NodeHandle nh;
  std::string scan_in, vel_in, vel_out;
  ph.param<std::string>("scan_in", scan_in, "scan");
  ph.param<std::string>("vel_in", vel_in, "vel_in");
  ph.param<std::string>("vel_out", vel_out, "vel_out");
  ph.param<std::string>("sensor_2d", sensor_2d, "rplidar");
  ph.param<double>("limit_range1", limit_ranges[0], 0.3);
  ph.param<double>("limit_range2", limit_ranges[1], 0.8);
  ph.param<int>("limit_points1", limit_points[0], 10);
  ph.param<int>("limit_points2", limit_points[1], 15);
  ph.param<double>("angle_speed", angle_speed, 0.6);
  ph.param<double>("min_angle_speed", min_angle_speed, 0.02);

  //vel_out = "/cmd_vel";
  ros::Subscriber subScan = nh.subscribe<sensor_msgs::LaserScan>(scan_in, 1, scanCallback);
  ros::Subscriber subVel = nh.subscribe<geometry_msgs::Twist>(vel_in, 1, velInCallback);
  pubvel = nh.advertise<geometry_msgs::Twist>(vel_out, 1);

  startCommand = ros::Time::now().toSec();

  ros::spin();

  return (0);
}

