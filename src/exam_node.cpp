/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

float forward_th = 1.4;     // 前方障碍物小于这个距离时启动VFH避障
float object_th = 2;            // 障碍物检测距离

#define STATE_READY              0
#define STATE_FORWARD             1
#define STATE_GEN_TEMP_TARGET    2
#define STATE_GOTO_TEMP_TARGET   3
#define STATE_ARRIVED_TEMP_TARGET       4
int state = STATE_READY;

float value_ranges[360];
ros::Publisher vel_pub;
tf::TransformListener* tf_listener;
int nFrontIndex = 180;
int flag_turn = -1;

// 生成临时目标点
void GenTempTarget(float* inRanges,  float& outLinear, float& outAngular)
{
    printf("正在生成临时目标点...\n");
    // 通过里程计tf获取机器人坐标
    tf::StampedTransform transform;
    try{
        tf_listener->lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    float robot_yaw = yaw;
    
    // 通过里程计tf获取目标点在机器人坐标系里的偏差值
    try{
        tf_listener->lookupTransform("/base_footprint", "/goal", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    tf::Quaternion q_2(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    tf::Matrix3x3(q_2).getRPY(roll, pitch, yaw);
    float target_yaw = yaw;

    outLinear = 0.0;
    outAngular = 0.3;
    if(fabs(robot_yaw - target_yaw) < 0.1)
    {
         int front_offset = 5;
        float front_range = value_ranges[nFrontIndex- front_offset];
        for(int i=nFrontIndex- front_offset ; i < nFrontIndex+ front_offset; i++)
        {
            if(value_ranges[i] < front_range)
                front_range = value_ranges[i];
        }
    }
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int nNum = scan->ranges.size();
    // ROS_WARN("[lidarCallback] nNum = %d",nNum);
    nFrontIndex = nNum/2;
    for(int i=0;i<360;i++)
    {
        value_ranges[i] = scan->ranges[i];
    }
    
    if(state == STATE_READY)
    {
        state = STATE_FORWARD;
        ROS_WARN("state ->  STATE_FORWARD");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"exam_node");
    ROS_INFO("exam_node start!");

    tf_listener = new tf::TransformListener;

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);

    float temp_target_theta = 0;
    float temp_target_R = 0;
    float temp_target_x = 0;
    float temp_target_y = 0;

    ros::Rate loop_rate(30);
    while( ros::ok())
    {
        if(state == STATE_FORWARD)
        {
            geometry_msgs::Twist vel_cmd;
            vel_cmd.angular.z = 0;
            vel_cmd.linear.x = 0.1;
            int front_offset = 5;
            float front_range = value_ranges[nFrontIndex- front_offset];
            for(int i=nFrontIndex- front_offset ; i < nFrontIndex+ front_offset; i++)
            {
                if(value_ranges[i] < front_range)
                    front_range = value_ranges[i];
            }
            printf("前方障碍物距离= %.2f 米\n",front_range);
            if(front_range < forward_th)
            {
                // 前方障碍物足够近了，启动避障
                vel_cmd.linear.x = 0.0;
                state = STATE_GEN_TEMP_TARGET;
                ROS_WARN("state ->  STATE_GEN_TEMP_TARGET");
            } 
            vel_pub.publish(vel_cmd);
        }

        if(state == STATE_GEN_TEMP_TARGET)
        {
            float linear,angular;
            GenTempTarget(value_ranges , linear , angular);

            // 通过里程计tf获取机器人当前的朝向角度
            tf::StampedTransform transform;
            try{
                tf_listener->lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.5).sleep();
            }
            tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            printf("机器人朝向角度=  %.2f \n",yaw);
            if(yaw > 0.8)
                flag_turn = -1;
            if(yaw < -0.8)
                flag_turn = 1;

            geometry_msgs::Twist vel_cmd;
            vel_cmd.linear.x = linear;
            vel_cmd.angular.z = angular * flag_turn;
            printf("vel_cmd.angular.z =  %.2f \n",vel_cmd.angular.z);
            vel_pub.publish(vel_cmd);
            if(linear > 0)
            {
                state = STATE_GOTO_TEMP_TARGET;
                ROS_WARN("state ->  STATE_GOTO_TEMP_TARGET");
            }
        }

        if(state == STATE_GOTO_TEMP_TARGET)
        {
            // 检测临时坐标点相对于机器人当前坐标系的位置
            float robot_target_x = 0;
            float robot_target_y =0;

            // 根据临时目标点计算自己的移动速度
            geometry_msgs::Twist vel_cmd;
            float robot_target_theta = atan2(robot_target_y, robot_target_x);
            printf("临时目标点相对于机器人朝向角度=  %.2f \n",robot_target_theta);
            vel_cmd.angular.z = 0.5 * robot_target_theta ;
            if(fabs(robot_target_theta) < 0.1)
                vel_cmd.linear.x = 0.2;
            else
                vel_cmd.linear.x = 0.0;

            vel_pub.publish(vel_cmd);
        }

        if(state == STATE_ARRIVED_TEMP_TARGET)
        {
            // 通过里程计tf获取机器人当前的朝向角度
            tf::StampedTransform transform;
            try{
                tf_listener->lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.5).sleep();
            }
            tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
            double roll, pitch, yaw;
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // 将机器人朝向角度调节到0度
            geometry_msgs::Twist vel_cmd;
            vel_cmd.linear.x = 0;
            vel_cmd.angular.z = -0.5*yaw;
            vel_pub.publish(vel_cmd);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}