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
#include <vfh_helper.h>

float forward_th = 1.4;  // 前方障碍物小于这个距离时启动VFH避障
float d_safe = 0.45;         // 可通行宽度的安全阈值
float object_th = 2;     // 障碍物检测距离

#define STATE_READY              0
#define STATE_FORWARD             1
#define STATE_GEN_TEMP_TARGET    2
#define STATE_GOTO_TEMP_TARGET   3
#define STATE_ARRIVED_TEMP_TARGET       4
int state = STATE_READY;

float value_ranges[360];
stGap gap[180];
ros::Publisher vel_pub;
tf::TransformListener* tf_listener;
int nFrontIndex = 180;

// VFH生成临时目标点
void GenTempTarget(float* inRanges,  float& outTargetAngle, float& outTargetR)
{
    // 设置障碍物
    int object[180];
    for(int i=0;i<180;i++)
    {
        if(inRanges[i+90] < object_th)
            object[i] = 1;
        else
            object[i] = 0;
    }

    // 检测障碍物
    int object_n = 0;
    int object_begin[180];
    int object_length[180];
    bool object_detected = false;
    for(int i=0;i<(180-2);i++)
    {
        if(object_detected == false)
        {
            // 检测到连续 3 个 1 或者 1 0 1 计为障碍物起点
            if( (object[i]==1 && object[i+1]==1 && object[i+2]==1 ) || (object[i]==1 && object[i+1]==0 && object[i+2]==1 ) )
            {
                object_detected = true;
                object_begin[object_n] = i;
                i+=2;
            }
        }
        else
        {
            // 检测到 000 记为障碍物终点
            if(object[i]==0 && object[i+1]==0 && object[i+2]==0 ) 
            {
                object_detected = false;
                object_length[object_n] = i -  object_begin[object_n] ;
                object_n ++;
            }
        }
    }
    // 如果在检测障碍物的尾部前中断，主动结束当前障碍物检测
    if(object_detected == true)
    {
        object_detected = false;
        object_length[object_n] = 180 -  object_begin[object_n] ;
        object_n ++;
    }
    printf("-----------------------------\n");
    printf("检测到 %d 个障碍物\n",object_n);
    for(int i=0;i<object_n;i++)
    {
        printf("障碍物[%d] ：起始于 %d  宽度 %d \n",i,object_begin[i] ,object_length[i]);
    }

    // 统计相邻障碍物之间的缝隙
    int gap_n = 0;
    if(object_n > 0)
    {
        if(object_begin[0] > 0 )
        {
            // 障碍物从非0号射线开始，将障碍物之前的缝隙加进去
            gap[0].angle_begin = 0;
            gap[0].r_begin = value_ranges[90+object_begin[0]];
            gap[0].angle_end = object_begin[0]-1;
            gap[0].r_end = value_ranges[90+object_begin[0]];
            gap_n ++;
        }
        // 接下来的缝隙从第一个障碍物后面开始找
        gap[gap_n].angle_begin = object_begin[0] + object_length[0];
        gap[gap_n].r_begin = value_ranges[90+object_begin[0] + object_length[0]-1];
        
        // 看看是否还存在第二个障碍物，有就找下一个障碍物的开始角度
        if(object_n > 1)
        {
            for(int i=1 ; i< object_n; i++)
            {
                gap[gap_n].angle_end = object_begin[i];   // 缝隙到下一个障碍物开始结束
                gap[gap_n].r_end = value_ranges[90+object_begin[i]]; 
                gap_n ++;

                // 如果当前障碍物收尾不在180度，说明后面还有缝隙，开始一个新的缝隙
                if(object_begin[i] + object_length[i] < 180-2)
                {
                    gap[gap_n].angle_begin = object_begin[i] + object_length[i];
                    gap[gap_n].r_begin = value_ranges[90+object_begin[i] + object_length[i]-1];
                }
            }
            // 看看最后一个障碍物结束时是不是180度位置，如果不是，需要将剩余的扫描角度作为一个缝隙
            if(object_begin[object_n -1] + object_length[object_n -1] < (180-2))
            {
                gap[gap_n].angle_begin = object_begin[object_n -1] + object_length[object_n -1];
                gap[gap_n].r_begin = value_ranges[90+object_begin[object_n -1] + object_length[object_n -1]-1];
                gap[gap_n].angle_end = 180;
                gap[gap_n].r_end = gap[gap_n].r_begin;
                gap_n ++;
            }
            else
            {
                // 收尾
                gap[gap_n].angle_end = 180;
                gap[gap_n].r_end = gap[gap_n].r_begin;
                gap_n ++;
            }
        }
    }
    else
    {
        // 没有障碍物，可以直接向目标角度移动
    }

    // 计算每个缝隙的宽度d
    for(int i=0;i<gap_n;i++)
    {
        float r_min = (gap[i].r_begin < gap[i].r_end)? gap[i].r_begin : gap[i].r_end;
        float theta = (gap[i].angle_end - gap[i].angle_begin)*M_PI/180;
        gap[i].d =  r_min * theta;
    }
    printf("检测到 %d 个缝隙\n",gap_n);
    for(int i=0;i<gap_n;i++)
    {
        printf("缝隙[%d] ：起始于角度 %.0f r=%.2f  ，结束于角度 %.0f  r=%.2f   d= %.2f \n",i,gap[i].angle_begin ,gap[i].r_begin,gap[i].angle_end ,gap[i].r_end, gap[i].d);
    }

    // 计算每个缝隙的可通行路线
    for(int i=0;i<gap_n;i++)
    {
        // 若 d < df ,不可通行
        if(gap[i].d < d_safe)
            gap[i].path_num = 0;
        // 若 df < d < 2 df ,只有一个通行方向
        if(gap[i].d > d_safe && gap[i].d < d_safe*2)
        {
            gap[i].path_num = 1;
            gap[i].path_theta_1 = (M_PI/180) * ((gap[i].angle_begin + gap[i].angle_end)/2);
        }
        // 若 d > 2 df ,则有两个通行方向
        if(gap[i].d > d_safe*2)
        {
            gap[i].path_num = 2;
            gap[i].path_theta_1 = (M_PI/180) * gap[i].angle_end - asin(d_safe/gap[i].angle_end);
            gap[i].path_theta_2 = (M_PI/180) * gap[i].angle_begin + asin(d_safe/gap[i].r_begin);
        }
    }
    printf("---可通行缝隙---\n");
    for(int i=0;i<gap_n;i++)
    {
        if(gap[i].path_num == 1)
            printf("缝隙[%d] 可通行角度= %.0f \n",i,gap[i].path_theta_1 *180/M_PI);
        if(gap[i].path_num == 2)
            printf("缝隙[%d] 可通行角度= %.0f  ,  %.0f\n",i,gap[i].path_theta_1 *180/M_PI,gap[i].path_theta_2 *180/M_PI);
    }

    // 通过里程计tf获取目标点在机器人坐标系里的偏差值
    tf::StampedTransform transform;
    try{
        tf_listener->lookupTransform("/base_footprint", "/goal", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    printf("最终目标点相对机器人坐标 ( %.2f , %.2f )\n",transform.getOrigin().x(),transform.getOrigin().y());
    float goal_angle = CalTargetAngle(transform.getOrigin().x(),transform.getOrigin().y());
    printf("最终目标点对应的激光射线角度 =  %.0f  \n",goal_angle *180/M_PI);

    // 从可通行方向里寻找最靠近目标点的
    float temp_path_theta = 90;
    float temp_path_R = 0;
    float angle_diff_min = 9999;
    for(int i=0;i<gap_n;i++)
    {
        if(gap[i].path_num == 1)
        {
            float diff = fabs(gap[i].path_theta_1 - goal_angle);
            if(diff < angle_diff_min)
            {
                temp_path_theta = gap[i].path_theta_1;
                angle_diff_min = diff;
                temp_path_R = (((gap[i].path_theta_1 - goal_angle) > 0)?gap[i].r_begin:gap[i].r_end) + d_safe;                
            }
        }
        if(gap[i].path_num == 2)
        {
            float diff = fabs(gap[i].path_theta_1 - goal_angle);
            if(diff < angle_diff_min)
            {
                temp_path_theta = gap[i].path_theta_1;
                angle_diff_min = diff;
                temp_path_R = gap[i].r_end + d_safe;
                // temp_path_R = ((gap[i].r_begin > gap[i].r_end)?gap[i].r_begin:gap[i].r_end) + d_safe;
            }
            diff = fabs(gap[i].path_theta_2 - goal_angle);
            if(diff < angle_diff_min)
            {
                temp_path_theta = gap[i].path_theta_2;
                angle_diff_min = diff;
                temp_path_R = gap[i].r_begin + d_safe;
                // temp_path_R = ((gap[i].r_begin > gap[i].r_end)?gap[i].r_begin:gap[i].r_end) + d_safe;
            }
        }
    }
    printf("机器人选择移动方向角度 =  %.2f   探测步长R = %.2f\n",temp_path_theta *180/M_PI,temp_path_R);
    outTargetAngle = temp_path_theta;
    outTargetR = temp_path_R + d_safe;
}

// 获取里程计坐标系里某一点相对于机器人坐标系的位置
void CetTargetPosition(float inOdomX, float inOdomY,  float& outX, float& outY)
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = inOdomX;
    pose.pose.position.y = inOdomY;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    pose.header.frame_id = "odom";
    pose.header.stamp = ros::Time(0);

    geometry_msgs::PoseStamped tf_pose;
    tf_listener->transformPose("base_footprint", pose, tf_pose);
    outX = tf_pose.pose.position.x;
    outY = tf_pose.pose.position.y;
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
    ros::init(argc,argv,"vfh_node");
    ROS_INFO("vfh_node start!");

    InitHelper();
    SetObjectTh(object_th);
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
                // 前方障碍物足够近了，启动VFH避障
                vel_cmd.linear.x = 0.0;
                state = STATE_GEN_TEMP_TARGET;
                ROS_WARN("state ->  STATE_GEN_TEMP_TARGET");
            } 
            vel_pub.publish(vel_cmd);
        }

        if(state == STATE_GEN_TEMP_TARGET)
        {
            // 将VFH和阈值线显示在窗口中
            SetRanges(value_ranges );
            GenTempTarget(value_ranges , temp_target_theta , temp_target_R);
            printf("临时目标点角度 =  %.2f   探测步长R = %.2f\n",temp_target_theta,temp_target_R);
            // 获取机器人此时的里程计坐标
            tf::StampedTransform transform;
            try{
                tf_listener->lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
            float odom_robot_x = transform.getOrigin().x();
            float odom_robot_y = transform.getOrigin().y();
            temp_target_x = temp_target_R*sin(temp_target_theta) + odom_robot_x;
            temp_target_y = -temp_target_R*cos(temp_target_theta) + odom_robot_y;
            printf("机器人里程计坐标 ( %.2f , %.2f ) - 临时目标点坐标  ( %.2f , %.2f )\n",odom_robot_x,odom_robot_y,temp_target_x,temp_target_y);
            state = STATE_GOTO_TEMP_TARGET;
            ROS_WARN("state ->  STATE_GOTO_TEMP_TARGET");
        }

        if(state == STATE_GOTO_TEMP_TARGET)
        {
            //SetRanges(value_ranges );
            //GenTempTarget(value_ranges , temp_target_theta , temp_target_R);
            // 检测临时坐标点相对于机器人当前坐标系的位置
            float robot_target_x = 0;
            float robot_target_y =0;
            CetTargetPosition(temp_target_x,temp_target_y,robot_target_x,robot_target_y);
            // printf("临时目标点全局坐标  ( %.2f , %.2f )，相对于机器人坐标  ( %.2f , %.2f )\n",temp_target_x,temp_target_y,robot_target_x,robot_target_y);

            // 根据临时目标点计算自己的移动速度
            geometry_msgs::Twist vel_cmd;
            float robot_target_theta = atan2(robot_target_y, robot_target_x);
            // printf("临时目标点相对于机器人朝向角度=  %.2f \n",robot_target_theta);
            vel_cmd.angular.z = 0.5 * robot_target_theta;
            if(fabs(robot_target_theta) < 0.1)
                vel_cmd.linear.x = 0.2;
            else
                vel_cmd.linear.x = 0.0;

            vel_pub.publish(vel_cmd);

            // 距离临时目标点足够近时，认为已经到达临时目标点，转向最终目标点行驶
            if(sqrt(robot_target_x*robot_target_x + robot_target_y*robot_target_y) < 0.1)
            {
                printf("到达临时目标点坐标，开始去往最终目标\n");
                state = STATE_ARRIVED_TEMP_TARGET;
                ROS_WARN("state ->  STATE_ARRIVED_TEMP_TARGET");
            }
            ShowVFH();
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