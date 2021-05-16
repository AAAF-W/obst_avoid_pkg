#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// 速度发布对象
ros::Publisher vel_pub;

// 将激光雷达测距转换成xy坐标值用到的变量
static double x_cos[360];
static double y_sin[360];
static float pnt_x[360];
static float pnt_y[360];

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// 将激光雷达测距转换成xy坐标值
	for (int i = 0; i < 360; i++)
	{
		pnt_x[i] = scan->ranges[i] * x_cos[i];
		pnt_y[i] = scan->ranges[i] * y_sin[i];
	}

	// 搜索前方障碍物
	float min_x = 999.99;
	int min_index = 180;
	for (int i = 150; i < 210; i++)
	{
		// 横向坐标小于0.4米的障碍物，被认为有碰撞风险
		if (fabs(pnt_y[i]) < 0.4)
		{
			if (pnt_x[i] < min_x)
			{
				// 找出有碰撞风险的障碍物中，距离机器人最近的障碍点
				min_x = pnt_x[i];
				min_index = i;
			}
		}
	}
	ROS_WARN("min_x= %.2f min_y = %.2f min_index= %d", min_x, pnt_y[min_index], min_index);

	// 构建速度控制消息包
	geometry_msgs::Twist vel_cmd;
	if (min_x < 1.0)
	{
		// 如果前方障碍物小于1米，则根据障碍物在左边还是右边，进行平移避障
		vel_cmd.linear.x = 0;
		if (min_index < 180)
		{
			// 障碍物在右边，向左平移避障
			vel_cmd.linear.y = 0.05;
		}
		else
		{
			// 障碍物在左边，向右平移避障
			vel_cmd.linear.y = -0.05;
		}
	}   
	else
	{
		// 前方1米之内没有障碍物的话，机器人往前直行
		vel_cmd.linear.x = 0.05;
		vel_cmd.linear.y = 0;
	}
	vel_pub.publish(vel_cmd);
}
 
int main(int argc, char** argv)
{
	ros::init(argc,argv,"behavior_node");
	ROS_INFO("behavior_node start!");

	// 初始化系数变量
	double kStep = (M_PI * 2) / 360;
	for (int i = 0; i < 360; i++)
	{
		x_cos[i] = -1 * cos(M_PI*0.0 - kStep*i);
		y_sin[i] = -1 * sin(M_PI*0.0 - kStep*i);
	}

	// 初始化消息对象，并执行订阅发布操作
	ros::NodeHandle nh;
	ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);
	vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	ros::spin();
}
