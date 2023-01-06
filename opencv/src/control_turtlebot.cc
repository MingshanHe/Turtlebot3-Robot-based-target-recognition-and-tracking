/*
#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Int8.h"

class SubandPub
{
public:
    SubandPub()
    {
        pub_ = n_.advertise<std_msgs::Int8>("/published_topic",1);
        sub_ = n_.subscribe("/subscribed_topic",1,&SubandPub::callback,this);
    }
    void callback(const std_msgs::Int8& input)
    {
        std_msgs::Int8 output;
        int a = 1;
        output.data = a;
        pub_.publish(output);
    }
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};
int main(int argc, char** argv)
{
    ros::init(argc,argv,"SubAndPub");
    SubandPub subandpub;
    ros::spin();
    return 0;
}*/
#include "ros/ros.h"
#include "std_msgs/Int8.h"
//话题回调函数
void chatterCallback(const std_msgs::Int8::ConstPtr& msg)
{
	ROS_INFO("node_b is receiving [%d]", msg->data);
}
 
 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "node_b");	//初始化ROS，节点命名为node_b，节点名必须唯一。
	ros::NodeHandle n;	//节点句柄实例化
	ros::Subscriber sub = n.subscribe("/target_pos", 1, chatterCallback);	//向话题“str_message”订阅，一旦发布节点（node_a）在该话题上发布消息，本节点就会调用chatterCallbck函数。
 
	ros::spin();	//程序进入循环，直到ros::ok()返回false，进程结束。
 
	return 0;
}
