#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>

#include "/home/hemingshan/opencv_ws/src/opencv_examples/include/opencv_examples/imageSolution.h"
#include "/home/hemingshan/opencv_ws/src/opencv_examples/include/opencv_examples/NodeBase.hpp"

//Turtlebot3 camera topic
#define TURTLEBOT3_RGB_TOPIC        "/turtlebot2/camera/rgb/image_raw"
#define TURTLEBOT3_DEPTH_TOPIC      "/turtlebot2/camera/depth/image_raw"

#define TURTLEBOT3_VEL_TOPIC        "/turtlebot2/cmd_vel"

using namespace cv;
using namespace std;

class FindContours : public NodeBase
{
public:
    FindContours(int nArgc, char** ppcArgv, const char* pcNodeName) 
        : NodeBase(nArgc, ppcArgv, pcNodeName)
        {
            //generate ImgTrans NodeHandle
            ImgTrans.reset(new image_transport::ImageTransport(*NodeHandlePtr));
            //Publisher
            PubVel     = NodeHandlePtr->advertise<geometry_msgs::Twist>(TURTLEBOT3_VEL_TOPIC,1);
            //Subscriber
            SubImg_RGB = ImgTrans->subscribe(TURTLEBOT3_RGB_TOPIC,1,
                        boost::bind(&FindContours::ImageRGBCallback, this, _1));
            SubImg_DEPTH = ImgTrans->subscribe(TURTLEBOT3_DEPTH_TOPIC,1,
                        boost::bind(&FindContours::ImageDEPTHCallback, this, _1));
            //Sleep
            ros::Duration(0.1).sleep();
            ROS_INFO("Successfully Initialized FindContours Node!!");

        }
    ~FindContours(){};
public:
    void control()
    {
        ROS_INFO("In Control Loop!");
        if (tar.radius<200)
        {
            ROS_INFO("Go!");
            geometry_msgs::Twist twist;
            twist.linear.x = 0.5;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            if(tar.center.x<(imgsolved.cols/3)){twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.5;}
            else if(tar.center.x>(imgsolved.cols/3)&&tar.center.x<(imgsolved.cols*2/3)){twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.0;}
            else if(tar.center.x>(imgsolved.cols*2/3)){twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = -0.5;}
            PubVel.publish(twist);
            //Sleep
            ros::Duration(0.1).sleep();
        }
        else if (tar.radius>300)
        {
            ROS_INFO("Stop!");
            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            if(tar.center.x<(imgsolved.cols/3)){twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.5;}
            else if(tar.center.x>(imgsolved.cols/3)&&tar.center.x<(imgsolved.cols*2/3)){twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = 0.0;}
            else if(tar.center.x>(imgsolved.cols*2/3)){twist.angular.x = 0.0;twist.angular.y = 0.0;twist.angular.z = -0.5;}
            PubVel.publish(twist);
            //Sleep
            ros::Duration(0.1).sleep();
        }
        
    }
    void Run(void) override
    {
        while(ros::ok())
        {
            control();
            ros::spinOnce();
        }
    }
private:
    void ImageRGBCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            img_rgb = cv_bridge::toCvShare(msg,"bgr8")->image;
            //imgsol.image_solution(img);
            imgsolved = imgsol.image_solution(img_rgb);
            tar = imgsol.image_contour_extraction();
            //ROS_INFO("The turtlebot1 in the camera center is: [%f,%f]",tar.center.x,tar.center.y);
            //ROS_INFO("The cols of this Mat is : [%d]",imgsolved.cols);
            //ROS_INFO("The rows of this Mat is : [%d]",imgsolved.rows);
            ROS_INFO("The turtlebot1 in the camera radius is: %d",tar.radius);
            //Scale
            resize(imgsolved, img_rgb_scaled, Size(),
                    ScaledFactor, ScaledFactor);

            namedWindow("img_rgb",cv::WINDOW_NORMAL);
            imshow("img_rgb",img_rgb_scaled);
            waitKey(1);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'",msg->encoding.c_str());
        }
    }
    void ImageDEPTHCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            img_depth = cv_bridge::toCvShare(msg)->image;
            resize(img_depth, img_depth_scaled, Size(),
                    ScaledFactor, ScaledFactor);
            namedWindow("img_depth",cv::WINDOW_NORMAL);
            imshow("img_depth",img_depth_scaled);
            waitKey(1);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert depth image.");
        }
    }
private:
    std::unique_ptr<image_transport::ImageTransport>    ImgTrans;
    
    ros::Publisher                                      PubVel;
    image_transport::Subscriber                         SubImg_RGB;
    image_transport::Subscriber                         SubImg_DEPTH;
    imageSolution                                       imgsol;
    
    Mat                                                 imgsolved;
    Mat                                                 img_rgb;
    Mat                                                 img_depth;
    Mat                                                 img_rgb_scaled;
    Mat                                                 img_depth_scaled;
    
    target                                              tar;

    float                                               ScaledFactor=0.25;
};

int main(int argc, char** argv)
{
    FindContours node(argc, argv, "find_contours");
    node.Run();
    return 0;
}