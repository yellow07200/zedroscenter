#ifndef CENTER_H_
#define CENTER_H_

#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>
#include "sl/Camera.hpp"
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <geometry_msgs/WrenchStamped.h>

namespace zedroscenternode
{
    class Center{
    public:
        Center(ros::NodeHandle & nh);//, ros::NodeHandle nh_private);
        //virtual ~Meanshift();
        double calDis(int x1, int y1, int x2, int y2);
        void detection();
        //float calDepth(sl::Mat point_cloud, cv::Point point);


    private:
        ros::NodeHandle nh_;
        void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
        void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void FTBiasedCallback(const geometry_msgs::WrenchStamped::ConstPtr &bias_ft);


        geometry_msgs::WrenchStamped FT_Biased;

       /*  image_transport::ImageTransport it(nh_);
        image_transport::Subscriber zed_depth_sub;
        image_transport::Subscriber zed_image_sub; */

        ros::Subscriber zed_depth_sub;
        ros::Subscriber zed_image_sub;
        ros::Subscriber ft_sub;

        ros::Publisher center_original_pub;
        ros::Publisher center_pub;
        //ros::Publisher points_pub;
        ros::Publisher center_disp_pub;
        ros::Publisher ft_pub;

        int iter;
        std::vector<cv::Point3f> xyzpoint;
        std::vector<cv::Point3f> curr_points_xyz;

        const cv::Mat_<uint8_t> l_image;
        const cv::Mat_<uint8_t> d_image;

        std::vector<float> depth;
        std::vector<float> ft;

        float fx,fy,fz,tx,ty,tz;

        float* depths;

    }; // class



} //namespace

#endif // CENTER_H_
