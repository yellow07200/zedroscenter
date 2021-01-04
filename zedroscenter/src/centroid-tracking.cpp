 
#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <chrono>
#include <stdio.h>

#include "centroid-tracking.hpp"
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Sample includes
#include "SaveDepth.hpp"


#include <fstream>
#include <sstream>
#include <math.h>  

#include <algorithm>

#include<std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include<std_msgs/Float32MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>


using namespace sl;
using namespace std;
//using namespace cv;

//void printHelp();

	vector<vector<cv::Point>> paths;
	vector<cv::Point> orig_points;
	vector<double> orig_arr;
	vector<cv::Point> points;
	vector<double> deformation;
	vector<cv::Point> defor;
	vector<double> defor_final;
	vector<vector<double> > deformation2d;
	vector<pair<int, pair<int, int>>> orig_objects;
	vector<pair<int, pair<int, int>>> last_objects;

	vector<cv::Point> region_points;
	float distance_final;
	vector<double> depth_final;
	//vector<cv::Point3f> xyzpoint;
	vector<cv::Point3f> region_points_xyz;
	//vector<cv::Point3f> curr_points_xyz;
	vector<cv::Point3f> defor_xyz;
	vector<cv::Point3f> points_xyz;

	vector<cv::Point3f> points_xyz_collect;

	//sl::Mat point_cloud;
	//sl::float4 point_cloud_value;

	float* depths;

namespace zedroscenternode {


void Center::FTBiasedCallback(const geometry_msgs::WrenchStamped::ConstPtr &biased_ft_msg)
{
	ROS_INFO("ft");
	FT_Biased=*biased_ft_msg;

	fx = FT_Biased.wrench.force.x;
	fy = FT_Biased.wrench.force.y;
	fz = FT_Biased.wrench.force.z;
	tx = FT_Biased.wrench.torque.x;
	ty = FT_Biased.wrench.torque.y;
	tz = FT_Biased.wrench.torque.z;

	ft.push_back(fx);ft.push_back(fy);ft.push_back(fz);ft.push_back(tx);ft.push_back(ty);ft.push_back(tz);

	//ROS_INFO("fx",fx);
}

void Center::depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	//const cv::Mat_<uint8_t> d_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO16)->image;
	//ROS_INFO("depth_image");
	float* depths_ptr = (float*)(&msg->data[0]);
	for(int i=0; i<921600; i++)
	{
		depths.push_back(depths_ptr[i]);
	}
	int len=sizeof(msg->data) / sizeof(msg->data[0]);
	std::cout<<"msg->data[0]="<<msg->data[0]<<std::endl;//",msg->data[0][0]"<<msg->data[0][0]<<std::endl;
	ROS_INFO("depth image subscribed!");

	int u = msg->width / 2;
    int v = msg->height / 2;
    int centerIdx = u + msg->width * v;
    ROS_INFO("Center distance : %g m", depths[centerIdx]);
	depth_sign=1;
	std::cout<<"size="<<msg->data.size()<<std::endl;
	std::cout<<"depths[640000]="<<depths[635376]<<std::endl;

	//ROS_INFO("size of depths: %d", sizeof(*depths) );

}

Center::Center(ros::NodeHandle & nh) : nh_(nh) //, ros::NodeHandle nh_private//
{
	ROS_INFO("beginning");
	
	//zed_depth_sub = it.subscribe("/zedm/zed_node/depth/de:depthCallbath_registered",1,&Center::depthCallback,this);
	//zed_image_sub = it.subscribe("/zedm/zed_node/rgb/image_rect_color",1,&Center::imageCallback,this);
	zed_depth_sub = nh_.subscribe("/zedm/zed_node/depth/depth_registered", 1, &Center::depthCallback, this); 
	zed_image_sub = nh_.subscribe("/zedm/zed_node/rgb/image_rect_color", 1, &Center::imageCallback, this);
	ft_sub = nh_.subscribe("/transformed_world", 1, &Center::FTBiasedCallback, this);

	//points_pub = nh_.advertise<std_msgs::Float32MultiArray>("points_info", 1);
	center_pub = nh_.advertise<std_msgs::Float32MultiArray>("center_info", 1);
	center_disp_pub = nh_.advertise<std_msgs::Float32MultiArray>("center_disp_info", 1);
	center_original_pub = nh_.advertise<std_msgs::Float32MultiArray>("center_original_info", 1);
	ft_pub = nh_.advertise<std_msgs::Float32MultiArray>("ft_info", 1);
	
}


void Center::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{

	height=msg->height;
	width=msg->width;
	const cv::Mat l_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->image;//_<uint8_t>
	if(!l_image.data) ROS_INFO("no rgb image!");
	else ROS_INFO("rgb image subscribed!");
	//ROS_INFO("beginning_image");
//}
	iter++;
	std::cout<<"iter="<<iter<<std::endl;
	/* if (sensor_msgs::image_encodings::isColor(msg->encoding))
		cv::Mat cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    else
        cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8); */

	/* Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    Mat depth_image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed); */
    //sl::Mat point_cloud;
	ROS_INFO("DETECTION_ITER");
	
	cv::Mat src;
	l_image.copyTo(src);
	
	ROS_INFO("00");
	if (src.data)
	{
		ROS_INFO("00-1");
    	cv::cvtColor(src, src, cv::COLOR_RGB2GRAY);//CV_BGR2GRAY
	}
	else 	ROS_INFO("no image subscribed from l_image");
	ROS_INFO("01");
    cv::threshold(src, src, 70, 220, cv::THRESH_BINARY);//50, 255 black marker //15, 255, //CV_THRESH_BINARY //150, 255,white marker//140, 255 last good //200, 255
	ROS_INFO("1");
    //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
    //std::vector<std::vector<cv::Point> > contours;
	vector<cv::Mat> contours;
    cv::Mat contourOutput = src.clone();
    cv::findContours( contourOutput, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);// CHAIN_APPROX_SIMPLE );//);/
    //cv::findContours( contourOutput, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	//cout<<"contour.size="<<contours.size()<<endl;
	//namedWindow("Contour1", cv::WINDOW_NORMAL);
    	//cv::imshow("Contour1", contours[1]);
	ROS_INFO("2");


    //Draw the contours
    cv::Mat contourImage(src.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Scalar colors[3];
	vector<cv::Point> curr_points;
    vector<cv::Mat> Better_Contours;
    vector<cv::Mat> Best_Contours;
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
	colors[2] = cv::Scalar(0, 0, 255);
    cv::Mat centroidImage(src.size(), CV_8UC3, cv::Scalar(0,0,0));
    for (size_t idx = 0; idx < contours.size(); idx++) 
	{

		auto area = cv::contourArea(contours[idx]);
		if(area > 10 && area < 200) //(1100,15000) video //area > 10 && area < 200
        {
            Better_Contours.push_back(contours[idx]);
			//cout<<"Better_Contours.size="<<Better_Contours.size()<<endl;
        	//cv::drawContours(contourImage, contours, idx, colors[idx % 3]);
			cv::Moments mo = cv::moments(contours[idx]);
			if (mo.m10/mo.m00<800 && mo.m10/mo.m00>400){
			if (mo.m01/mo.m00<500 && mo.m01/mo.m00>200){
			
			curr_points.push_back(cv::Point(mo.m10/mo.m00 , mo.m01/mo.m00));
			Best_Contours.push_back(contours[idx]);
			}}
			//circle(centroidImage, Point(mo.m10/mo.m00 , mo.m01/mo.m00), 1, cv::Scalar(255, 255, 255), 5, 8, 0);
			//cout<<"m00="<<mo.m00<<endl;
			if (iter==1)
			{
				orig_objects.push_back({idx, {mo.m10/mo.m00, mo.m01/mo.m00}});
				orig_points.push_back(cv::Point(mo.m10/mo.m00 , mo.m01/mo.m00));//orig_points
			}
			
		}
    }
	//sort(begin(region_points), end(region_points),[p](const cv::Point& lhs, const cv::Point& rhs){ return lhs.x < rhs.x;});
	//sort(begin(Best_Contours), end(Best_Contours),[p](const cv::Point& lhs, const cv::Point& rhs){ return lhs.x < rhs.x;});
	cout<<"Best_Contours.size="<<Best_Contours.size()<<endl;
	//cv::imshow("0", Best_Contours[2]);
	if(depth_sign==1)
	{
    for (size_t idx = 0; idx < Best_Contours.size(); idx++) 
	{
		cv::drawContours(contourImage, Best_Contours, idx, colors[idx % 3]);
		//cv::Moments mo = cv::moments(contours[idx]);
		//curr_points.push_back(cv::Point(mo.m10/mo.m00src.cols , mo.m01/mo.m00));
		//if (iter==1)
		//{
		//	orig_objects.push_back({idx, {mo.m10/mo.m00, mo.m01/mo.m00}});
		//	orig_points.push_back(cv::Point(mo.m10/mo.m00 , mo.m01/mo.m00));//orig_points
		//}
		// get average depth of each contour
		//cv::Point tp=Best_Contours[idx](0);
		//cout<<"value="<<Best_Contours[idx].at<double>(1,2)<<",size="<<Best_Contours[idx].size()<<endl;
		//auto tt=Best_Contours[idx].size();	
		vector<cv::Point> A=Best_Contours[idx];
		//cout<<"x="<<A[0].x<<",y="<<A[0].y<<endl;
		std::cout<<"A_size="<<A.size()<<std::endl;
		distance_final=0;		
		int countt=0;
		vector<int> Ax,Ay;
		//ROS_INFO("002");
 		for (int t=0; t<A.size(); t++)
		{
			//ROS_INFO("002-1");
			//sl::float4 point_cloud_value;
			//point_cloud.getValue(A[t].x, A[t].y, &point_cloud_value);
			//std::cout<<"A[t].x="<<A[t].x<<", A[t].y="<<A[t].y<<", width="<<width<<",height="<<height<<std::endl;
			int idx=A[t].y*width+A[t].x;
			//ROS_INFO("002-2");
			//std::cout<<"idx="<<idx<<std::endl;
			//std::cout<<"depths.size="<<depths.size()<<std::endl;
			//std::cout<<"depths[idx]="<<depths[idx]<<std::endl;
			double depth_value=0;
			if(!isnan(depths[idx])) 
			{		
				countt++;
				distance_final += depths[idx];
			}
			//ROS_INFO("003");
			Ax.push_back(A[t].x);
			Ay.push_back(A[t].y); 
		} 
 		//distance_final/=countt;
		//cout<<"distance_final="<<distance_final<<endl;
		sort(Ax.begin(), Ax.end());	
		sort(Ay.begin(), Ay.end());
		//cout<<"Ax="<<Ax[0]<<","<<Ax[10]<<","<<Ax[Ax.size()-1]+1<<endl;
		for (int ax=Ax[0]; ax<Ax[Ax.size()-1]+1; ax++)
		{
			for (int ay=0; ay<Ay[Ay.size()-1]+1; ay++)
			{
				//sl::float4 point_cloud_value;
				//point_cloud.getValue(ax, ay, &point_cloud_value);
				int idx=ay*width+ax;
				double depth_value;
				if(!isnan(depths[idx])) 
				{		
					countt++;
					distance_final += depths[idx];
				}
			}
			
		}
		distance_final/=countt;
		cout<<"distance_final="<<distance_final<<endl;
		//depth_final.push_back(distance_final);
		float xf=(float)curr_points[idx].x;
		float yf=(float)curr_points[idx].y;
		
		xyzpoint.push_back(cv::Point3f(xf,yf,distance_final));//, distance_final));
		Ax.clear();
		Ay.clear();

	
	}
	cout<<"region00="<<orig_points.size()<<endl;
	cout<<"region0="<<xyzpoint.size()<<endl;
	// region points---------------------------------------------------------
		
	if (orig_points.size()>=xyzpoint.size())
	{
		for (int i=0;i <orig_points.size(); i++)
		{
			int aa=0,bb=0;
			if (xyzpoint[i].x>200 && xyzpoint[i].x<1200) aa=1;//orig_points
			if (xyzpoint[i].y>100 && xyzpoint[i].y<700) bb=1;
			if (aa==1&&bb==1)
			{
				if (iter==1)
					region_points_xyz.push_back(cv::Point3f(xyzpoint[i].x,xyzpoint[i].y,xyzpoint[i].z));
				else curr_points_xyz.push_back(cv::Point3f(xyzpoint[i].x,xyzpoint[i].y,xyzpoint[i].z));
			}
		}
	}
	cout<<"region1="<<region_points_xyz.size()<<endl;
	cv::Point3i p;//cv::Point p;
	if (iter==1) 
	{
		//sort(begin(region_points), end(region_points),[p](const cv::Point& lhs, const cv::Point& rhs){ return lhs.x < rhs.x;});
		sort(begin(region_points_xyz), end(region_points_xyz),[p](const cv::Point3f& lhs, const cv::Point3f& rhs){ return lhs.x <= rhs.x;});
		//for (int p=0; p<orig_points.size();p++)
			//cout<<"p=("<<orig_points[p].x<<","<<orig_points[p].y<<endl;
	}
	else sort(begin(curr_points_xyz), end(curr_points_xyz),[p](const cv::Point3f& lhs, const cv::Point3f& rhs){ return lhs.x <= rhs.x;});	
	//else sort(begin(curr_points), end(curr_points),[p](const cv::Point& lhs, const cv::Point& rhs){ return lhs.x <= rhs.x;});
	cout<<"region2="<<curr_points_xyz.size()<<endl;
	

	//Calculating Paths
	//deformation2d.push_back(std::vector<double>());
	
	
    for( int a = 0; a < region_points_xyz.size() ; a++)//Better_Contours.size() 
    {

		vector<double> temp_dis;
		vector<cv::Point3f> temp_points;
		int c=0;
		for( int b = 0; b < curr_points_xyz.size() ; b++)
		{
			double dis=calDis(curr_points_xyz[b].x,curr_points_xyz[b].y,region_points_xyz[a].x,region_points_xyz[a].y);
			
			if (dis<10)//50 white marker video //60 black marker video
			{
				c++;
				//circle(centroidImage, Point(curr_points[a].x,curr_points[a].y), 1, Scalar(255, 255, 255), 5, 8, 0);
				//defor.push_back(Point(curr_points[a].x,curr_points[a].y));
				//deformation.push_back(dis);

				cv::Point3f temp_point1=cv::Point3f(curr_points_xyz[b].x,curr_points_xyz[b].y,curr_points_xyz[b].z);
				//temp_points.push_back(cv::Point3f(curr_points_xyz[b].x,curr_points_xyz[b].y,curr_points_xyz[b].z));
				
				if (c==1) 
				{
					deformation.push_back(dis);
					defor_xyz.push_back(cv::Point3f(curr_points_xyz[b].x,curr_points_xyz[b].y,curr_points_xyz[b].z));
				}
				else if(c>1)
				{
					//deformation.pop_back(); 
					//temp_points.push_back(cv::Point3f(curr_points_xyz[b].x,curr_points_xyz[b].y,curr_points_xyz[b].z));
					//double dis1=calDis(temp_points[0].x,temp_points[0].y,region_points_xyz[a].x,region_points_xyz[a].y);
					//double dis2=calDis(temp_points[1].x,temp_points[1].y,region_points_xyz[a].x,region_points_xyz[a].y);
					double dis1=calDis(temp_point1.x,temp_point1.y,region_points_xyz[a].x,region_points_xyz[a].y);
					//double dis2=calDis(temp_point1.x,temp_points[1].y,region_points[a].x,region_points[a].y);
					if (dis1>dis) //(dis1>dis2) 
					{
						defor_xyz.pop_back();
						defor_xyz.push_back(cv::Point3f(curr_points_xyz[b].x,curr_points_xyz[b].y,curr_points_xyz[b].z));
						deformation.pop_back();
						deformation.push_back(calDis(curr_points_xyz[b].x,curr_points_xyz[b].y,region_points_xyz[a].x,region_points_xyz[a].y));
					}

					temp_point1=cv::Point3f(curr_points_xyz[b].x,curr_points_xyz[b].y,curr_points_xyz[b].z);
								

				}

				//temp_dis.push_back(dis);
			}
			
		}
		//temp_dis.clear();
		
		//putText(centroidImage, dis, Point(cX,cY), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(255,255,255), 2, CV_AA);

        bool close = false;
            
    }
	cout<<"iter="<<iter<<" size1="<<region_points_xyz.size()<<" size2="<<curr_points_xyz.size()<<" size3="<<defor_xyz.size()<<",szie4="<<deformation.size()<<endl;
	for (int q=0;q<defor_xyz.size();q++)
	{
		circle(centroidImage, cv::Point(defor_xyz[q].x,defor_xyz[q].y), 1, cv::Scalar(255, 255, 255), 5, 8, 0);
		points_xyz.push_back(cv::Point3f(defor_xyz[q].x,defor_xyz[q].y,defor_xyz[q].z));
	}
	cout<<" size5="<<defor_xyz.size()<<" size6="<<points_xyz.size()<<endl;
	defor_xyz.clear();


	cout<<"1..."<<endl;

    for (int d = 0; d < region_points_xyz.size(); d++)
    {
		circle(contourImage, cv::Point(region_points_xyz[d].x, region_points_xyz[d].y), 1, cv::Scalar(255, 255, 255), 1, 8, 0);
    }
	cout<<"2..."<<endl;

	cout<<"3..."<<endl;

	/* cv::namedWindow("Input Image", CV_WINDOW_AUTOSIZE);
    cv::imshow("Input Image", src);
	cv::waitKey(1); */
    //cvMoveWindow("Input Image", 0, 0);

	/* namedWindow("src", cv::WINDOW_NORMAL);
   	cv::imshow("src", src);

	namedWindow("Contours", cv::WINDOW_NORMAL);
   	cv::imshow("Contours", contourImage);

	namedWindow("Centroid", cv::WINDOW_NORMAL);
    cv::imshow("Centroid", centroidImage); */
    //cvMoveWindow("Contours", 200, 0);
    //cv::waitKey(0);


    std_msgs::Float32MultiArray center_msg; //(fx,fy,fz,tx,ty,tz, x1,y1,z1, x2,y2,z2, .....) center_points_position_xyz
	/* center_msg.data.push_back(p);
	center_msg.data.push_back(cx);
	center_msg.data.push_back(cy); */
	for (int i=0; i<ft.size(); i++)
	{
		center_msg.data.push_back(ft[i]);
	}
	for (int i=0; i<points_xyz.size(); i++)
	{
		center_msg.data.push_back(points_xyz[i].x);
		center_msg.data.push_back(points_xyz[i].y);
		center_msg.data.push_back(points_xyz[i].z);
	}
	center_pub.publish(center_msg);

	std_msgs::Float32MultiArray center_o_msg; // center_points_original_position_xyz
	for (int i=0; i<region_points_xyz.size(); i++)
	{
		center_o_msg.data.push_back(region_points_xyz[i].x);
		center_o_msg.data.push_back(region_points_xyz[i].y);
		center_o_msg.data.push_back(region_points_xyz[i].z);
	}
	center_original_pub.publish(center_o_msg);

	std_msgs::Float32MultiArray center_disp_msg; // center_points_displacements (compared with original position)
	for (int i=0; i<deformation.size(); i++)
	{
		center_disp_msg.data.push_back(deformation[i]);
	}
	center_disp_pub.publish(center_disp_msg);

	std_msgs::Float32MultiArray ft_msg; // ft information 
	for (int i=0; i<ft.size(); i++)
	{
		ft_msg.data.push_back(ft[i]);
	}
	ft_pub.publish(ft_msg); 

//}
	xyzpoint.clear();
	//points_xyz.clear();
	//region_points_xyz.clear();
	curr_points_xyz.clear();
	depths.clear();
	depth_sign=0;
/* 	depths = NULL;
	delete depths; */
    //zed.close();
} //end-if(depth_sign==1)

	ros::spinOnce(); 
//}//end if(iter>1)
    //return 0; 

}



/**
* This function displays help in console
**/

double Center::calDis(int x1, int y1, int x2, int y2) { //double
    int x = x1 - x2;
    int y = y1 - y2;
    double dist = sqrt((x * x) + (y * y));       //calculating Euclidean distance

    return dist;
}

/* float Center::calDepth(sl::Mat point_cloud, cv::Point point) //float
{
	int xx=point.x; int yy=point.y;
	sl::float4 point_cloud_value;
	point_cloud.getValue(xx, yy, &point_cloud_value);
	float distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

	return distance;
} */

}//namespaces