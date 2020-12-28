 
#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <chrono>

#include "meanShift-huang.hpp"

#include <sl/Camera.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Sample includes
#include <SaveDepth.hpp>


#include <fstream>
#include <sstream>
#include <math.h>  

#include <algorithm>

using namespace sl;
using namespace std;
//using namespace cv;

cv::Mat slMat2cvMat(Mat& input);
void printHelp();
double calDis(int x1, int y1, int x2, int y2) {
    int x = x1 - x2;
    int y = y1 - y2;
    double dist = sqrt((x * x) + (y * y));       //calculating Euclidean distance

    return dist;
}

float calDepth(Mat point_cloud, cv::Point point)
{
	int xx=point.x; int yy=point.y;
	sl::float4 point_cloud_value;
	point_cloud.getValue(xx, yy, &point_cloud_value);
	float distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);

	return distance;
}

int main(int argc, char **argv) {
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
	vector<cv::Point3f> xyzpoint;
	vector<cv::Point3f> region_points_xyz;
	vector<cv::Point3f> curr_points_xyz;
	vector<cv::Point3f> defor_xyz;
	vector<cv::Point3f> points_xyz;

	vector<cv::Point3f> points_xyz_collect;
    // Create a ZED camera object
    Camera zed;

    // Set configuration parameters
    InitParameters init_params;
    init_params.camera_resolution = RESOLUTION::HD1080;
    init_params.depth_mode = DEPTH_MODE::ULTRA;
    init_params.coordinate_units = UNIT::METER;
    if (argc > 1) init_params.input.setFromSVOFile(argv[1]);
        
    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Display help in console
    printHelp();

    // Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD;

    // Prepare new image size to retrieve half-resolution images
    Resolution image_size = zed.getCameraInformation().camera_resolution;
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    Resolution new_image_size(new_width, new_height);

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat image_ocv = slMat2cvMat(image_zed);
    Mat depth_image_zed(new_width, new_height, MAT_TYPE::U8_C4);
    cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
    Mat point_cloud;
    //Mat depth_measure;

    // Loop until 'q' is pressed
    char key = ' ';
	int iter=0;
    while (key != 'q') {
	iter++;

        if (zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

            // Retrieve the left image, depth image in half-resolution
            zed.retrieveImage(image_zed, VIEW::LEFT, MEM::CPU, new_image_size);
            zed.retrieveImage(depth_image_zed, VIEW::DEPTH, MEM::CPU, new_image_size);
            //zed.retrieveMeasure(depth_measure, MEASURE::DEPTH);//, MEM::CPU, new_image_size);

            // Retrieve the RGBA point cloud in half-resolution
            // To learn how to manipulate and display point clouds, see Depth Sensing sample
            zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, new_image_size);

            // Display image and depth using cv:Mat which share sl:Mat data
            cv::imshow("Image", image_ocv);
            cv::imshow("Depth", depth_image_ocv);


//-------------------------------------------
    //for ( int i = 1; i < 15; i = i + 2 )
     //   { GaussianBlur( src, src, Size( i, i ), 0, 0 );}
//Prepare the image for findContours
	cv::Mat src=image_ocv;
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);//CV_BGR2GRAY
    cv::threshold(src, src, 70, 220, cv::THRESH_BINARY);//50, 255 black marker //15, 255, //CV_THRESH_BINARY //150, 255,white marker//140, 255 last good //200, 255

    //Find the contours. Use the contourOutput Mat so the original image doesn't get overwritten
    //std::vector<std::vector<cv::Point> > contours;
	vector<cv::Mat> contours;
    cv::Mat contourOutput = src.clone();
    cv::findContours( contourOutput, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);// CHAIN_APPROX_SIMPLE );//);/
    //cv::findContours( contourOutput, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	//cout<<"contour.size="<<contours.size()<<endl;
	//namedWindow("Contour1", cv::WINDOW_NORMAL);
    	//cv::imshow("Contour1", contours[1]);

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
    	for (size_t idx = 0; idx < Best_Contours.size(); idx++) 
	{
		cv::drawContours(contourImage, Best_Contours, idx, colors[idx % 3]);
		//cv::Moments mo = cv::moments(contours[idx]);
		//curr_points.push_back(cv::Point(mo.m10/mo.m00 , mo.m01/mo.m00));
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
		distance_final=0;		
		int countt=0;
		vector<int> Ax,Ay;
		for (int t=0; t<A.size(); t++)
		{
			/*sl::float4 point_cloud_value;
			point_cloud.getValue(A[t].x, A[t].y, &point_cloud_value);
			float distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
			if (!isnan(distance))
			{		
				countt++;
				distance_final += distance;
			}*/
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
				sl::float4 point_cloud_value;
				point_cloud.getValue(ax, ay, &point_cloud_value);
				float distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
				if (!isnan(distance))
				{		
					countt++;
					distance_final += distance;
				}
			}
			
		}
		distance_final/=countt;
		cout<<"distance_final="<<distance_final<<endl;
		//depth_final.push_back(distance_final);
		float xf=(float)curr_points[idx].x;
		float yf=(float)curr_points[idx].y;
		
		xyzpoint.push_back(cv::Point3f(xf,yf,distance_final));//, distance_final));

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
								

					/*for(int v=0;v<temp_dis.size();v++)
					{
						dis2=dis2+temp_dis[v];
					}
					dis2=dis2/temp_dis.size();
					deformation.push_back(dis2);
					
					defor.pop_back();
					defor.push_back(Point(curr_points[a].x,curr_points[a].y));*/
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

 	 if (cv::waitKey(10) == 27)
  	{
  		 cout << "Esc key is pressed by user. Stoppig the video" << endl;
  		 break;
  	}
	cout<<"3..."<<endl;
    //cv::imshow("Input Image", src);
    //cvMoveWindow("Input Image", 0, 0);

	namedWindow("src", cv::WINDOW_NORMAL);
   	cv::imshow("src", src);

	namedWindow("Contours", cv::WINDOW_NORMAL);
   	cv::imshow("Contours", contourImage);

	namedWindow("Centroid", cv::WINDOW_NORMAL);
    	cv::imshow("Centroid", centroidImage);
    //cvMoveWindow("Contours", 200, 0);
    //cv::waitKey(0);


            // Handle key event
            key = cv::waitKey(10);
            processKeyEvent(zed, key);
	cout<<"file recordig..."<<endl;
//record-------------------------------------------
	fstream file_dis;
  	file_dis.open("/home/user/Documents/ZED/displacements.csv");
  	for (int i=0;i<deformation.size();i++)
  	{
		if ((i+1)%25!=0)	
		{
			file_dis << deformation[i]<<",";
		}	
		else file_dis << deformation[i]<<"\n";
  	}
  	file_dis.close();

	fstream file_ori;
  	file_ori.open("/home/user/Documents/ZED/original_points.csv");

  	for (int i=0;i<region_points_xyz.size();i++)
  	{

		file_ori << region_points_xyz[i].x<<","<< region_points_xyz[i].y<<","<<region_points_xyz[i].z<<"\n";

  	}
  	file_ori.close();

	fstream file_points;
  	file_points.open("/home/user/Documents/ZED/points.csv");
	cout<<" size7="<<points_xyz.size()<<endl;
  	for (int i=0;i<points_xyz.size();i++)
  	{
		if ((i+1)%25!=0)	//42
		{
			file_points << points_xyz[i].x<<","<< points_xyz[i].y<<","<<points_xyz[i].z<<",";
		}	
		else file_points << points_xyz[i].x<<","<< points_xyz[i].y<<","<<points_xyz[i].z<<"\n";
  	}
  	file_points.close();

	fstream file_depth;
  	file_depth.open("/home/user/Documents/ZED/depth.csv");
  	for (int i=0;i<points_xyz.size();i++)
  	{
		int xx=points_xyz[i].x; int yy=points_xyz[i].y;
		if ((i+1)%25!=0)	//42
		{
			file_depth << depth_image_ocv.at<double>(xx,yy)<<",";
		}	
		else file_depth <<  depth_image_ocv.at<double>(xx,yy)<<"\n";
  	}
  	file_depth.close();

	fstream file_depth_zed;
  	file_depth_zed.open("/home/user/Documents/ZED/depth_zed.csv");
  	for (int i=0;i<points_xyz.size();i++)
  	{
		int xx=points_xyz[i].x; int yy=points_xyz[i].y;
		//sl::float4 point_cloud_value;
		//float distance;
		//point_cloud.getValue(points[i].x, points[i].y, &point_cloud_value);
		//distance = sqrt(point_cloud_value.x*point_cloud_value.x + point_cloud_value.y*point_cloud_value.y + point_cloud_value.z*point_cloud_value.z);
		if ((i+1)%25!=0)	//42
		{		
			//file_depth_zed << distance_final<<",";
			file_depth_zed << points_xyz[i].z <<",";
			//file_depth_zed << depth_image_zed.at<double>(xx,yy)<<",";
		}	
		else file_depth_zed <<points_xyz[i].z<<"\n";
  	}
  	file_depth_zed.close();



    }
	xyzpoint.clear();
	//points_xyz.clear();
	//region_points_xyz.clear();
	curr_points_xyz.clear();
}
    zed.close();



    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }



    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM::CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}