/* =============================================================
 *
 * This Node was developed for initial testing of "Terrain steeping"
 * of the KI assignment. No TCP. Publish over ROS instead.
 *
 * Vision-related functions were written by Moonyoung Lee.
 *
 * E-mail : ml634@kaist.ac.kr (Moonyoung Lee)

 *
 * Versions :
 * v0.1.0 (20.02.20) 
 * =============================================================
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>
#include <string> 
#include <fstream>

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <vector> 
//#include "liftBox/KINECT_DATA.h"
#include "RBLANData.h"

#include "object_detector/bbox3d.h"

//matrix transformation
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//OpenCV for image handling
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.h>


//to find min,max pixel
#include <algorithm>
#include <math.h>

//ROS Image msg
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>


// PCL specific includes
#include <sensor_msgs/PointCloud2.h>

//poses 
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"

#define PODO_ADDR "10.12.3.30"
#define PODO_PORT 5500

int     CreateSocket(const char *addr, int port);
int     Connect2Server();
void*   LANThread(void *);

ros::Publisher      sendKinectData_pub;
ros::Publisher 		marker_publisher; 
ros::Publisher 		marker_publisher_filter; 
//liftBox::KINECT_DATA    TXData;
/* Define the global message to send */
//liftBox::KINECT_DATA msg;
std::ofstream outputFile;

int tcp_size = 0;
int RXDataSize = 0;
int sock = 0;
struct sockaddr_in  server;
pthread_t LANTHREAD_t;
int TXDataSize;
void* TXBuffer;
void* RXBuffer;

int count = 0;
int stateMachine_counter = -1;
//min,max bound for color threshold
//neon green
//int minH = 24, maxH = 46, minS = 120, maxS = 228, minV = 83, maxV = 138;
//pink
//int minH = 154, maxH = 180, minS = 55, maxS = 212, minV = 140, maxV = 233;
//yellow
int minH = 9, maxH = 49, minS = 100, maxS = 252, minV = 87, maxV = 205;
//int minH = 10, maxH = 27, minS = 32, maxS = 255, minV = 129, maxV = 225;
int boxX = 0;
int boxY = 0;
float boxAngle = 0;
float robotAngle = 0.0;


				
float cameraHeight = 1.55;
float cameraAngle = 35;
float cameraRad = cameraAngle*3.14/180.0;
float tunedOffset = -0.35;

bool first_detect_flag = true;
bool vector_empty = false;
ros::Time endTime, beginTime;

#define opening              2
#define rectangleConst       0
#define contourAreaThreshold 5000
#define boxLikeRatioMax      1.5
#define boxLikeRatioMin      0.7
//#define boxLikeRatioMax      2.0
//#define boxLikeRatioMin      0.5

#define centerPixelX         320
#define centerPixelY         240
#define oneThirdPixelX       210
#define twoThirdPixelX       420
#define twoThirdPixelY       320
#define plane_distance_threshold 0.15

LAN_PODO2VISION RXdata;

ros::Publisher pub;
ros::Publisher 		pose_publisher;

// Global variable for subscribing the data
int numPlanningSteps = 0; 
geometry_msgs::PoseArray stepsArray_pixel;
geometry_msgs::PoseArray stepsArray_pose;
geometry_msgs::PoseArray stepsArray_pose_mean;
geometry_msgs::Pose 	 current_com_pose;

std::vector<geometry_msgs::PoseArray> stepsArray_filtered (0);
int frame_wait_counter = 0;
int stateMachine_loopCompletion = 0;

bool pcloud_callback_done = true;

/* initialize CV Window
 * MOONYOUNG 03.21 */
void initVideoWindow() {

    cv::namedWindow("RGBview");
    // cv::namedWindow("HSVview");
     cv::namedWindow("HSVThreshold");
     cv::namedWindow("HSVFilter");

    cv::startWindowThread();

}

/* display images in Window
 * MOONYOUNG 03.21
   input: Mat of all displayed img */
void displayImages(cv::Mat image1, cv::Mat image2, cv::Mat image3, cv::Mat image4) {


    cv::imshow("RGBview", image1);
    //cv::imshow("HSVview", image2);
    //cv::imshow("HSVThreshold", image3);
    cv::imshow("HSVFilter", image4);
    
    cv::waitKey(30);


}


//returns euclidian distance between two pts to compare if within near distance
float get_distance_twoPts(geometry_msgs::Pose input_a, geometry_msgs::Pose input_b)
{
	//ROS_INFO("a.x: %f, b.x: %f\n", input_a.position.x, input_b.position.x);
	float distance;
	
	distance =  pow( (pow( (input_a.position.x - input_b.position.x),2) + pow( (input_a.position.y - input_b.position.y),2)) , 0.5);
	return distance;
	
}

void get_current_com(const geometry_msgs::Pose input_pose) 
{
		
	current_com_pose.position.x = input_pose.position.x;
	current_com_pose.position.y = input_pose.position.y;
	ROS_INFO("RX comX: %f, comY: %f\n",current_com_pose.position.x, current_com_pose.position.y );
	
	//reset so that FSM can loop 3 times until next reset
	stateMachine_loopCompletion = 0;
	first_detect_flag = true;

} 



void cloud_cb (sensor_msgs::PointCloud2ConstPtr const& input)
{
	
 	

	geometry_msgs::PointStamped base_point;
	//static tf::TransformListener listner;
    //std::cout << "=== handling pointCloud === " << std::endl;
    
    if(stateMachine_counter != 0) return;
    stateMachine_counter = 1;
    ROS_INFO("====== State 0. Img. SizePixelArray: %lu\n ====== ", stepsArray_pixel.poses.size());


    int width = input->width;
    int height = input->height;
    //printf("cloud width: %u, height: %u, rowstep: %u, pointstep: %u\n" , input->width, input->height, input->row_step, input->point_step);
	
    //get pose for multiple steps positions
    for(int i =0; i < stepsArray_pixel.poses.size() ; i ++)
    {
		
		int arrayPosition = (int)stepsArray_pixel.poses[i].position.y*input->row_step + (int)stepsArray_pixel.poses[i].position.x*input->point_step;
	   //printf("position: %u\n" , arrayPosition);
	   
	   // compute position in array where x,y,z data start
      int arrayPosX = arrayPosition + input->fields[0].offset; // X has an offset of 0
      int arrayPosY = arrayPosition + input->fields[1].offset; // Y has an offset of 4
      int arrayPosZ = arrayPosition + input->fields[2].offset; // Z has an offset of 8

      float X = 0.0;
      float Y = 0.0;
      float Z = 0.0;

      memcpy(&X, &input->data[arrayPosX], sizeof(float));
      memcpy(&Y, &input->data[arrayPosY], sizeof(float));
      memcpy(&Z, &input->data[arrayPosZ], sizeof(float));

		//printf("step : %d, x: %f, y: %f, z: %f\n" , i, X, Y, Z);
		
		//add pose to stepArray
		geometry_msgs::Pose step_pose;
		
		step_pose.position.x = X;
		step_pose.position.y = Y;
		step_pose.position.z = Z;
		
		stepsArray_pose.poses.push_back(step_pose);	

	}
	
	ROS_INFO("====== State 1. PCloud CB. StepArraySize: %lu\n ====== ", stepsArray_pose.poses.size());

}



/* get box X,Y and draw bounding box
 * MOONYOUNG 03.21
   input: all contours of points, Mat to draw on*/
void getBoundingBox(std::vector<std::vector<cv::Point> > contourInput, cv::Mat imageIn)
{
    //print contour size
    //std::cout << "Number of Contour: " << contourInput.size() << std::endl;
    numPlanningSteps = 0;
    
    

    //loop through all detected contour
    for (int contourIndex = 0; contourIndex  < contourInput.size(); contourIndex++)
    {
        //if detected contour is big enough
        //std::cout << "counterarea: " << cv::contourArea(contourInput[contourIndex]) << std::endl;
        if ((cv::contourArea(contourInput[contourIndex])) > contourAreaThreshold)
        {
            //get boundingBox
            //fit an minimum Rect around contour
            cv::RotatedRect box = cv::minAreaRect(contourInput[contourIndex]);
            cv::Point2f box_points[4];
            box.points(box_points);
  
            //to get width, length of rotated box
            float deltaX01, deltaY01, deltaX12, deltaY12;
            deltaX01 = box_points[0].x - box_points[1].x;
            deltaY01 = box_points[0].y - box_points[1].y;
            deltaX12 = box_points[2].x - box_points[1].x;
            deltaY12 = box_points[1].y - box_points[2].y;

            double height = pow( pow(deltaX01,2)+ pow(deltaY01,2) ,0.5);
            double width  = pow( pow(deltaX12,2)+ pow(deltaY12,2) ,0.5);
            
            //std::cout << "height: " << height << std::endl;
            //std::cout << "width: " << width << std::endl;
            double ratio_hor = height/width;
            double ratio_ver = width/height;
            
            //ROS_INFO("h: %f, w: %f, h/w: %f, w/h: %f\n",height, width, ratio_hor, ratio_ver);


            //if boundingBox has width/height ratio like box
            if( ((height / width) < boxLikeRatioMax) && ((height / width) > boxLikeRatioMin) )
            {
                //std::cout << " ==== Found Box! ==="  << std::endl;
                
                //add to vector size
                numPlanningSteps = numPlanningSteps + 1;
                
                //draw green bounding box
                //for(int side = 0; side <4; side++) cv::line(imageIn, box_points[side], box_points[(side+1)%4], cv::Scalar(0,255,0),3 );

                //get center position of box
                cv::circle(imageIn,box.center,4,cv::Scalar(0,255,0),2,8,0);
                boxX = box.center.x;
                boxY = box.center.y;
                boxAngle = box.angle;
                //printf("i: %d, Box X,Y: (%f,%f)\n", contourIndex, box.center.x, box.center.y);
                //std::cout << "Box X, Y: (" << box.center.x << " , " << box.center.y  << ") and width, height: (" << height << ", " << width << ")" << std::endl;
                
                //add to vector
                geometry_msgs::Pose step_pixel;
                step_pixel.position.x = box.center.x;
                step_pixel.position.y = box.center.y;
                stepsArray_pixel.poses.push_back(step_pixel);
                
                			
				 
            }//end box detected case
        }//end size detected case
    }//end contour loop
    //ROS_INFO("========= State 0. added pixel value to pixelArray ======= ");

}


/*call back function from imageSub
 * MOONYOUNG 03.21
 * input: sensor_msgs
 * output: update global var msg */
int image_cb_counter = 0;
void boxImageHandler(const sensor_msgs::ImageConstPtr& msgInput)
{
	image_cb_counter++;
    //convert ROS image to OpenCV img pixel encoding
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvShare(msgInput, "bgr8");

    //color change RGB2HSV
    cv::Mat HSVImage;
    cv::cvtColor(cv_ptr->image, HSVImage, CV_BGR2HSV);

    //threshold HSV range for box
    cv::Mat HSVThreshold;
    cv::inRange(HSVImage, cv::Scalar(minH, minS, minV), cv::Scalar(maxH, maxS, maxV),HSVThreshold);

    //opening filter by dilating then eroding image
    cv::Mat HSVFilter;
    cv::Mat morphKernal = cv::getStructuringElement(rectangleConst, cv::Size(9,9), cv::Point(-1,-1) );
    cv::morphologyEx(HSVThreshold,HSVFilter,opening, morphKernal);

    //segment objects by contour
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(HSVFilter, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
	
	if(image_cb_counter > 5 && stateMachine_counter == -1) {
		//get box info ==> update msg
		stateMachine_counter = 0;
		getBoundingBox(contours, cv_ptr->image);
		
	}
    

    //display
    displayImages(cv_ptr->image, HSVImage, HSVThreshold, HSVFilter);
    

}


void output_plot()
{
	outputFile.open("/home/rainbow/Desktop/step_position_compare_filtered.csv");
			
			for(int i = 0 ; i < 300; i ++) {
				
				for(int j=0; j < stepsArray_filtered.size(); j ++) {
					
					if( i < stepsArray_filtered[j].poses.size() ) {
						//add
						outputFile << stepsArray_filtered[j].poses[i].position.x <<  "," << stepsArray_filtered[j].poses[i].position.y << "," ;
					}
					else {
						//skip
						
						outputFile << "," << ",";
					}
					
					
				}
				outputFile << std::endl;
			}
			
	ROS_INFO("================================================================");
}

int main(int argc, char **argv)
{

   ros::init(argc, argv, "COM_peak_detect");
	ros::NodeHandle nh;

	pose_publisher = nh.advertise<geometry_msgs::Pose>("mobile_hubo/com_pose_current", 1);
	ros::Rate loop_rate(40);
	initVideoWindow();
	
    /* spin */
    while(nh.ok())
    {
			 
			int key = (cv::waitKey(0) & 0xFF);
			
			if (key == 'a')
			{ //a
					
					ROS_INFO("pressed A. publishing");
					//publish
					geometry_msgs::Pose current_com;
					current_com.position.x = 0;
					current_com.position.y = 0;
					current_com.position.z = 0.8;
					

					pose_publisher.publish(current_com); 
						
			}
			

		
		 	

        loop_rate.sleep(); // Go to sleep according to the loop period defined above.
        ros::spinOnce();
 
    }


    return 0;
}
