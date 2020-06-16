#include "ros/ros.h"
#include <math.h>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <cmath>
#include <tbb/atomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>


#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
//********************************************** cv_bridge
#include <cv_bridge/cv_bridge.h>

#include "patterndetector.h"
#include <iostream>
#include <ros/package.h>
#include <math.h>

#define PI 3.14159265
using namespace std;
using namespace boost;
using namespace ARma;

#define PAT_SIZE 64//equal to pattern_size variable (see below) //+_taghirate in che tasiri dare?
#define SAVE_VIDEO 0 //if true, it saves the video in "output.avi"
#define NUM_OF_PATTERNS 1// define the number of patterns you want to use

// char const* filename1 = (ros::package::getPath("arma") + "pattern1.png").c_str();//id=1
// char const* filename2 = (ros::package::getPath("arma") + "pattern2.png").c_str();//id=2
// char const* filename3 = (ros::package::getPath("arma") + "pattern3.png").c_str();//id=3

RNG rng(12345);
pcl::PointCloud<pcl::PointXYZRGB> globalcloud;
cv::Mat camImage;
cv::Mat cameraMatrix;
cv::Mat distCoeffs;
bool image_init = false;
bool calib_init = false;

char* filename1 = "/home/shaghayegh/catkin_ws/src/arma/pattern1.png";//id=1
char* filename2 = "/home/shaghayegh/catkin_ws/src/arma/pattern3.png";//id=2
char* filename3 = "/home/shaghayegh/catkin_ws/src/arma/pattern2.png";//id=3
char* filename4 = "/home/shaghayegh/catkin_ws/src/arma/pattern5.png";//id=3
std::vector<cv::Mat> patternLibrary;
std::vector<Pattern> detectedPattern;
int patternCount=0;// tu khode tabe ++ mishe

PatternDetector *myDetector;

struct position
{
    float x;
    float y;
    float z;
};

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg) ;

int loadPattern(const char* filename, std::vector<cv::Mat>& library, int& patternCount){
    Mat img = imread(filename,0);

    if(img.cols!=img.rows){
        return -1;
        printf("Not a square pattern");
    }

    int msize = PAT_SIZE;

    Mat src(msize, msize, CV_8UC1);
    Point2f center((msize-1)/2.0f,(msize-1)/2.0f);
    Mat rot_mat(2,3,CV_32F);

    resize(img, src, Size(msize,msize));//be 64*64 resize mikone tasviro
    Mat subImg = src(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));//nesfe vasate tasvio mirize tu sub image
    library.push_back(subImg);

    rot_mat = getRotationMatrix2D( center, 90, 1.0);// Center of the rotation in the source image, Rotation angle in degrees. Positive values mean counter-clockwise rotation (the coordinate origin is assumed to be the top-left corner), Isotropic scale factor, the output affine transformation, 2x3 floating-point matrix
    //in matriso jayi zakhire nakard!
    for (int i=1; i<4; i++){
        Mat dst= Mat(msize, msize, CV_8UC1);
        rot_mat = getRotationMatrix2D( center, -i*90, 1.0);// dar 4 jahat matrise rotation ono bedest miare
        warpAffine( src, dst , rot_mat, Size(msize,msize));
        Mat subImg = dst(Range(msize/4,3*msize/4), Range(msize/4,3*msize/4));// 2/4 vasate tasivre dar 4 zavieye mokhtalef zakhire mikone

        library.push_back(subImg);//dar in ketabkhune 5 tasvir zakhire mishavad
    }

    patternCount++;
    return 1;
}

void PointCloudCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, globalcloud);
}
void roscaminfoCallBack(const sensor_msgs::CameraInfoConstPtr &msg)
{
    cameraMatrix = (cv::Mat_<double>(3,3) << msg->K[0], msg->K[1], msg->K[2], msg->K[3], msg->K[4], msg->K[5], msg->K[6], msg->K[7], msg->K[8]); //9
    distCoeffs = (cv::Mat_<double>(5,1) << msg->D[0], msg->D[1], msg->D[2], msg->D[3], msg->D[4]); //5
    calib_init = true;

    //cout<<"get"<<endl;
}

void rosImageCallBack(const sensor_msgs::ImageConstPtr &msg)
{
    if (msg->width != 0 )
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if (cv_ptr->image.size().width > 0)
            {
                cv_ptr->image.copyTo(camImage);
                image_init = true;
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
}
position calcposition(Point points[4])
{
    float a[4] = {0,0,0,0};
    float b[4] = {0,0,0,0};

    position World_Pose;
    World_Pose.x = 0;
    World_Pose.y = 0;
    World_Pose.z = 0;
    int cnt_z = 0;
    int cnt_y = 0;
    int cnt_x = 0;
    pcl::PointXYZ p1;
    bool SlopCheck =  false;
    bool indexOutOfrange = false;
    for(int i=0; i<4;++i)
    {
        if((points[i].x>640)||(points[i].x<0))
            indexOutOfrange = true;
        if((points[i].y>480)||(points[i].y<0))
            indexOutOfrange = true;
    }
    if((globalcloud.size()!=0)&&(!indexOutOfrange))
    {
    
        // cout<<a[0]<<" a[0] "<<a[1]<<" a[1] "<<a[2]<<" a[2] "<<a[3]<<" a[3] "<<endl;
        // cout<<points[0].x<<" p0x "<<points[0].y<<" p0y "<<points[1].x<<" p1x "<<points[1].y<<" p1y "<<points[2].x<<" p2x "<<points[2].y<<" p2y "<<points[3].x<<" p3x "<<points[3].y<<" p3y "<<endl;
        int MaxX = 0;
        int MaxY = 0;
        int MinX = 650;
        int MinY = 650;
        // if(a[0]<0.09)
        // {
        //     a[0]=0;
        // }
        // if(a[2]<0.09)
        // {
        //     a[2]=0;
        // }
        // int sortX[4];
        // int sortY[4];

        // for(int i=0; i<4, ++i)
        // {
        //     sortX[i] = points[i].x;
        //     sortY[i] = points[i].y;
        // }
        // for(int i=0; i<4, ++i)
        // {
        //     for(int j=i+1; j<4, ++j)
        //     {
        //         if(sortX[i]>sortX[j])
        //         {
        //             int temp = sortX[j];
        //             sortX[j] = sortX[i];
        //             sortX[i] = sortX[j];
        //         }
        //         if(sortY[i]>sortY[j])
        //         {
        //             int temp = sortY[j];
        //             sortY[j] = sortY[i];
        //             sortY[i] = sortY[j];
        //         }
        //     }            
        // }
        // cout<<
        int lefest_point_i = 1000;
        int lefest_point_j = 1000;
        int rightest_point_i = -1;
        int rightest_point_j = -1;
        position lefest_point_pos;
        position rightest_point_pos;
        for(int i=0; i<4;++i)
        {
            if(points[i].x>MaxX)
                MaxX = points[i].x;
            if(points[i].x<MinX)
                MinX = points[i].x;
            if(points[i].y>MaxY)
                MaxY = points[i].y;
            if(points[i].y<MinY)
                MinY = points[i].y;
        }
            for (int i=MinX; i<MaxX; ++i)
                for(int j=MinY;j<MaxY;++j)
                {
                            Point pod ;
                            pod.x = i;
                            pod.y = j;
                            Scalar color = Scalar( 0, 0, 255);
                            circle( camImage, pod, 4, color, -1, 8, 0 );
                            // cout<<" a[1] "<<a[1]<<endl;
                            p1.x = globalcloud.at(i, j).x;
                            p1.y = globalcloud.at(i, j).y;
                            p1.z = globalcloud.at(i, j).z;
                            if(!isnan(p1.x) && !isnan(p1.z))
                            {
                            	if(i<lefest_point_i)
                            	{
                            		lefest_point_i=i ;
                            		lefest_point_pos.x = p1.x;
                            		lefest_point_pos.z = p1.z;
                            	
                            	}
                            	if(i>rightest_point_i)
                            	{
                            		rightest_point_i = i;
                            		rightest_point_pos.x = p1.x;
                            		rightest_point_pos.z = p1.z;
                            	}
                            }

                            if ( !isnan(p1.x))
                            {

                                World_Pose.x += p1.x;
                                cnt_x++;
                            }
                            if ( !isnan(p1.y))
                            {
                                World_Pose.y += p1.y;
                                cnt_y++;
                            }
                            if ( !isnan(p1.z) && (p1.z > 0))
                            {
                                World_Pose.z += p1.z;
                                cnt_z++;
                            }
                }
            if ((cnt_x != 0) && (cnt_y != 0) && (cnt_z != 0))
            {
                World_Pose.x = World_Pose.x / cnt_x;
                World_Pose.y = World_Pose.y / cnt_y;
                World_Pose.z = World_Pose.z / cnt_z;
                  double slop = (double(rightest_point_pos.z - lefest_point_pos.z)/double(rightest_point_pos.x - lefest_point_pos.x)); 
        double result = atan (slop) * 180 / PI;
        cout<<result<<"   angular  result "<<endl;// age mosbat bashad be samte chap bayad becharkhad manfi bashe be samte rast bayad becharkhad
            }
        }
    
    return World_Pose;
}
string detect_ramp(position &ramp_pos)
{
    Point Marker_points_1[4];
    Point Marker_points_2[4];
    position ramp_left_pos,ramp_right_pos,ramp_middle_position;

    ramp_left_pos.x = ramp_left_pos.y = ramp_left_pos.z=0; 
    ramp_right_pos.x = ramp_right_pos.y = ramp_right_pos.z=0; 

    for (unsigned int i =0; i<detectedPattern.size(); i++)
        {

            if(detectedPattern.at(i).id==1)
            {
                detectedPattern.at(i).setPoints(cameraMatrix, distCoeffs,Marker_points_1);
                ramp_right_pos = calcposition(Marker_points_1);
                // cout<<" right "<<ramp_right_pos.x<<" x "<<ramp_right_pos.z<<" z "<<endl;
            }
            if(detectedPattern.at(i).id==2)
            {
                detectedPattern.at(i).setPoints(cameraMatrix, distCoeffs,Marker_points_2);
                ramp_left_pos = calcposition(Marker_points_2);
                // cout<<" left "<<ramp_left_pos.x<<" x "<<ramp_left_pos.z<<" z "<<endl;
            }
        }

    if((ramp_left_pos.x!=0)&&(ramp_right_pos.x!=0))
    {
        float tafazol = ramp_left_pos.x-ramp_right_pos.x;
        // cout<<tafazol<<" tafazole x rast ba chap "<<endl;
        ramp_middle_position.x = (ramp_right_pos.x + ramp_left_pos.x)/2;
        ramp_middle_position.y = (ramp_right_pos.y + ramp_left_pos.y)/2;
        ramp_middle_position.z = (ramp_right_pos.z + ramp_left_pos.z)/2;
        ramp_pos.x = ramp_middle_position.x;
        ramp_pos.y = ramp_middle_position.y;
        ramp_pos.z = ramp_middle_position.z;
        double slop = (double(ramp_right_pos.z - ramp_left_pos.z)/double(ramp_right_pos.x - ramp_left_pos.x)); 
        double result = atan (slop) * 180 / PI;
        if(result>0)
        {
        	cout<<result<<"   degree turn left "<<endl;// age mosbat bashad be samte chap bayad becharkhad manfi bashe be samte rast bayad becharkhad
        }
        else
        {
			cout<<result<<"   degree turn right "<<endl;// age mosbat bashad be samte chap bayad becharkhad manfi bashe be samte rast bayad becharkhad
        }
        return "middle";
    }
    else if(ramp_left_pos.x!=0)
    {
        ramp_pos.x = ramp_left_pos.x;
        ramp_pos.y = ramp_left_pos.y;
        ramp_pos.z = ramp_left_pos.z;
        return "left";
    }
    else if(ramp_right_pos.x!=0)
    {
        ramp_pos.x = ramp_right_pos.x;
        ramp_pos.y = ramp_right_pos.y;
        ramp_pos.z = ramp_right_pos.z;
        return "right";
    }
    return "nonseen";
}
int main(int argc, char **argv)
{
    cameraMatrix = (cv::Mat_<double>(3,3) << 0, 0, 0, 0, 0, 0, 0, 0, 0); //9
    distCoeffs = (cv::Mat_<double>(5,1) << 0, 0, 0, 0, 0); //5

    ros::init(argc, argv, "arma_core");
    cout << "arma started done..." << endl;
    ros::Time::init();

    ros::NodeHandle node_handles[3];
    ros::Subscriber sub_handles[3];

    sub_handles[0] = node_handles[0].subscribe("/camera/rgb/image_color", 1, rosImageCallBack);
    sub_handles[1] = node_handles[1].subscribe("/camera/rgb/camera_info", 1, roscaminfoCallBack);
    sub_handles[2] = node_handles[2].subscribe("/camera/depth_registered/points", 1, PointCloudCallBack);

    ros::Rate loop_rate(25); //50 Hz

    
    loadPattern(filename1, patternLibrary, patternCount);
    loadPattern(filename2, patternLibrary, patternCount);
    loadPattern(filename3, patternLibrary, patternCount);
    loadPattern(filename4, patternLibrary, patternCount);
    cout << patternCount << " patterns are loaded." << endl;


    int norm_pattern_size = PAT_SIZE;
    double fixed_thresh = 40;
    double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
    int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode
    double confidenceThreshold = 0.35;
    int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

     myDetector = new PatternDetector(fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);

    while (ros::ok())
    {
        if ( image_init && calib_init )
        {
            image_init = false;
            myDetector->detect(camImage, cameraMatrix, distCoeffs, patternLibrary, detectedPattern);

            cout << "Detected Patterns: " << detectedPattern.size() << endl;
            position ramp_pos;
            cout<<"  ralative to  "<<detect_ramp(ramp_pos)<< "   Marker  ";
            // cout<<ramp_pos.x<<" x "<<ramp_pos.z<<" z "<<endl;

              if((ramp_pos.x == 0)&&(ramp_pos.y == 0)&&(ramp_pos.z == 0))
                {
                    cout<<"i cannot see points cloud"<<endl;
                }
                else{
                    if(ramp_pos.x<0)
                        cout<< ramp_pos.x*100<<" cm boro chap ";
                    else
                        cout<< ramp_pos.x*100<<" cm boro rast ";

                    cout<< ramp_pos.z*100<<" cm boro jelo "<<endl;
                }

            for (unsigned int i =0; i<detectedPattern.size(); i++){
                // detectedPattern.at(i).showPattern();
                Point points[4];
                points[0].x = 4.501;
                cout<<points[0].x<<"fffffffffffffff"<<endl;
                detectedPattern.at(i).draw( camImage, cameraMatrix, distCoeffs,points);
                cout<<detectedPattern.at(i).id<<" id "<<endl;
                position MarkerPos = calcposition(points);
                // if((MarkerPos.x == 0)&&(MarkerPos.y == 0)&&(MarkerPos.z == 0))
                // {
                //     cout<<"i cannot see points cloud"<<endl;
                // }
                // else{
                //     if(MarkerPos.x<0)
                //         cout<< MarkerPos.x*100<<" cm boro chap "<<endl;
                //     else
                //         cout<< MarkerPos.x*100<<" cm boro rast "<<endl;

                //     cout<< MarkerPos.z*100<<" cm boro jelo "<<endl;
                // }
                Scalar color = Scalar( 0, 255, 0);
                // circle( camImage, points[0], 4, color, -1, 8, 0 );
                circle( camImage, points[0], 4, color, -1, 8, 0 );

                color = Scalar( 255, 0, 0);
                // circle( camImage, points[0], 4, color, -1, 8, 0 );
                circle( camImage, points[1], 4, color, -1, 8, 0 );


                color = Scalar( 0, 0, 255);
                // circle( camImage, points[0], 4, color, -1, 8, 0 );
                circle( camImage, points[2], 4, color, -1, 8, 0 );

                color = Scalar( 0, 255, 255);
                // circle( camImage, points[0], 4, color, -1, 8, 0 );
                circle( camImage, points[3], 4, color, -1, 8, 0 );

                // circle( camImage, points[2], 4, color, -1, 8, 0 );
                // circle( camImage, points[3], 4, color, -1, 8, 0 );
            }
            cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
            cv::imshow("image", camImage);
            cv::waitKey(1);
            detectedPattern.clear();
        }

        ros::spinOnce();
        loop_rate.sleep();


    }
    return 0;
}
