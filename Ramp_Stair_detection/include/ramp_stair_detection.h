#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pattern_info.h>
#include <patterndetector.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#define PAT_SIZE 64//equal to pattern_size variable (see below) //+_taghirate in che tasiri dare?
class ramp_stair_detection
{
	private:
		ARma::PatternDetector *myDetector;
        std::vector<cv::Mat> patternLibrary;
        std::vector<ARma::Pattern> detectedPattern;
        std::vector<pattern_info> patterns_info;
		ros::NodeHandle nh_[2];
		ros::Subscriber sub_[2];
		pcl::PointCloud<pcl::PointXYZRGB> globalcloud;
        bool work_start;
		cv::Mat camImage;
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        int patterncount;
		void rosImage_CB(const sensor_msgs::ImageConstPtr &msg);
		void PointCloud_CB(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void roscaminfo_CB(const sensor_msgs::CameraInfoConstPtr &msg);
        int loadPattern(const char* filename, std::vector<cv::Mat>& library, int& patternCount);

	public:
		ramp_stair_detection();
       ~ramp_stair_detection();
        vector<pattern_info> all_detected_pattern();
        void load_pattern(vector<string> filename);
};

