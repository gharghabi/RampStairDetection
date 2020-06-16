#include <ros/ros.h>
#include "ramp_stair_detection.h"

using namespace cv;
ramp_stair_detection::ramp_stair_detection()
{
    sub_[0] = nh_[0].subscribe("/camera/rgb/image_color", 1, &ramp_stair_detection::rosImage_CB,this);
    sub_[1] = nh_[1].subscribe("/camera/depth_registered/points", 1, &ramp_stair_detection::PointCloud_CB,this);
    sub_[2] = nh_[2].subscribe("/camera/rgb/camera_info", 1, &ramp_stair_detection::roscaminfo_CB,this);
    int norm_pattern_size = PAT_SIZE;
    double fixed_thresh = 40;
    double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
    int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode
    double confidenceThreshold = 0.35;
    int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD
    myDetector = new ARma::PatternDetector(fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);
    cameraMatrix = (cv::Mat_<double>(3,3) << 0, 0, 0, 0, 0, 0, 0, 0, 0); //9
    distCoeffs = (cv::Mat_<double>(5,1) << 0, 0, 0, 0, 0); //5
    work_start = false;
    patterncount = 0;
}
ramp_stair_detection::~ramp_stair_detection()
{

}
void ramp_stair_detection::rosImage_CB(const sensor_msgs::ImageConstPtr &msg)
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
                if(work_start)
                {
                    detectedPattern.clear();//!!!!!!dorost shavad
                    myDetector->detect(camImage, cameraMatrix, distCoeffs, patternLibrary, detectedPattern);

                    for (unsigned int i =0; i<detectedPattern.size(); i++)
                    {
                        // detectedPattern.at(i).showPattern();
                        Point points[4];
                        detectedPattern.at(i).draw( camImage, cameraMatrix, distCoeffs,points);

                        patterns_info.x
//                            cout<<detectedPattern.at(i).id<<" id "<<endl;
//                            position MarkerPos = calcposition(points);
						patterns_info.x = MarkerPos.x;
						patterns_info.y = MarkerPos.y;
patterns_info.z = MarkerPos.z;
                    }
                    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
                    cv::imshow("image", camImage);
                    cv::waitKey(1);
                }
            }
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

}
void ramp_stair_detection::PointCloud_CB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::fromROSMsg(*msg, globalcloud);
}
void ramp_stair_detection::roscaminfo_CB(const sensor_msgs::CameraInfoConstPtr &msg)
{
    cameraMatrix = (cv::Mat_<double>(3,3) << msg->K[0], msg->K[1], msg->K[2], msg->K[3], msg->K[4], msg->K[5], msg->K[6], msg->K[7], msg->K[8]); //9
    distCoeffs = (cv::Mat_<double>(5,1) << msg->D[0], msg->D[1], msg->D[2], msg->D[3], msg->D[4]); //5
    //     calib_init = true;
}
vector<pattern_info> ramp_stair_detection::all_detected_pattern()
{
     return patterns_info;
}
int ramp_stair_detection::loadPattern(const char* filename, std::vector<cv::Mat>& library, int& patterncount)
{
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

    patterncount++;
    return 1;
}
void ramp_stair_detection::load_pattern(vector<string> filename)
{
    for(int i=0; ++i; i<filename.size())
    {
        loadPattern(filename[i].c_str(), patternLibrary, patterncount);
    }
}
