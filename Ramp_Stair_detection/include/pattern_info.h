#include <opencv2/core/core.hpp>

class pattern_info
{
	public:
		cv::Point pos_in_image[4];
		float x;
		float y;
		float z;
		double slop;
		int id;
};

