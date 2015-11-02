#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <PerFoRoControl/SelectTarget.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Select Target Person";
static void onMouse(int event, int x, int y, int, void* );

class SelectTarget
{
public:
	SelectTarget();

	~SelectTarget() {cv::destroyWindow(OPENCV_WINDOW);} 

	void 	SelectObject(int event, int x, int y);

protected:
	
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	ros::Subscriber target_sub_;
	ros::Publisher target_shirt_pub_;
	ros::Publisher target_pant_pub_;

	PerFoRoControl::SelectTarget select_target_shirt_msg;
	PerFoRoControl::SelectTarget select_target_pant_msg;
	bool IMSHOW;
	Rect selection;
	bool selectObject;
	Mat frame;
	Point origin;

	void	ImageCallback(const sensor_msgs::ImageConstPtr& msg);
	void 	SelectTargetCallback(const PerFoRoControl::SelectTarget msg);
};

namespace st
{
    SelectTarget *ST = NULL;
}
