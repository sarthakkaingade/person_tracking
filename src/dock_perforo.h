#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <PerFoRoControl/MODE.h>
#include <PerFoRoControl/SelectTarget.h>
#include <PerFoRoControl/NavigatePerFoRo.h>
#include <person_tracking/TrackedObject.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Dock PerFoRo Window";
static void onMouse(int event, int x, int y, int, void* );

class DockPerFoRo
{
public:
	DockPerFoRo();

	~DockPerFoRo() {cv::destroyWindow(OPENCV_WINDOW);} 
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
	image_transport::Publisher image_dock_pub_;
	ros::Subscriber mode_sub_;
	ros::Subscriber target_dock_sub_;
	ros::Publisher track_dock_pub_;
	ros::Publisher navigate_pub_;

	Mat frame;
	bool IMSHOW;
	int trackObject;
	int rectOffset;
	int dilation_size, erosion_size;
	Mat elemDilate, elemErode;
	Mat structure_elem;
	Rect selection;
	Scalar mColorRadius;
	Scalar mLowerBound;
	Scalar mUpperBound;
	Point selectCentroid, selectCenter, origin;
	bool selectObject, ObjectDetected;
	int navX, navY, prevmsg = 1, PerFoRoMode = 0, imgcount;
	person_tracking::TrackedObject dock_msg;

	void	ImageCallback(const sensor_msgs::ImageConstPtr& msg);
	void 	ModeCallback(const PerFoRoControl::MODE msg);
	void 	SelectTargetDockCallback(const PerFoRoControl::SelectTarget msg);
	void 	RotatePerFoRo();
};

namespace dp
{
    DockPerFoRo *DP = NULL;
}
