#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <PerFoRoControl/NavigatePerFoRo.h>
#include <PerFoRoControl/MODE.h>
#include <person_tracking/TrackedObject.h>

using namespace cv;
using namespace std;

class NavigatePerFoRo
{
public:
	NavigatePerFoRo();

	~NavigatePerFoRo() {}

	bool shirt_updated;
	bool pant_updated;
	int PerFoRoMode = 0;

	void 	navigate(bool trueDetected);
	bool 	ifvertical();

protected:
	
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/

	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Subscriber tracked_shirt_sub_;
	ros::Subscriber tracked_pant_sub_;
	ros::Subscriber mode_sub_;
	ros::Publisher navigate_pub_;

	Mat frame;
	Point shirt,pant;
	bool selectObject;
	double AreaRatio = 1.0f;
	int prevmsg = 2, Rows = 240, Columns = 320, AREA = Rows * Columns, interval = 50;
	person_tracking::TrackedObject TrackedShirt, TrackedPant;
	

	void 	drawArrow(Mat image, Point p, Point q, Scalar color, int arrowMagnitude, int thickness, int line_type, int shift);
	void	ImageCallback(const sensor_msgs::ImageConstPtr& msg);
	void 	TrackedShirtCallback(const person_tracking::TrackedObject msg);
	void 	TrackedPantCallback(const person_tracking::TrackedObject msg);
	void 	ModeCallback(const PerFoRoControl::MODE msg);
};
