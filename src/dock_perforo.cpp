#include "dock_perforo.h"

/**
 * Dock PerFoRo
*/

DockPerFoRo::DockPerFoRo() : 
	it_(nh_)
{
	image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, &DockPerFoRo::ImageCallback, this);
	mode_sub_ = nh_.subscribe("/ModePerFoRo", 1, &DockPerFoRo::ModeCallback, this);
	target_dock_sub_ = nh_.subscribe("/SelectTargetDockPerFoRo", 1, &DockPerFoRo::SelectTargetDockCallback, this);
	image_dock_pub_ = it_.advertise("/dock_perforo/image_raw", 1);
	track_dock_pub_ = nh_.advertise<person_tracking::TrackedObject>("/track_dock/tracked_dock", 10);
	navigate_pub_ = nh_.advertise<PerFoRoControl::NavigatePerFoRo>("/NavigatePerFoRo", 2);

	IMSHOW = false;
	selectObject = false;
	ObjectDetected = false;
	imgcount = 0;
	trackObject = -1;
	dilation_size = 1;
	erosion_size = 1;
	elemDilate = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	elemErode = getStructuringElement( MORPH_ELLIPSE, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
	mColorRadius = Scalar(10,100,100,0);
	mUpperBound = Scalar(0);
	mLowerBound = Scalar(0);
//	cv::namedWindow(OPENCV_WINDOW);
}

void DockPerFoRo::ModeCallback(const PerFoRoControl::MODE msg)
{
	PerFoRoMode = msg.MODE;
}

void DockPerFoRo::SelectTargetDockCallback(const PerFoRoControl::SelectTarget msg)
{
	if (PerFoRoMode == 4)	{
		Mat clickFrame = frame;
		Mat roiHSV;
		selection.x = msg.x;
		selection.y = msg.y;
		selection.width = msg.width;
		selection.height = msg.height;

		if( selection.width > 0 && selection.height > 0 )
			trackObject = 1;

		//defines roi
		cv::Rect roi( selection.x, selection.y, selection.width, selection.height );

		//copies input image in roi
		cv::Mat image_roi = clickFrame(roi);

		cvtColor(image_roi, roiHSV, CV_BGR2HSV);

		//computes mean over roi
		cv::Scalar hsvColor = cv::mean( roiHSV );
		cout<<"Dock hsv"<<hsvColor<<endl;

		double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
		double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 179) ? hsvColor.val[0]+mColorRadius.val[0] : 179;

		mLowerBound.val[0] = minH;
		mUpperBound.val[0] = maxH;

		mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
		mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

		mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
		mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

		mLowerBound.val[3] = 0;
		mUpperBound.val[3] = 255;
	}
}

void DockPerFoRo::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}

	frame = cv_ptr->image;

	char key = (char)cvWaitKey(10);
	if (key == 27 )	{
		ros::requestShutdown();
	} else if ( key =='z' )	{
		IMSHOW = true;
		namedWindow(OPENCV_WINDOW);
		setMouseCallback(OPENCV_WINDOW, onMouse, NULL); 
	} else if (key == 'x')	{
		IMSHOW =  false;
		cvDestroyAllWindows() ;
		namedWindow(OPENCV_WINDOW);
	}

	if ((trackObject == 1) && (PerFoRoMode == 4))	{
		Mat imgHSV, binFrame, imgThresh;
 
		cvtColor(frame, imgHSV, CV_BGR2HSV); 

		//Get binary image using HSV threshold
		inRange(imgHSV, mLowerBound, mUpperBound, imgThresh); 
		
		//Morphological operations to get smoother blobs with reduced noise                
		dilate( imgThresh, imgThresh, elemDilate );
		erode( imgThresh, imgThresh, elemErode ); 
		dilate( imgThresh, imgThresh, elemDilate );                               
		erode( imgThresh, imgThresh, elemErode );                                                       
		morphologyEx(imgThresh, imgThresh, MORPH_OPEN, structure_elem);
		//imshow("Binary Image with Detected Object", imgThresh);
		// Find contours
		imgThresh.copyTo(binFrame);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
			      
		/// Find contours
		findContours( binFrame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
		vector<Point> approx;
		for (int i = 0; i < contours.size(); i++)
		{
			// Approximate contours
			approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);
			// Skip small or non-convex objects
			if (fabs(contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))	{
				continue;
			} else	{
				// Detect and label circles
				double area = contourArea(contours[i]);
				Rect r = boundingRect(contours[i]);
				int radius = r.width / 2;
				if (abs(1 - ((double)r.width / r.height)) <= 0.2 && abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)	{
					rectangle( frame, r, Scalar(255,255,255), 2, 8, 0 );
					ObjectDetected = true;
					dock_msg.x = r.x;
					dock_msg.y = r.y;
					dock_msg.area = r.width * r.height;
					track_dock_pub_.publish(dock_msg);
					break;
				}
			}
		}
		if ((ObjectDetected == false) && (imgcount == 10))	{
			RotatePerFoRo();
			imgcount = 0;
		}
		imgcount++;
		// Output modified video stream
		image_dock_pub_.publish(cv_ptr->toImageMsg());
	} else {
		ObjectDetected = false;
		imgcount = 0;
	}
	// Update GUI Window
	if (IMSHOW)	{
		imshow(OPENCV_WINDOW, frame);
		///imshow("Binary Image with Detected Object", imgThresh);
	}
	//cv::waitKey(3);
}

void DockPerFoRo::RotatePerFoRo()
{
	PerFoRoControl::NavigatePerFoRo msg;
	msg.command = 3;
	navigate_pub_.publish(msg);
	ros::Duration(1).sleep();
	msg.command = 5;
	navigate_pub_.publish(msg);
	ros::Duration(1).sleep();
}

void DockPerFoRo::SelectObject(int event, int x, int y)
{
	Mat roiHSV;
	fflush(stdout);
	if (selectObject)	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch(event)
	{

	case CV_EVENT_RBUTTONDOWN:
	{
		Mat clickFrame = frame;
		Mat hSV;

		selection = Rect(x-rectOffset,y-rectOffset,rectOffset*2,rectOffset*2);

		trackObject=0;

		selectCenter.x = x;
		selectCenter.y = y;

		selectCentroid = selectCenter;


		cvtColor(clickFrame, hSV, CV_BGR2HSV);

		cv::Mat roiHSV = hSV(selection);

		///computes mean over roi
		cv::Scalar hsvColor = cv::mean( roiHSV );
		cout<<"hsv"<<hsvColor<<endl;

		double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
		double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 179) ? hsvColor.val[0]+mColorRadius.val[0] : 179;

		mLowerBound.val[0] = minH;
		mUpperBound.val[0] = maxH;

		mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
		mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

		mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
		mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

		mLowerBound.val[3] = 0;
		mUpperBound.val[3] = 255;

		trackObject = -1;
		//cout<<"Bounds"<<mLowerBound.val[0]<<"  "<<mUpperBound.val[0]<<endl;

		break;
	}


	case CV_EVENT_LBUTTONDOWN:
	{
		//cout<<"L BTN DWN"<<endl;
		//drawing = true;
		origin = Point(x,y);
		selection = Rect(x,y,0,0);
		selectObject = true;
		trackObject=0;
		//cout<<"Left B"<<origin<<endl;
		break;
	}

	case CV_EVENT_LBUTTONUP:
	{
		Mat clickFrame = frame;
		//cout<<"L BTN UP"<<endl;
		selectObject = false;
		if( selection.width > 0 && selection.height > 0 )
		  trackObject = 1;

		selectCenter.x = (int)(selection.x + selection.width/2);
		selectCenter.y = (int)(selection.y + selection.height/2);

		selectCentroid = selectCenter;

		//defines roi
		cv::Rect roi( selection.x, selection.y, selection.width, selection.height );

		//copies input image in roi
		cv::Mat image_roi = clickFrame(roi);

		cvtColor(image_roi, roiHSV, CV_BGR2HSV);

		//computes mean over roi
		cv::Scalar hsvColor = cv::mean( roiHSV );
		cout<<"hsv"<<hsvColor<<endl;

		double minH = (hsvColor.val[0] >= mColorRadius.val[0]) ? hsvColor.val[0]-mColorRadius.val[0] : 0;
		double maxH = (hsvColor.val[0]+mColorRadius.val[0] <= 179) ? hsvColor.val[0]+mColorRadius.val[0] : 179;

		mLowerBound.val[0] = minH;
		mUpperBound.val[0] = maxH;

		mLowerBound.val[1] = hsvColor.val[1] - mColorRadius.val[1];
		mUpperBound.val[1] = hsvColor.val[1] + mColorRadius.val[1];

		mLowerBound.val[2] = hsvColor.val[2] - mColorRadius.val[2];
		mUpperBound.val[2] = hsvColor.val[2] + mColorRadius.val[2];

		mLowerBound.val[3] = 0;
		mUpperBound.val[3] = 255;

		break;
	}
	}
}           

void onMouse(int event, int x, int y, int, void* )
{
	dp::DP->SelectObject(event,x,y);
}
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "Dock_PerFoRo");
	dp::DP = new DockPerFoRo;
	ros::spin();
	return 0;
}
