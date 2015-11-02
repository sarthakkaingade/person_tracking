#include "select_target.h"

/**
 * Select Target Person
*/

SelectTarget::SelectTarget() : 
	it_(nh_)
{
	image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, &SelectTarget::ImageCallback, this);
	target_sub_ = nh_.subscribe("/SelectTargetPerFoRo", 1, &SelectTarget::SelectTargetCallback, this);
	target_shirt_pub_ = nh_.advertise<PerFoRoControl::SelectTarget>("/SelectTargetShirtPerFoRo", 1);
	target_pant_pub_ = nh_.advertise<PerFoRoControl::SelectTarget>("/SelectTargetPantPerFoRo", 1);

	IMSHOW = false;
	
	selectObject = false;
	//cv::namedWindow(OPENCV_WINDOW);
}

void SelectTarget::SelectTargetCallback(const PerFoRoControl::SelectTarget msg)
{
	selection.x = msg.x;
	selection.y = msg.y;
	selection.width = msg.width;
	selection.height = msg.height;
	select_target_shirt_msg.x = selection.x + (selection.width/4);
	select_target_shirt_msg.y = selection.y;
	select_target_shirt_msg.width = selection.width/4;
	select_target_shirt_msg.height = selection.height/4;
	target_shirt_pub_.publish(select_target_shirt_msg);
	select_target_pant_msg.x = selection.x + (selection.width/4);
	select_target_pant_msg.y = selection.y + (3*selection.height/4);
	select_target_pant_msg.width = selection.width/4;
	select_target_pant_msg.height = selection.height/4;
	target_pant_pub_.publish(select_target_pant_msg);
}

void SelectTarget::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
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
	if (key ==27 )	{
		ros::requestShutdown();
	} else if ( key =='z' )	{
		IMSHOW = true;
		//namedWindow(OPENCV_WINDOW);
		//setMouseCallback(OPENCV_WINDOW, onMouse, NULL); 
	} else if (key == 'x')	{
		IMSHOW =  false;
		cvDestroyAllWindows() ;
		//namedWindow(OPENCV_WINDOW);
	}

	// Update GUI Window
	//if (IMSHOW)	{
	//	imshow(OPENCV_WINDOW, frame);
		///imshow("Binary Image with Detected Object", imgThresh);
	//}
	//cv::waitKey(3);
}    

void SelectTarget::SelectObject(int event, int x, int y)
{
	fflush(stdout);
	if (selectObject)	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch(event)
	{
		case CV_EVENT_LBUTTONDOWN:
		{
			//cout<<"L BTN DWN"<<endl;
			//drawing = true;
			origin = Point(x,y);
			selection = Rect(x,y,0,0);
			selectObject = true;
			//cout<<"Left B"<<origin<<endl;
			break;
		}

		case CV_EVENT_LBUTTONUP:
		{
			select_target_shirt_msg.x = selection.x + (selection.width/4);
			select_target_shirt_msg.y = selection.y;
			select_target_shirt_msg.width = selection.width/4;
			select_target_shirt_msg.height = selection.height/4;
			target_shirt_pub_.publish(select_target_shirt_msg);
			select_target_pant_msg.x = selection.x + (selection.width/4);
			select_target_pant_msg.y = selection.y + (3*selection.height/4);
			select_target_pant_msg.width = selection.width/4;
			select_target_pant_msg.height = selection.height/4;
			target_pant_pub_.publish(select_target_pant_msg);
			break;
		}
	}
}           

void onMouse(int event, int x, int y, int, void* )
{
	st::ST->SelectObject(event,x,y);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Select_Target");
	st::ST = new SelectTarget;
	ros::spin();
	return 0;
}
