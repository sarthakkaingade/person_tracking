#include "navigate_perforo.h"

/**
 * Navigate PerFoRo
*/

NavigatePerFoRo::NavigatePerFoRo() : 
	it_(nh_)
{
	image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, &NavigatePerFoRo::ImageCallback, this);
	tracked_shirt_sub_ = nh_.subscribe("/track_shirt/tracked_shirt", 10, &NavigatePerFoRo::TrackedShirtCallback, this);
	tracked_pant_sub_ = nh_.subscribe("/track_pant/tracked_pant", 10, &NavigatePerFoRo::TrackedPantCallback, this);
	tracked_dock_sub_ = nh_.subscribe("/track_dock/tracked_dock", 10, &NavigatePerFoRo::TrackedDockCallback, this);
	mode_sub_ = nh_.subscribe("/ModePerFoRo", 1, &NavigatePerFoRo::ModeCallback, this);
	navigate_pub_ = nh_.advertise<PerFoRoControl::NavigatePerFoRo>("/NavigatePerFoRo", 2);
	image_pub_ = it_.advertise("/object_tracking/image_raw", 1);
}

void NavigatePerFoRo::drawArrow(Mat image, Point p, Point q, Scalar color, int arrowMagnitude=9 , int thickness=2, int line_type=8, int shift=0)
{
	//Draw the principle line
	cv::line(image, p, q, color, thickness, line_type, shift);
	const double PI = 3.141592653;
	//compute the angle alpha
	double angle = atan2((double)p.y-q.y, (double)p.x-q.x);
	//compute the coordinates of the first segment
	p.x = (int) ( q.x +  arrowMagnitude * cos(angle + PI/4));
	p.y = (int) ( q.y +  arrowMagnitude * sin(angle + PI/4));
	//Draw the first segment
	line(image, p, q, color, thickness, line_type, shift);
	//compute the coordinates of the second segment
	p.x = (int) ( q.x +  arrowMagnitude * cos(angle - PI/4));
	p.y = (int) ( q.y +  arrowMagnitude * sin(angle - PI/4));
	//Draw the second segment
	line(image, p, q, color, thickness, line_type, shift);
}

void NavigatePerFoRo::TrackedShirtCallback(const person_tracking::TrackedObject msg)
{
	TrackedShirt = msg;
	shirt_updated = true;
}

void NavigatePerFoRo::TrackedPantCallback(const person_tracking::TrackedObject msg)
{
	TrackedPant = msg;
	pant_updated = true;
}

void NavigatePerFoRo::TrackedDockCallback(const person_tracking::TrackedObject msg)
{
	TrackedDock = msg;
	dock_updated = true;
}

void NavigatePerFoRo::ModeCallback(const PerFoRoControl::MODE msg)
{
	PerFoRoMode = msg.MODE;
}

void NavigatePerFoRo::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
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
	if (shirt_updated)	{
		shirt.x = TrackedShirt.x;
		shirt.y = TrackedShirt.y;
		drawArrow(frame, cv::Point(frame.cols/2, frame.rows/2), shirt, Scalar(255,0,0));
	} 
	if (pant_updated)	{
		pant.x = TrackedPant.x;
		pant.y = TrackedPant.y;
		drawArrow(frame, cv::Point(frame.cols/2, frame.rows/2), pant, Scalar(0,255,0));
	}
	if (dock_updated)	{
		dock.x = TrackedDock.x;
		dock.y = TrackedDock.y;
		drawArrow(frame, cv::Point(frame.cols/2, frame.rows/2), dock, Scalar(0,255,0));
	}

	// Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());
}

void NavigatePerFoRo::navigate(bool trueDetected)
{
	PerFoRoControl::NavigatePerFoRo msg;
	if (trueDetected)	{
		//cout<<"Shirt "<<TrackedShirt.x<<" "<<TrackedShirt.area<<" "<<AREA<<endl;
		//cout<<"Pant "<<TrackedPant.x<<" "<<TrackedPant.area<<" "<<(TrackedShirt.area + TrackedPant.area)<<endl;
		AreaRatio = (float)(TrackedShirt.area + TrackedPant.area) / (float)AREA;
		cout.precision(5);
		cout<<"R "<<fixed<<AreaRatio<<endl;
		if ((TrackedShirt.x < (0.3 * Columns)) && (TrackedPant.x < (0.3 * Columns)))	{
			//cout<<"Turn Left"<<endl;
			msg.command = 3;
		} else if ((TrackedShirt.x > (0.7 * Columns)) && (TrackedPant.x > (0.7 * Columns)))	{
			//cout<<"Turn Right"<<endl;
			msg.command = 4;
		} else if ( (AreaRatio < 0.07f) && (TrackedShirt.y > (0.15 * Rows)) && (TrackedShirt.y < (0.85 * Rows)) )	{
			//cout<<"Go Front"<<endl;
			msg.command = 1;
		} else if ( (AreaRatio > 0.15f) || (TrackedShirt.y < (0.1 * Rows)) )	{
			cout<<"Go Back"<<endl;
			msg.command = 2;
		} else	{
			//cout<<"Center"<<endl;
			msg.command = 0;
		}
	} else	{
		msg.command = 0;
	}
	if (prevmsg != msg.command)	{
		prevmsg = msg.command;
		navigate_pub_.publish(msg);
	}
}

bool NavigatePerFoRo::ifvertical()
{
  return (TrackedPant.x >= (TrackedShirt.x - interval) && TrackedPant.x <= (TrackedShirt.x + interval));
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Navigate_PerFoRO");
	NavigatePerFoRo NP;
	ros::Rate loop_rate(20);
	while (ros::ok())
  	{
		if (NP.PerFoRoMode == 3)	{
			if (NP.shirt_updated && NP.pant_updated)	{
				NP.shirt_updated = false;
				NP.pant_updated = false;
				if (NP.ifvertical())	{
					NP.navigate(true);
				} else	{
					NP.navigate(false);
				}
			}
		} else	{
			NP.shirt_updated = false;
			NP.pant_updated = false;
			NP.dock_updated = false;
		}

		ros::spinOnce();
    		loop_rate.sleep();
	}
	return 0;
}
