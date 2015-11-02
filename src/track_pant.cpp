#include "track_pant.h"

/**
 * Track Pant
*/

TrackPant::TrackPant() : 
	it_(nh_), 
	KF(6, 2, 0)
{
	image_sub_ = it_.subscribe("/ps3_eye/image_raw", 1, &TrackPant::ImageCallback, this);
	target_pant_sub_ = nh_.subscribe("/SelectTargetPantPerFoRo", 1, &TrackPant::SelectTargetPantCallback, this);
	mode_sub_ = nh_.subscribe("/ModePerFoRo", 1, &TrackPant::ModeCallback, this);
	track_pant_pub_ = nh_.advertise<person_tracking::TrackedObject>("/track_pant/tracked_pant", 10);
	image_pant_pub_ = it_.advertise("/track_pant/image_raw", 1);

	structure_elem = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
	maxDistance = 320 + 240;
	dilation_size = 1;
	erosion_size = 1;
	elemDilate = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	elemErode = getStructuringElement( MORPH_ELLIPSE, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );

	IMSHOW = false;
	trackObject = -1;
	rectOffset = 5;
	minDistThresh = 0.1;
	distThresh = minDistThresh;
	missCount = 0;
	selectObject = false;
	mColorRadius = Scalar(20,50,70,0);
	mUpperBound = Scalar(0);
	mLowerBound = Scalar(0);
	//cv::namedWindow(OPENCV_WINDOW);
}

void TrackPant::drawArrow(Mat image, Point p, Point q, Scalar color, int arrowMagnitude=9 , int thickness=2, int line_type=8, int shift=0)
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

void TrackPant::SelectTargetPantCallback(const PerFoRoControl::SelectTarget msg)
{
	Mat clickFrame = frame;
	Mat roiHSV;
	selection.x = msg.x;
	selection.y = msg.y;
	selection.width = msg.width;
	selection.height = msg.height;

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
}

void TrackPant::ModeCallback(const PerFoRoControl::MODE msg)
{
	PerFoRoMode = msg.MODE;
}

void TrackPant::ImageCallback(const sensor_msgs::ImageConstPtr& msg)
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
	} else if (key == 'x')	{
		IMSHOW =  false;
		cvDestroyAllWindows() ;
		//namedWindow(OPENCV_WINDOW);
	}

	if (trackObject == -1)	{
		//Initial stage, before selecting object. Do nothing. Camera view shown as is.
	} else if (trackObject == 0)	{
		rectangle(frame, Point(selection.x,selection.y),Point(selection.x+selection.width,selection.y+selection.height),Scalar(0,0,255),1);
	} else if (PerFoRoMode == 3)	{
		Mat imgHSV, imgThresh, binFrame;
		int contSize;

		cvtColor(frame, imgHSV, CV_BGR2HSV); 

		//Get binary image using HSV threshold
		inRange(imgHSV, mLowerBound, mUpperBound, imgThresh); 

		//Morphological operations to get smoother blobs with reduced noise                
		dilate( imgThresh, imgThresh, elemDilate );
		erode( imgThresh, imgThresh, elemErode ); 
		dilate( imgThresh, imgThresh, elemDilate );                               
		erode( imgThresh, imgThresh, elemErode );                                                       
		morphologyEx(imgThresh, imgThresh, MORPH_OPEN, structure_elem);  
			  
		imgThresh.copyTo(binFrame);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
			      
		/// Find contours
		findContours( binFrame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		contSize = contours.size();
		//cout<<"contours size "<<contSize<<endl;
				                
		//If no contours 
		if (contSize==0)	{       
			navX = 0;
			navY = 0;
			if (IMSHOW)	{             
				imshow(OPENCV_WINDOW, frame);
				//imshow("Binary Image with Detected Object", imgThresh); 
			}
			return;
		}
				                    
		/// Approximate contours to polygons + get bounding rects 
		vector<vector<Point> > contours_poly( contSize );
		vector<Rect> boundRect( contSize );
				  
		/// Get the moments
		vector<Moments> mu(contSize );
		cv::Mat contArea = Mat::zeros(contSize,1,CV_32FC1);
		for( int i = 0; i < contSize; i++ )
		{ 
			approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
			boundRect[i] = boundingRect( Mat(contours_poly[i]) );
				     
			mu[i] = moments( contours[i], false );
				     
			contArea.at<float>(i) = contourArea(contours[i]);
		}

		///  Get the mass centers:
		vector<Point2f> mc( contSize );
		for( int i = 0; i < contSize; i++ )
		{ 
			mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
		}

				
		///Nearest centroid to previous position
		cv::Mat dist = Mat::zeros(contSize,1,CV_32FC1); 
		cv::Mat normDist = Mat::zeros(contSize,1,CV_32FC1);
				
		for( int i = 0; i < contSize; i++ )
		{ 
			dist.at<float>(i) = abs(mc[i].x - selectCentroid.x) + abs(mc[i].y - selectCentroid.y);

			normDist.at<float>(i) = maxDistance - dist.at<float>(i);
		}
				                   
				   
		cv::Mat normSelect= Mat::zeros(contSize,1,CV_32FC1);

		normSelect =  contArea + normDist; //

		cv::Mat sortedSelect = Mat::zeros(contSize,1,CV_32FC1);

		cv::sortIdx(normSelect, sortedSelect, CV_SORT_EVERY_COLUMN+CV_SORT_DESCENDING);
		       
		Point selectPt = mc[sortedSelect.at<int>(0)];

		//If first tracked frame, initialze Kalman
		if (trackObject == 1)	{
			initTracker();
			trackObject = 2;  
		}

		
		//Kalman estimate based on previous state and measurement   
		kalmanEstimatePt = kalmanTracker(selectPt);
		  
		///Distance of object position estimate from previous position
		distPrevCurrent = abs(kalmanEstimatePt.x - selectCentroid.x) + abs(kalmanEstimatePt.y - selectCentroid.y);
		distPrevCurrent = distPrevCurrent / maxDistance;


		if (missCount > 5)	{
			distThresh*=1.5;
		} else	{
			distThresh = minDistThresh;
		}                                               
				        
		/// /////////////////////////////////////////////////////////////
		///Threshold the detected centroid's distance from prev///////////////
		if (distPrevCurrent < distThresh && contArea.at<float>(sortedSelect.at<int>(0)) >= 10)	{
			//Final object position estimate using kalman
			selectCentroid = kalmanEstimatePt;
			if (IMSHOW)	{ 
				rectangle( frame, boundRect[sortedSelect.at<int>(0)], Scalar(255,255,255), 2, 8, 0 );
			}  
			pant_msg.x = selectCentroid.x;
			pant_msg.y = selectCentroid.y;
			pant_msg.area = boundRect[sortedSelect.at<int>(0)].width * boundRect[sortedSelect.at<int>(0)].height;
			track_pant_pub_.publish(pant_msg);
			//cout<<"X="<<navX<<"Y="<<navY<<endl; 
			missCount = 0;
			drawArrow(frame, cv::Point(frame.cols/2, frame.rows/2), selectCentroid, Scalar(255,0,0));
		} else	{
			missCount++;
			navX = 0.0;
			navY = 0.0;
		}
	}
	// Update GUI Window
	//if (IMSHOW)	{
	//	imshow(OPENCV_WINDOW, frame);
		///imshow("Binary Image with Detected Object", imgThresh);
	//}
	//cv::waitKey(3);

	// Output modified video stream
	image_pant_pub_.publish(cv_ptr->toImageMsg());
}
 
void TrackPant::initTracker()
{
	//measurement.setTo(Scalar(0));
	KF.statePre.at<float>(0) = selectCenter.x;
	KF.statePre.at<float>(1) = selectCenter.y;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	KF.statePre.at<float>(4) = 0;
	KF.statePre.at<float>(5) = 0;

	float accelParam = 1;
	KF.transitionMatrix = *(Mat_<float>(6, 6) << 1,0,1,0,accelParam,0, 0,1,0,1,0,accelParam, 0,0,1,0,1,0, 0,0,0,1,0,1, 0,0,0,0,1,0, 0,0,0,0,0,1);
	KF.measurementMatrix = *(Mat_<float>(2, 6) << 1,0,1,0,accelParam,0, 0,1,0,1,0,accelParam);

	//KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	//setIdentity(KF.measurementMatrix);

	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(KF.errorCovPost, Scalar::all(.1));
}
          
          
Point TrackPant::kalmanTracker(Point centroid)
{
	Mat prediction = KF.predict();
	Point predictPt(prediction.at<float>(0),prediction.at<float>(1));

	Mat_<float> measurement(2,1); 

	measurement(0) = centroid.x;
	measurement(1) = centroid.y;

	//Point measPt(measurement(0),measurement(1));

	Mat estimated = KF.correct(measurement);
	Point statePt(estimated.at<float>(0),estimated.at<float>(1));
	    
	//cout<<"kalman est"<<estimated<<endl;
	//cout<<"kalman pt"<<statePt.x<<statePt.y<<endl;
	return statePt;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Track_Pant");
	TrackPant TP;
	ros::spin();
	return 0;
}
