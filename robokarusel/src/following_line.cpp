#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "robokarusel/SetSpeed.h"
#include "robokarusel/FollowLine.h"

class DetectLine
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;



public:
	DetectLine()
		:	it_(nh_)
	{
		image_sub_ = it_.subscribe("/main_camera/image_raw", 1, &DetectLine::findLineCenter, this);
		image_pub_ = it_.advertise("/following_line/detect_line", 1);
		ROS_INFO("Inited");
	}

	ros::ServiceServer service = nh_.advertiseService("following_line", &DetectLine::process, this);


	float Kp, Ki, Kd;
	int32_t userSpeed = 0, error, j = 0;
	bool PIDReset = false;
    bool setNewSpeed = true;

	bool process(robokarusel::FollowLine::Request &req,
	             robokarusel::FollowLine::Response &res)
	{
		if (req.kp) Kp = req.kp;
		else Kp = 0;
		Ki = req.ki;
		Kd = req.kd;
		userSpeed = req.speed;

		PIDReset = true;
		res.status = true;

		if (userSpeed == 0) { setNewSpeed = false; j = 0;}
		else setNewSpeed = true;
		return true;
	}


	void findLineCenter(const sensor_msgs::ImageConstPtr& msg)
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Rect roi(70, 140, 170, 90);
		cv::Mat roiImg = image(roi);
		cv::Mat output = roiImg.clone();

		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		cv::cvtColor(roiImg, roiImg, CV_BGR2GRAY);

		// blur to remove some noise
		cv::blur(roiImg, roiImg, cv::Size(5, 5), cv::Point(-1,-1));

		// separate track from background as possible as it can
		cv::threshold(roiImg, roiImg, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

		//reverse color on image
		cv::bitwise_not(roiImg, roiImg);

		//find contours
		cv::findContours(roiImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));


		int MaxAreaContourIndex = -1;
		float MaxArea = 0.f;

		for (size_t s = 0; s < contours.size(); s++)
		{
			float Area = cv::contourArea(contours[s]);
			if (Area > MaxArea)
			{
				MaxAreaContourIndex = s;
				MaxArea = Area;
			}
		}
		if (MaxArea > 200 && MaxAreaContourIndex >= 0)
		{
			cv::Moments mu;
			mu = cv::moments(contours[MaxAreaContourIndex], false);
			cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);

			//std::cout << center << std::endl;

			//std::cout << "center.x = " << center.x << std::endl;

			error = (center.x - 85) * 100 / 85;

			//ROS_INFO("Error = %d", error);

			cv::line(output, center, cv::Point(85, 35), cv::Scalar(0, 255, 0), 1, 8, 0);
			cv::circle(output, center, 5, cv::Scalar(255, 255, 255), -1, 8, 0);
		}
		cv::drawContours(output, contours, MaxAreaContourIndex, cv::Scalar(0, 0, 0), 2, 8, hierarchy, 0, cv::Point());

		cv_bridge::CvImage out_msg;
		out_msg.header.frame_id = msg->header.frame_id;
		out_msg.header.stamp = msg->header.stamp;
		out_msg.encoding = sensor_msgs::image_encodings::BGR8;
		out_msg.image = output;
		image_pub_.publish(out_msg.toImageMsg());
		LineNavigator(error);
	}




	void LineNavigator(int error)
	{
		ros::ServiceClient client = nh_.serviceClient<robokarusel::SetSpeed>("set_speed");
		robokarusel::SetSpeed srv;

		int32_t errorDiff;
		int32_t newSpeed;
		int32_t newRightSpeed, newLeftSpeed;
		int32_t accumulatedShiftEror, thisShiftError, lastShiftError;


		if (PIDReset)
		{
			lastShiftError = error;
			PIDReset = false;
		}

		thisShiftError = error;

		accumulatedShiftEror += thisShiftError;

		errorDiff = (thisShiftError - lastShiftError);

		newSpeed = Kp * thisShiftError + Ki * accumulatedShiftEror + Kd * errorDiff;

		lastShiftError = thisShiftError;

		ROS_INFO("kp = %d, ki = %d, kd = %d", Kp, Ki, Kd);
		if (userSpeed == 0)
		{
			newRightSpeed = 0;
			newLeftSpeed = 0;
		}
		else
		{
			newRightSpeed = userSpeed - newSpeed;
			newLeftSpeed = userSpeed + newSpeed;
		}

		if (newRightSpeed > 99) newRightSpeed = 99;
		if (newLeftSpeed > 99) newLeftSpeed = 99;

		//ROS_INFO("UserSpeed = %d, newRightSpeed = %d, newLeftSpeed = %d", userSpeed, newRightSpeed, newLeftSpeed);

		srv.request.newRightSpeed = newRightSpeed;
		srv.request.newLeftSpeed = newLeftSpeed;

		if (setNewSpeed || (!setNewSpeed && j == 0))
		{
			if (client.call(srv))
            	ROS_INFO("newRightSpeed = %d, newLeftSpeed = %d", srv.request.newRightSpeed, srv.request.newLeftSpeed);
			else
				ROS_ERROR("Failed to call service speed");
			j++;
		}

	}
};



int main(int argc, char** argv)
{
	ros::init(argc, argv, "following_line");
	DetectLine ic;
	ros::spin();
	return 0;
}
