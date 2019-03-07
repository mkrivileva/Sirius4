#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "robokarusel/ChangeImageSize.h"

class Test
{
private:
	ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
	ros::ServiceServer service;
	int startX = 0, startY = 0, width = 0, height = 0;

public:
    Test()
        :   it_(nh_)
    {
		image_sub_ = it_.subscribe("/main_camera/image_raw", 1, &Test::findLineCenter, this);
        image_pub_ = it_.advertise("/following_line/detect_line", 1);
		service = nh_.advertiseService("change_image_size", &Test::Change, this);
        ROS_INFO("Inited");
    }


	bool Change(robokarusel::ChangeImageSize::Request &req,
				robokarusel::ChangeImageSize::Response &res)
	{
		startX = req.startX;
		startY = req.startY;
		width = req.width;
		height = req.height;
		res.status = true;
		return true;
	}

	void findLineCenter(const sensor_msgs::ImageConstPtr& msg)
    {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Rect rect (startX, startY, width, height);
		cv::rectangle(image, rect, cv::Scalar(0, 255, 0));
		cv_bridge::CvImage out_msg;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = image;
        image_pub_.publish(out_msg.toImageMsg());
	}
};

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "test");
	Test ic;
	ros::spin();
	return 0;
}
