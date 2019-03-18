#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "robokarusel/SearchObject.h"

//using namespace std;
class GetCurrentObject
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_res_pub_;
	image_transport::Publisher image_cur_obj_pub_;
	std::string object = "cyl", res_color = "no";
	ros::ServiceServer service;

public:
	GetCurrentObject()
		:	it_(nh_)
	{
		image_sub_ = it_.subscribe("/main_camera/image_raw", 1, &GetCurrentObject::getImage, this);
		image_res_pub_ = it_.advertise("/following_line/current_object", 1);
		image_cur_obj_pub_ = it_.advertise("/following_line/connected_components", 1);
		service = nh_.advertiseService("search_object", &GetCurrentObject::getCurrentObjectHandle, this);
		ROS_INFO("Service SearchObject is inited");
	}

	bool getCurrentObjectHandle(robokarusel::SearchObject::Request  &req,
								robokarusel::SearchObject::Response &res)
	{
		if (req.status)
		{
			res.object = object;
			res.color = res_color;
		}
		return true;
	}

	void sendImage(cv::Mat image, image_transport::Publisher image_pub_)
	{
		cv_bridge::CvImage out_msg;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = image;
        image_pub_.publish(out_msg.toImageMsg());
	}

	void getImage(const sensor_msgs::ImageConstPtr& msg)
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
		cv::Mat img_edge;
		cv::resize(image, image, cv::Size(), 0.5, 0.5);
		int tresh = 90;
		cv::cvtColor(image, img_edge, cv::COLOR_BGR2GRAY);
		cv::threshold(img_edge, img_edge, tresh, 255, cv::THRESH_BINARY);
		cv::bitwise_not(img_edge, img_edge);
		getCurrentObject(image, img_edge);
	}

	void getCurrentObject(cv::Mat image, cv::Mat img_edge)
	{
		cv::Mat labels, stats, centroids;
		int i, nccomps;
		nccomps = cv::connectedComponentsWithStats (
			img_edge,
			labels,
			stats,
			centroids
		);
		int min_dist = 10000, min_idx = 10000, dist;
		for (int i = 1; i <= nccomps; i++)
		{
			double x = centroids.at<double>(i, 0);
			double y = centroids.at<double>(i, 1);
			if( stats.at<int>(i, cv::CC_STAT_AREA) > 100 )
			{
				dist = getDist(image.size().width, image.size().height, x, y);
				if (dist < min_dist) {min_dist = dist; min_idx = i;}
			}
		}
		cv::Mat connectedComponents = image.clone();
		if (min_idx != 10000)
		{
			double x = centroids.at<double>(min_idx, 0);
			double y = centroids.at<double>(min_idx, 1);
			for( int y = 0; y < image.rows; y++ )
				for( int x = 0; x < image.cols; x++ )
				{
					int label = labels.at<int>(y, x);
					if (label == min_idx)
						connectedComponents.at<cv::Vec3b>(y, x) = cv::Vec3b(255, 0, 0);
				}
		}
        sendImage(connectedComponents, image_cur_obj_pub_);
		setColorFilter(labels, image, min_idx);
	}

	int getDist(int width, int height, double x, double y)
	{
		int center_x = width / 2;
		int center_y = height / 2;

		int dist = sqrt((pow(center_x - x, 2.0)) + pow(center_y - y, 2.0));
		return dist;
	}

	void setColorFilter(cv::Mat labels, cv::Mat img, int min_idx)
	{
		std::vector <cv::Scalar> lower; // the lower boundaries of the colors in the HSV color space
		lower.push_back(cv::Scalar(160, 70, 90)); //red
		lower.push_back(cv::Scalar(0, 0, 60)); //green
		lower.push_back(cv::Scalar(110, 30, 70)); //blue
		lower.push_back(cv::Scalar(0, 40, 90)); //yellow

		std::vector <cv::Scalar> upper; // the upper boundaries of the colors in the HSV color space
		upper.push_back(cv::Scalar(180, 110, 150)); //red
		upper.push_back(cv::Scalar(125, 25, 90)); //green
		upper.push_back(cv::Scalar(131, 66, 110)); //blue
		upper.push_back(cv::Scalar(22, 85, 120)); //yellow

		std::vector <std::string> color_name; // the names of color
		color_name.push_back("red"); color_name.push_back("green");
		color_name.push_back("blue"); color_name.push_back("yellow");

		std::vector <cv::Scalar> color; // standart colors
		color.push_back(cv::Scalar(0, 0, 255)); color.push_back(cv::Scalar(0, 255, 0));
		color.push_back(cv::Scalar(255, 0, 0)); color.push_back(cv::Scalar(0, 255, 217));

		cv::Mat blurred, hsv, kernel, mask;
		bool is_cube = false;

		cv::GaussianBlur(img, blurred, cv::Size(11, 11), 0, 0);
		cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
		for (int i = 0; i < lower.size(); i++)
		{
			kernel = cv::Mat(9, 9, CV_8UC1, cv::Scalar(1));
			cv::inRange(hsv, lower[i], upper[i], mask);
			cv::morphologyEx( mask, mask, cv::MORPH_OPEN, kernel );
			cv::morphologyEx( mask, mask, cv::MORPH_CLOSE, kernel );

			std::vector<std::vector<cv::Point> > contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::findContours( mask, contours, hierarchy, CV_RETR_EXTERNAL, 
							  CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
			for (int j = 0; j < contours.size(); j++)
			{
				cv::drawContours( img, contours, j, color[i], 1, 8, hierarchy, 0, cv::Point() );
				cv::Moments mu;
				mu = cv::moments(contours[j], false);
				cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
				cv::circle(img, center, 5, cv::Scalar(0, 0, 0), -1, 8, 0);
				int label = labels.at<int>(center.y, center.x);
				if (label == min_idx)
				{
					cv::putText(img, color_name[i], center, cv::FONT_HERSHEY_DUPLEX,1.0,color[i], 2);
					object = "cube"; res_color = color_name[i];
					//cout << "Color was found. Its name is " << color_name[i] << endl;
					is_cube = true;
					break;
				}
			}
			if (is_cube) break;
		}
		if (!is_cube) object = "cyl";
		sendImage(img, image_res_pub_);
	}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_current_object");
    GetCurrentObject ic;
    ros::spin();
    return 0;
}
