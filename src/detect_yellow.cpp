#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <chrono>
#include <iomanip>
#include <sstream>

#define MIN_AREA 3000

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_subscriber")
    {
        subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));
        publisher = this->create_publisher<geometry_msgs::msg::Point>("center_coordinate", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv::Mat original_image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat processed_image = original_image.clone();

            // Image processing
            cv::Mat hsv;
            cv::cvtColor(processed_image, hsv, cv::COLOR_BGR2HSV);
            cv::Mat mask;
            cv::inRange(hsv, cv::Scalar(20, 100, 100), cv::Scalar(30, 255, 255), mask);
            //cv::inRange(hsv, cv::Scalar(85/2, 50, 50), cv::Scalar(115/2, 255, 255), mask);


            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            cv::Rect largest_rect;
	    int max_area = 0;

            for (const auto& contour : contours)
            {
                cv::Rect rect = cv::boundingRect(contour);
                int area = rect.width * rect.height;

		if (area > max_area)
		{
		     max_area = area;
		     largest_rect = rect;
		}
	    }


	    if (max_area > MIN_AREA)
	    {
		cv::rectangle(processed_image, largest_rect, cv::Scalar(0, 255, 0), 5);

                int center_x = largest_rect.x + largest_rect.width / 2;
                int center_y = largest_rect.y + largest_rect.height / 2;

                // Publish center coordinates
                geometry_msgs::msg::Point point_msg;
                point_msg.x = center_x;
                point_msg.y = center_y;
                point_msg.z = 0; // Z is unused in this case
                publisher->publish(point_msg);
            }
	    
            // Display images
            cv::imshow("Original Image", original_image);
            cv::imshow("Processed Image", processed_image);
            cv::waitKey(1);

	    // Save original image
            std::string original_filename = createOriginalFilename("original_image");
            bool saved = cv::imwrite("/home/dan/images/" + original_filename, original_image);
            if (!saved) 
	    {
                RCLCPP_ERROR(this->get_logger(), "Failed to save original image");
            }

            // Save processed image
            std::string processed_filename = createProcessedFilename("processed_image");
            cv::imwrite("/home/dan/images/" + processed_filename, processed_image);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    // ユニークなタイムスタンプ付きのファイル名を生成
    std::string createOriginalFilename(const std::string& base_name)
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << "_" << base_name << ".jpg";
        return ss.str();
    }
    // ユニークなタイムスタンプ付きのファイル名を生成
    std::string createProcessedFilename(const std::string& base_name)
    {
        auto now = std::chrono::system_clock::now();
        auto now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << "_" << base_name << ".jpg";
        return ss.str();
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
