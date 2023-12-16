#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <torch/script.h>
#include <chrono>

class ImageSubscriber : public rclcpp::Node {
public:
    ImageSubscriber() : Node("image_subscriber") {
        model = torch::jit::load("/home/dan//ros2_ws/src/strategy/src/quantized_mobilenet_v2.pt");
        model.eval();

        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1));

        last_logged_time_ = std::chrono::steady_clock::now();
    }

private:
   void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        // BGRからRGBに変換
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

        // 画像の前処理
        cv::Mat image_resized;
        cv::resize(image, image_resized, cv::Size(224, 224));
        torch::Tensor tensor_image = torch::from_blob(image_resized.data, {1, 224, 224, 3}, torch::kByte);
        tensor_image = tensor_image.permute({0, 3, 1, 2});  // [B, C, H, W]に変換
        tensor_image = tensor_image.toType(torch::kFloat);
        tensor_image = tensor_image.div(255.0);
        tensor_image = torch::sub(tensor_image, 0.485).div(0.229);  // Normalize

        // 推論の実行
        torch::Tensor output = model.forward({tensor_image}).toTensor();

	// FPS計算
        frame_count_++;
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_logged_time_).count();
        if (duration >= 1) {
            double fps = frame_count_ / duration;
            std::cout << fps << " fps" << std::endl;
            last_logged_time_ = now;
            frame_count_ = 0;
        }

        // 画像表示
        //cv::imshow("Camera Image", image);
        //cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    torch::jit::script::Module model;
    std::chrono::steady_clock::time_point last_logged_time_;
    int frame_count_ = 0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
