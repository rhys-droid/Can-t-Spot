#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
 
class ImageOverlay : public rclcpp::Node {

public:

ImageOverlay()
    : Node("imageOverlay")
{
    background_ = cv::imread("/home/rhys/ros2_ws/src/tut9/images/car_junction_gazebo.png", cv::IMREAD_UNCHANGED);
    foreground_ = map("/home/rhys/ros2_ws/src/tut9/images/car_junction_map2.pgm");
 
    if (background_.empty() || foreground_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Error loading images!");
        return;
    }
    if (background_.size() != foreground_.size()) {
        cv::resize(foreground_, foreground_, background_.size());
    }
 
    overlayImages();
}
 
cv::Mat background_;
cv::Mat foreground_;
 
cv::Mat map(const std::string& input_pgm)
{
    cv::Mat grayscale = cv::imread(input_pgm, cv::IMREAD_GRAYSCALE);
 
    cv::Mat rotated;
    cv::rotate(grayscale, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
 
    cv::Mat rgba_image;
    cv::cvtColor(rotated, rgba_image, cv::COLOR_GRAY2BGRA);
 
    for (int y = 0; y < rgba_image.rows; ++y) {
        for (int x = 0; x < rgba_image.cols; ++x) {
            uchar pixel_value = rotated.at<uchar>(y, x);
            if (pixel_value != 0) {
                rgba_image.at<cv::Vec4b>(y, x)[3] = 0;
            } else {
                rgba_image.at<cv::Vec4b>(y, x)[3] = 255;
            }
        }
    }
 
    return rgba_image;
}
 
void overlayImages()
{
    cv::Mat output = background_.clone();
    for (int y = 0; y < background_.rows; ++y) {
        for (int x = 0; x < background_.cols; ++x) {
            cv::Vec4b bg_pixel = background_.at<cv::Vec4b>(y, x);
            cv::Vec4b fg_pixel = foreground_.at<cv::Vec4b>(y, x);
            float alpha = fg_pixel[3] / 255.0;
            for (int c = 0; c < 3; ++c) {
                bg_pixel[c] = bg_pixel[c] * (1.0 - alpha) + fg_pixel[c] * alpha;
            }
            output.at<cv::Vec4b>(y, x) = bg_pixel;
        }
    }
 
    cv::imshow("Composited Image", output);
    cv::waitKey(0);
    cv::imwrite("output_composited.png", output);
}
 

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageOverlay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}