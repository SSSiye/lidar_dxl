#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>

using std::placeholders::_1;
using namespace cv;
using namespace std;

cv::Point point_from_angle(const cv::Point& center, double angle_deg, int length)
    {
        double angle_rad = angle_deg * CV_PI / 180.0;
        int x = static_cast<int>(center.x + length * std::cos(angle_rad));
        int y = static_cast<int>(center.y - length * std::sin(angle_rad));  // y축 보정
        return cv::Point(x, y);
    }
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    // 영상 이진화 (흑백 변환 후 Otsu 이진화)
    cv::Mat gray, bin;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);

    int center_x = frame.cols / 2;
    int center_y = frame.rows / 2;
    Point center(center_x, center_y); //영상의 중심심
    int width = frame.cols;
    int height = frame.rows/2;

    Mat bin_left;
    Mat bin_right;

    if (center_x > 1 && width - center_x - 1 > 0) {
        bin_left = bin(Rect(0, 0, center_x - 1, height)).clone();
        bin_right = bin(Rect(center_x + 1, 0, width - center_x - 1, height)).clone();
    } else {
        std::cerr << "❌ Error: Invalid image size for splitting left/right." << std::endl;
        return;
    }
    
    // 레이블링
    Mat l_labels, l_stats, l_centroids;
    int l_num_labels = connectedComponentsWithStats(bin_left, l_labels, l_stats, l_centroids);
    Mat r_labels, r_stats, r_centroids;
    int r_num_labels = connectedComponentsWithStats(bin_right, r_labels, r_stats, r_centroids);

    double r_angle_deg = 0.0; // 오른쪽 각도
    double r_angle_rad = r_angle_deg * CV_PI / 180.0;
    double l_angle_deg = 180.0; // 왼쪽쪽 각도
    double l_angle_rad = l_angle_deg * CV_PI / 180.0;

    int length = 100;

    Point r_end(
    center_x + static_cast<int>(length * std::cos(r_angle_rad)),
    center_y - static_cast<int>(length * std::sin(r_angle_rad))  // y는 반대
    );
    Point l_end(
    center_x + static_cast<int>(length * std::cos(l_angle_rad)),
    center_y - static_cast<int>(length * std::sin(l_angle_rad))  // y는 반대
    );
    
    //****************************************** */

    // 가장 가까운 왼쪽/오른쪽 바운딩박스 저장용
    double min_left_dist = DBL_MAX;
    double min_right_dist = DBL_MAX;
    cv::Point left_target, right_target;

    // 왼쪽 바운딩 박스
    for (int i = 1; i < l_num_labels; ++i) {
        int x = l_stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = l_stats.at<int>(i, cv::CC_STAT_TOP);
        int w = l_stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = l_stats.at<int>(i, cv::CC_STAT_HEIGHT);

        int x_corrected = x;
        int y_corrected = y;    
        cv::rectangle(frame, Rect(x_corrected, y_corrected, w, h), Scalar(0, 255, 255), 1);

        
        // 바운딩박스 중심 좌표
        cv::Point obj_center(x_corrected + w / 2, y_corrected + h / 2);

        // 중심으로부터 거리 계산
        double dist = norm(obj_center - center);

        if (dist < min_left_dist) {
            min_left_dist = dist;
            left_target = obj_center;
            std::cout << "[LEFT] updated target: (" << left_target.x << ", " << left_target.y << "), dist = " << dist << std::endl;
        }
    }   

    //오른쪽 바운딩 박스스
    for (int i = 1; i < r_num_labels; ++i) {
        int x = r_stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = r_stats.at<int>(i, cv::CC_STAT_TOP);
        int w = r_stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = r_stats.at<int>(i, cv::CC_STAT_HEIGHT);

        int x_corrected = x;
        int y_corrected = y;   
        cv::rectangle(frame, Rect(x_corrected + center_x, y_corrected, w, h), Scalar(255,0,0), 1);

        // 바운딩박스 중심 좌표
        cv::Point obj_center(x_corrected + w / 2, y_corrected + h / 2);
        obj_center.x += center_x;

        // 중심으로부터 거리 계산
        double dist = norm(obj_center - center);
        
        if (obj_center.x > center_x && dist> 5.0 && dist < min_right_dist) {
            min_right_dist = dist;
            right_target = obj_center;
            std::cout << "[RIGHT] updated target: (" << right_target.x << ", " << right_target.y << "), dist = " << dist << std::endl;
        }
    }   

    //왼쪽 화살표 각도 계산
    if (min_left_dist < DBL_MAX) {
        Point2f vec_l = left_target - center;

        double angle_rad_l = std::atan2(-vec_l.y, vec_l.x);
        double angle_deg_l = angle_rad_l * 180.0 / CV_PI;
        cv::Point front_l = point_from_angle(center, angle_deg_l, length);
        arrowedLine(frame, center, left_target, Scalar(0,255,255), 1);  // 노란 화살표
    }
    //오른쪽 화살표 각도 계산산
    if (min_right_dist < DBL_MAX) {
        Point2f vec_r = right_target - center;

        double angle_rad_r = std::atan2(-vec_r.y, vec_r.x);
        double angle_deg_r = angle_rad_r * 180.0 / CV_PI;
        cv::Point front_r = point_from_angle(center, angle_deg_r, length);
        arrowedLine(frame, center, right_target, Scalar(255,0,0), 1);  // 파란 화살표
    }
    double angle_deg_front = 0.0;
    bool left_exists = (min_left_dist < DBL_MAX);
    bool right_exists = (min_right_dist < DBL_MAX);

    if (left_exists && right_exists) {
        // 왼쪽과 오른쪽 방향 벡터
        Point2f vec_l = left_target - center;
        Point2f vec_r = right_target - center;

        // 각각의 각도 구하기
        double angle_rad_l = std::atan2(-vec_l.y, vec_l.x);
        double angle_rad_r = std::atan2(-vec_r.y, vec_r.x);

        // 평균 각도 (주의: 원형 평균이므로 atan2 써야 정확하지만 일반 평균도 근사적으로 충분함)
        double angle_rad_avg = (angle_rad_l + angle_rad_r) / 2.0;
        angle_deg_front = angle_rad_avg * 180.0 / CV_PI;

        // 앞쪽 화살표
        Point front_end = point_from_angle(center, angle_deg_front, length);
        arrowedLine(frame, center, front_end, Scalar(255, 0, 255), 1);  // 앞쪽(핑크)
    }

    circle(frame, center, 10, cv::Scalar(0,255,0), -1); // 중심점에 초록 점
    

    imshow("wsl", frame);
    imshow("bin",bin );
    // imshow("bin_left", bin_left);
    // imshow("bin_right", bin_right);
    cv::waitKey(1);
    RCLCPP_INFO(node->get_logger(), "Received Image : %s, %d, %d", msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
