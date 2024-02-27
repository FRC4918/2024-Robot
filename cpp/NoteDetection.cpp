
/*#include <iostream>
#include <opencv2/opencv.hpp>

// Specify the camera index (usually 0 for built-in webcam)
const int CAMERA_INDEX = 0;
// Define lower and upper bounds for orange color in HSV
cv::Scalar LOWER_ORANGE_HSV = cv::Scalar(3, 80, 80);
cv::Scalar UPPER_ORANGE_HSV = cv::Scalar(6, 255, 255);
// The minimum contour area to detect a note
const double MINIMUM_CONTOUR_AREA = 400;
// The threshold for a contour to be considered a disk
const double CONTOUR_DISK_THRESHOLD = 0.9;

cv::RotatedRect findLargestOrangeContour(const cv::Mat& hsvImage) {
    // Threshold the HSV image to get only orange colors
    cv::Mat mask;
    cv::inRange(hsvImage, LOWER_ORANGE_HSV, UPPER_ORANGE_HSV, mask);
    // Find contours in the mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Find the largest contour
    double maxArea = 0;
    int maxAreaIdx = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > maxArea) {
            maxArea = area;
            maxAreaIdx = i;
        }
    }

    if (maxAreaIdx >= 0)
        return cv::minAreaRect(contours[maxAreaIdx]);
    else
        return cv::RotatedRect();
}

bool contourIsNote(const cv::RotatedRect& contour) {
    // Get the contour points
    std::vector<cv::Point2f> contourPoints(4);
    contour.points(contourPoints.data());

    // Make sure the contour isn't some random small spec of noise
    if (contour.size.area() < MINIMUM_CONTOUR_AREA)
        return false;

    // Get the smallest convex polygon that can fit around the contour
    std::vector<cv::Point2f> hull;
    cv::convexHull(contourPoints, hull);
    // Fits an ellipse to the hull, and gets its area
    cv::RotatedRect ellipse = cv::fitEllipse(hull);
    double bestFitEllipseArea = CV_PI * (ellipse.size.width / 2) * (ellipse.size.height / 2);
    // Returns True if the hull is almost as big as the ellipse
    return contour.size.area() / bestFitEllipseArea > CONTOUR_DISK_THRESHOLD;
}


int main() {
    // Open the camera
    cv::VideoCapture cap(CAMERA_INDEX);
    if (!cap.isOpened()) {
        std::cerr << "Error: Unable to open camera" << std::endl;
        return -1;
    }

    while (true) {
        // Capture frame-by-frame
        cv::Mat frame;
        cap.read(frame);
        if (frame.empty()) {
            std::cerr << "Error: Unable to capture frame" << std::endl;
            break;
        }

        // Converts from BGR to HSV
        cv::Mat frameHsv;
        cv::cvtColor(frame, frameHsv, cv::COLOR_BGR2HSV);
        cv::RotatedRect contour = findLargestOrangeContour(frameHsv);
        if (contour.size.area() > 0 && contourIsNote(contour)) {
            cv::ellipse(frame, contour, cv::Scalar(255, 0, 255), 2);
            // Update SmartDashboard
            // smart_dashboard.putBoolean("Can See Note", true);
        }
        // else {
        //     // Update SmartDashboard
        //     // smart_dashboard.putBoolean("Can See Note", false);
        // }

        cv::imshow("Frame", frame);

        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Release the capture
    cap.release();
    cv::destroyAllWindows();

    return 0;
}*/