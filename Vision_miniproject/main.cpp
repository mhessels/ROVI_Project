#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

void draw_histogram(cv::Mat &img, std::string image_name="../ImagesForStudents/out_hist.bmp"){

int histSize = 255;
float range[] = {0,256};
const float* histRange = {range};
bool uniform = true; bool accumulate = false;
cv::Mat hist = cv::Mat::zeros(512,400,CV_8UC1);
calcHist( &img, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );

int hist_w = 512; int hist_h = 400;
int bin_w = cvRound( (double) hist_w/histSize );

cv::Mat histImage = cv::Mat::zeros( hist_h, hist_w, CV_8UC1);

cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

/// Draw for each channel
for( int i = 1; i < histSize; i++ )
{
    line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
                     cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
                     cv::Scalar( 255, 0, 0), 2, 8, 0  );
}

  cv::imwrite(image_name,histImage);

}

int main()
{
    cv::Mat img1 = cv::imread("../ImagesForStudents/Image4_2.png",CV_LOAD_IMAGE_GRAYSCALE);
    cv::Mat img1_dest;
    cv::medianBlur(img1,img1_dest,3);

    cv::imwrite("../ImagesForStudents/out.png",img1_dest);
    draw_histogram(img1_dest);
    draw_histogram(img1,"../ImagesForStudents/hist_original.png");

    cv::Mat equalhist;
    cv::equalizeHist(img1_dest,equalhist);

    cv::imwrite("../ImagesForStudents/out_hist_eq.png",equalhist);
    draw_histogram(equalhist,"../ImagesForStudents/hist_eq.png");

    return 0;
}
