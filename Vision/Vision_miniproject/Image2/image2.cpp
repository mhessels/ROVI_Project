#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

void draw_histogram(cv::Mat&, std::string);

/*
 * 
 * To clean this picture a median filter with kernal size 9 is applied. This is due to the ratio of salt and pepper noise is approx 1.
 * It is assumed the the noise is position independent
 * 
 */


int main()
{
  cv::Mat input_matrix = cv::imread("./Images/Image2.png",CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat output_matrix = input_matrix;
  draw_histogram(input_matrix,"./Histograms/Original_Histogram.png");
  
  cv::medianBlur(input_matrix,output_matrix,9);
  
  cv::imwrite("./Images/MedianBlur_Image.png",output_matrix);
  draw_histogram(output_matrix,"./Histograms/MedianBlur_Histogram.png");
  
  return 0;
}


void draw_histogram(cv::Mat &img, std::string image_name="../ImagesForStudents/out_hist.png"){
  
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
