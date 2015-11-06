#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

void draw_histogram(cv::Mat&, std::string);
void local_noise_reduction(cv::Mat&, cv::Mat,double);
std::pair<double,double> make_statistic(std::vector<double>);

/*
 * 
 * To clean this picture a median filter with kernal size 9 is applied. This is due to the ratio of salt and pepper noise is approx 1.
 * It is assumed the the noise is position independent
 * 
 */


int main()
{
  cv::Mat input_matrix = cv::imread("./Images/Image3.png",CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat output_matrix = input_matrix;
  cv::Mat noise_box = input_matrix(cv::Rect(1000,1500,300,200));
  draw_histogram(noise_box,"./Histograms/Original_Noise_Histogram.png");
  
  int cols = noise_box.cols;
  int rows = noise_box.rows;
  std::vector<double> data;
  std::pair<double,double> noise_parameters;
  for(int i = 0;i<rows;i++){
	for(int j = 0; j < cols ; j++){
	  data.push_back (noise_box.at<uchar>(i, j));
	}
  }
  noise_parameters = make_statistic(data);
  std::cout << noise_parameters.second << std::endl;
  local_noise_reduction(input_matrix, output_matrix,noise_parameters.second);
  cv::imwrite("./Images/Adaptive_image.png",output_matrix);
  noise_box = output_matrix(cv::Rect(1000,1500,300,200));
  draw_histogram(noise_box,"./Histograms/Adaptive_Noise_Histogram.png");
  
  
  cv::boxFilter(input_matrix,output_matrix,-1,cv::Size(3,3));
  cv::imwrite("./Images/LinearBlur_image.png",output_matrix);
  noise_box = output_matrix(cv::Rect(1000,1500,300,200));
  draw_histogram(noise_box,"./Histograms/LinearBlur_Noise_Histogram.png");
  
  cv::GaussianBlur(input_matrix,output_matrix,cv::Size(3,3),6.2);
  cv::imwrite("./Images/GaussianBlur_image.png",output_matrix);
  noise_box = output_matrix(cv::Rect(1000,1500,300,200));
  draw_histogram(noise_box,"./Histograms/GaussianBlur_Noise_Histogram.png");
  
  cv::medianBlur(input_matrix,output_matrix,9);
  noise_box = output_matrix(cv::Rect(1000,1500,300,200)); 
  draw_histogram(noise_box,"./Histograms/MedianBlur_Noise_Histogram.png");
  cv::imwrite("./Images/MedianBlur_image.png",output_matrix);
  
  cv::boxFilter(input_matrix,output_matrix,-1,cv::Size(3,3));
  cv::GaussianBlur(output_matrix,output_matrix,cv::Size(3,3),6.2);
  cv::medianBlur(output_matrix,output_matrix,9);
  noise_box = output_matrix(cv::Rect(1000,1500,300,200));
  draw_histogram(noise_box,"./Histograms/Combined_Noise_Histogram.png");
  cv::imwrite("./Images/Combined_image.png",output_matrix);
  
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
  
  cv::Mat histImage = cv::Mat::zeros( hist_h, hist_w+10, CV_8UC1);
  
  cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  
  /// Draw for each channel
  for( int i = 11; i < histSize; i++ )
  {
	line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
		  cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
		  cv::Scalar( 255, 0, 0), 2, 8, 0  );
  }
  
  cv::imwrite(image_name,histImage);
  
}

std::pair<double,double> make_statistic(std::vector<double> data){
  std::pair<double,double> mean_var;
  double mean = 0;
  double var = 0;
  for(int i = 0; i < data.size();i++){
	mean += data[i];
  }
  mean/=data.size();
  for(int i = 0; i < data.size();i++){
	var += pow(data[i]-mean,2);
  }
  var/=(data.size() + 1);
  mean_var = std::make_pair<double,double>(mean,var);
  
  return mean_var;
};

void local_noise_reduction(cv::Mat& input_matrix, cv::Mat output_matrix, double standard_dev){
  int rows = input_matrix.rows;
  int cols = input_matrix.cols;
  
  std::vector<double> mask_values;
  std::pair<double,double> mean_var;
  
  for(int i = 1; i < rows - 1; i++){
	for(int j = 1; j < cols - 1; j++){
	  double mat_val = input_matrix.at<uchar>(i,j);
	  double out_val = 0;
	  mask_values.push_back (input_matrix.at<uchar>(i - 1, j + 1));
	  mask_values.push_back (input_matrix.at<uchar>(i, j + 1));
	  mask_values.push_back (input_matrix.at<uchar>(i + 1, j + 1));
	  mask_values.push_back (input_matrix.at<uchar>(i, j - 1));
	  mask_values.push_back (input_matrix.at<uchar>(i, j));
	  mask_values.push_back (input_matrix.at<uchar>(i, j + 1));
	  mask_values.push_back (input_matrix.at<uchar>(i - 1, j - 1));
	  mask_values.push_back (input_matrix.at<uchar>(i, j - 1));
	  mask_values.push_back (input_matrix.at<uchar>(i + 1, j - 1));
	  
	  mean_var = make_statistic(mask_values);
	  
	  mask_values.erase(mask_values.begin(),mask_values.end());
// 	  std::cout << mean_var.first << "\t" << mean_var.second << std::endl;
	  
	  
	  if(mean_var.second > 0){
	  out_val = cv::saturate_cast<uchar>(mat_val- standard_dev/mean_var.second*(mat_val - mean_var.first))	;
	  }else{
		out_val = input_matrix.at<uchar>(i, j);
	  }
	  output_matrix.at<uchar>(i,j) = out_val;
	}
  }
  
}