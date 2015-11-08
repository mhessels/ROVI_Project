#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

void draw_histogram(cv::Mat&, std::string);

int main()
{
  cv::Mat input_matrix = cv::imread("./Images/Image1.png",CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat output_matrix = input_matrix;
  draw_histogram(input_matrix,"./Histograms/Original_Histogram.png");
  
  const int shift = 2;                                        //Weight parameter
  const int k_size = 9;                                       // Kernal size
  
  int rows = input_matrix.rows;
  int cols = input_matrix.cols;
  int count=0;
  for(int i = 0;i<rows;i++){
	for(int j = 0;j<cols;j++){
	  if(input_matrix.at<uchar>(i, j) == 0)
		count++;
	}
  }
  std::cout << count << " " << rows*cols << " " << count/(double(rows)*cols)<< std::endl;
  std::vector<int> vectorMean;
  
  for(int i = 1; i < rows - 1; i++){
	for(int j = 1; j < cols - 1; j++){
	  vectorMean.push_back (input_matrix.at<uchar>(i - 1, j + 1));
	  vectorMean.push_back (input_matrix.at<uchar>(i, j + 1));
	  vectorMean.push_back (input_matrix.at<uchar>(i + 1, j + 1));
	  vectorMean.push_back (input_matrix.at<uchar>(i, j - 1));
	  vectorMean.push_back (input_matrix.at<uchar>(i, j));
	  vectorMean.push_back (input_matrix.at<uchar>(i, j + 1));
	  vectorMean.push_back (input_matrix.at<uchar>(i - 1, j - 1));
	  vectorMean.push_back (input_matrix.at<uchar>(i, j - 1));
	  vectorMean.push_back (input_matrix.at<uchar>(i + 1, j - 1));
	  
	  std::sort(vectorMean.begin(),vectorMean.end());
	  output_matrix.at<uchar>(i, j) = (vectorMean[k_size/2-2+shift] + vectorMean[k_size/2-1+shift]+ vectorMean[k_size/2+shift] + vectorMean[k_size/2+1+shift]+vectorMean[k_size/2+2+shift])/5;
	  vectorMean.erase(vectorMean.begin(),vectorMean.end());
	}
  }
  cv::imwrite("./Images/Alpha_Image.png",output_matrix);
  draw_histogram(output_matrix,"./Histograms/Alpha_Histogram.png");
  
  cv::equalizeHist(output_matrix,output_matrix);
  cv::imwrite("./Images/Alpha&HistEq_Image.png",output_matrix);
  draw_histogram(output_matrix,"./Histograms/Alpha&HistEq_Histogram.png");
  
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
  
  cv::Mat histImage( hist_h, hist_w, CV_8UC1,cv::Scalar(255));
  
  cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  
  /// Draw for each channel
  for( int i = 1; i < histSize; i++ )
  {
	line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
		  cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
		  cv::Scalar( 0, 0, 0), 2, 8, 0  );
  }
  
  cv::imwrite(image_name,histImage);
  
}
