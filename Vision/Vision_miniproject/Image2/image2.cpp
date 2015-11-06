#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <vector>
#include <algorithm>
 
cv::Mat image;
cv::Mat resImage(cv::Size(1524,2316),CV_8UC1);
cv::Mat paddedImage(cv::Size(1524,2316),CV_8UC1);
 
int Smax = 10;
int Sinit = 1;
int S = Sinit;
int zMin;
int zMax;
int zMed;
int zxy;
 
int i;
int j;
int row;
int col;
int padding;
 
void runAMF();
void findZs();
void stageA();
void stageB();
 
void draw_histogram(cv::Mat&, std::string);

/*
 * 
 * To clean this picture a median filter with kernal size 9 is applied. This is due to the ratio of salt and pepper noise is approx 1.
 * It is assumed the the noise is position independent
 * 
 */


int main()
{
  image = cv::imread("Images/Image2.png",CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat input_matrix = cv::imread("./Images/Image2.png",CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat output_matrix = input_matrix;
  draw_histogram(input_matrix,"./Histograms/Original_Histogram.png");
  
  cv::medianBlur(input_matrix,output_matrix,9);
 
  padding = 0.05*col;
 
  cv::copyMakeBorder(image, paddedImage, padding, padding, padding, padding, cv::BORDER_CONSTANT, 0);
  row = paddedImage.rows;
  col = paddedImage.cols;
  std::cout << row << ", " << col << std::endl;
 
  runAMF();
  
  cv::imwrite("./Images/MedianBlur_Image.png",output_matrix);
  cv::imwrite("Images/adaptive.png", resImage);
  draw_histogram(output_matrix,"./Histograms/MedianBlur_Histogram.png");
  draw_histogram(resImage,"./Histograms/Adaptive_Histogram.png");
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



void runAMF(){
        for(i = padding; i < row; i++){
                for(j = padding; j < col; j++){
// 				  std::cout<< "Alive" << std::endl;
                        S = Sinit;
                         stageA();
                }
        }
}
 
void findZs(){
         std::vector<int> vector;
         int vectorSize;
  
          zxy = paddedImage.at<uchar>(i, j);
  
         for(int sizeH = 1; sizeH < S + 1; sizeH++){
                 vector.push_back(paddedImage.at<uchar>(i + sizeH, j));
                 vector.push_back(paddedImage.at<uchar>(i - sizeH, j));
                 vector.push_back(paddedImage.at<uchar>(i, j + sizeH));
                 vector.push_back(paddedImage.at<uchar>(i, j - sizeH));
                 for(int sizeW = 1; sizeW < S + 1; sizeW++){
                         vector.push_back(paddedImage.at<uchar>(i + sizeH, j + sizeW));
                         vector.push_back(paddedImage.at<uchar>(i + sizeH, j - sizeW));
                         vector.push_back(paddedImage.at<uchar>(i - sizeH, j - sizeW));
                         vector.push_back(paddedImage.at<uchar>(i - sizeH, j + sizeW));
                 }
         }
         vector.push_back(paddedImage.at<uchar>(i, j));
  
         vectorSize = vector.size();
         sort(vector.begin(), vector.end());
         zMin = vector[0];
         zMax = vector[vector.size()-1];
 
         if(vectorSize % 2 == 0){
                 zMed = (vector[vectorSize / 2 - 1] + vector[vectorSize / 2]) / 2;
         }
         else{
                 zMed = vector[vectorSize / 2];
         }
// 		 std::cout << vectorSize << std::endl;
        vector.erase(vector.begin(), vector.end());
}
 
void stageA(){
        findZs();
        int A1 = zMed - zMin;
        int A2 = zMed - zMax;
        if(A1 > 0 && A2 < 0){
                stageB();
        }
        else{
                S = S + 1;
                if(S <= Smax){
                        stageA();
                }
                else{
                        resImage.at<uchar>(i, j) = zMed;
                        //image.at<uchar>(i, j) = zMed;
                }
        }
}
 
void stageB(){
        int B1 = zxy - zMin;
        int B2 = zxy - zMax;
        if(B1 > 0 && B2 < 0){
                resImage.at<uchar>(i, j) = zxy;
                //image.at<uchar>(i, j) = zxy;
        }
        else{
                resImage.at<uchar>(i, j) = zMed;
                //image.at<uchar>(i, j) = zMed;
        }
}