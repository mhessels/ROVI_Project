#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

#define D_0 100
float D(int u, int cols, int v, int u_k, int v_k,int rows)
{
//   std::cout << sqrt(pow(u-cols/2,2)+pow(v-rows/2,2))<<std::endl;
 return sqrt(pow(u-cols/2,2)+pow(v-rows/2,2));//-u_k,2)+pow(v-rows/2-v_k,2));
}

cv::Mat_<float> notchfilter(int cols, int rows){
	//If this does not work try switching u and v, u_k and v_k, rows and cols
  cv::Mat_<float> a(cv::Size(cols,rows),0.0f);
  for(int i = 0;i<rows;i++){
	for(int j = 0;j<cols;j++){
	  //Testing with lowpass filter first
	  a.at<float>(i,j) = 1/(1+pow(D(j, cols,i,-2140,-1740,rows)/D_0,4));
//       a.at<float>(i,j) = 1/(1+pow(D_0*sqrt(D(i, cols,j,-2140,-1740,rows))/D(i, cols,j,-2140,-1740,rows),2*2))*1/(1+pow(D_0*sqrt(D_0*D(i,cols,j,2140,1740,rows))/D(i,cols,j,2140,1740,rows),2*2));
//  	  std::cout << 1/(1+pow(D_0/D(i, cols,j,2140,1740,rows),2*2))*1/(1+pow(D_0/D(i, cols,j,-2140,-1740,rows),2*2))<< std::endl;
	}
  }
  return a;
}

void dftshift(cv::Mat_<float>&);
int main()
{
  // A gray image
  cv::Mat_<float> img = cv::imread("Image4_1.png", CV_LOAD_IMAGE_GRAYSCALE);
//   cv::Mat_<float> img = cv::imread("Image3.png", CV_LOAD_IMAGE_GRAYSCALE);
  
  //Pad the image with borders using copyMakeBorders. Use getOptimalDFTSize(A+B-1). See G&W page 251,252 and 263 and dft tutorial. (Typicly A+B-1 ~ 2A is used)
  int rows = cv::getOptimalDFTSize(2*img.rows);
  int cols = cv::getOptimalDFTSize(2*img.cols);
  int imgRows = img.rows;
  int imgCols = img.cols;
  cv::copyMakeBorder(img,img,0,rows-img.rows,0,cols-img.cols,cv::BORDER_CONSTANT,cv::Scalar(0));
  
  //Copy the gray image into the first channel of a new 2-channel image of type Mat_<Vec2f>, e.g. using merge(), save it in img_dft
  //The second channel should be all zeros.
  cv::Mat_<float> imgs[] = {img.clone(), cv::Mat_<float>(img.rows, img.cols, 0.0f)};
  cv::Mat_<cv::Vec2f> img_dft;
  cv::merge(imgs, 2, img_dft);
  
  // Compute DFT
  cv::dft(img_dft, img_dft);
  
  // Split
  cv::split(img_dft, imgs);
  
  // Compute magnitude/phase
  cv::Mat_<float> magnitude, phase;
  cv::cartToPolar(imgs[0], imgs[1], magnitude, phase);
  
  // Shift quadrants for viewability
  dftshift(magnitude);
  
  // Logarithm of magnitude
  cv::Mat_<float> magnitudel;
  
  // Output image for 
  cv::Mat_<float> imgout;
  
 	// Take logarithm of magnitude
  magnitudel = magnitude + 1.0f;
  cv::log(magnitudel, magnitudel);
  
  cv::Mat_<float> filter = notchfilter(cols,rows);
  cv::mulSpectrums(magnitude,filter,magnitude,cv::DFT_ROWS);
  
  // Show
  cv::normalize(img, img, 0.0, 1.0, CV_MINMAX);
  cv::normalize(magnitudel, magnitudel, 0.0, 1.0, CV_MINMAX);
  cv::normalize(phase, phase, 0.0, 1.0, CV_MINMAX);
//   cv::imshow("Input", img);
//   cv::imshow("Magnitude", magnitudel);

  cv::Mat lol(cv::Size(magnitudel.cols,magnitudel.rows),CV_8UC1);
  
  for(int i = 0;i<magnitudel.rows;i++){
	for(int j = 0;j<magnitudel.cols;j++){
	  lol.at<uchar>(i,j) = magnitudel.at<float>(i,j)*255;
	}
  }
  cv::imwrite("img/Magnitude.png", lol);
  
  
  
  return 0;
}

void dftshift(cv::Mat_<float>& magnitude) {
   const int cx = magnitude.cols/2;
   const int cy = magnitude.rows/2;

   cv::Mat_<float> tmp;
   cv::Mat_<float> topLeft(magnitude, cv::Rect(0, 0, cx, cy));
   cv::Mat_<float> topRight(magnitude, cv::Rect(cx, 0, cx, cy));
   cv::Mat_<float> bottomLeft(magnitude, cv::Rect(0, cy, cx, cy));
   cv::Mat_<float> bottomRight(magnitude, cv::Rect(cx, cy, cx, cy));

   topLeft.copyTo(tmp);
   bottomRight.copyTo(topLeft);
   tmp.copyTo(bottomRight);

   topRight.copyTo(tmp);
   bottomLeft.copyTo(topRight);
   tmp.copyTo(bottomLeft);
}
