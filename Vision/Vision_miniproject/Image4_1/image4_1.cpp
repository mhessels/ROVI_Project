#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

template<class ImgT>
void dftshift(ImgT& img);

cv::Mat BLPF(double d0, double n, int wy, int wx, int cx, int cy)
{
  cv::Mat_<cv::Vec2f> hpf(wy, wx);
  for(int y = 0; y < wy; ++y) {
	for(int x = 0; x < wx; ++x) {

	  const double d_pos = std::sqrt( double((x-cx)*(x-cx)) + double((y-cy)*(y-cy)));
	  
	  if(d_pos==0) // Avoid division by zero
		hpf(y,x)[0] = 0;
	  else
		hpf(y,x)[0] = 1.0 / (1.0 + std::pow(d_pos/d0, 2.0*n));
	  
	  hpf(y,x)[1] = 0;
	}
  }
  return hpf;
}

cv::Mat BNF(double n, int wy, int wx, int cx, int cy)
{
  cv::Mat_<cv::Vec2f> hpf(wy, wx);
  for(int y = 0; y < wy; ++y) {
	for(int x = 0; x < wx; ++x) {

	  const double d_pos_1 = std::sqrt( double((x-2140)*(x-2140)) + double((y-1777)*(y-1777)));
	  const double d_neg_1 = std::sqrt( double((x-930)*(x-930)) + double((y-3020)*(y-3020)));
	  const double d_pos_2 = std::sqrt( double((x-2140)*(x-2140)) + double((y-1777)*(y-1777)));
	  const double d_neg_2 = std::sqrt( double((x-1735)*(x-1735)) + double((y-2610)*(y-2610)));

	  hpf(y,x)[0] = 1.0 / (1.0 + std::pow(25/d_pos_1, 2.0*n))*1.0 / (1.0 + std::pow(25/d_neg_1, 2.0*n));
	  hpf(y,x)[0] = 1.0 / (1.0 + std::pow(25/d_pos_2, 2.0*n))*1.0 / (1.0 + std::pow(25/d_neg_2, 2.0*n));
	  hpf(y,x)[1] = 0;
	
	  
	}
  }
  return hpf;
}

cv::Mat BBSF(double n, int wy, int wx, int cx, int cy){
  cv::Mat_<cv::Vec2f> nf(wy, wx);

  for(int y = 0; y < wy; ++y) {
	for(int x = 0; x < wx; ++x) {
	  const double d_pos = std::sqrt( double((x-cx)*(x-cx)) + double((y-cy)*(y-cy)));
	    
	  if(d_pos==0) // Avoid division by zero
		nf(y,x)[0] = 0;
	  else
		nf(y,x)[0] = (1.0 / (1.0 + std::pow(d_pos*25/(std::pow(d_pos,2)-std::pow(283,2)), 2.0*n)))*(1.0 / (1.0 + std::pow(d_pos*25/(std::pow(d_pos,2)-std::pow(862,2)), 2.0*n)));
	}
  }
  return nf;
}
void dftshift(cv::Mat_<float>&);
int main()
{
  // A gray image
  cv::Mat_<float> img = cv::imread("Image4_1.png", CV_LOAD_IMAGE_GRAYSCALE);
  // Load image
  //     cv::Mat_<float> img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
  
  // Get original size
  int wxOrig = img.cols;
  int wyOrig = img.rows;
  
  
  int m = cv::getOptimalDFTSize( 2*wyOrig );
  int n = cv::getOptimalDFTSize( 2*wxOrig );
  
  copyMakeBorder(img, img, 0, m - wyOrig, 0, n - wxOrig, cv::BORDER_CONSTANT, cv::Scalar::all(0));
  
  // Get padded image size
  const int wx = img.cols, wy = img.rows;
  const int cx = wx/2, cy = wy/2;
  
  std::cout << wxOrig << " " << wyOrig << std::endl;
  std::cout << wx << " " << wy << std::endl;
  std::cout << cx << " " << cy << std::endl;
  
  // Compute DFT of image
  cv::Mat_<float> imgs[] = {img.clone(), cv::Mat_<float>::zeros(wy, wx)};
  cv::Mat_<cv::Vec2f> img_dft;
  cv::merge(imgs, 2, img_dft);
  cv::dft(img_dft, img_dft);
  
  // Shift to center
  dftshift(img_dft);
  
  // Used for visualization only
  cv::Mat_<float> magnitude, phase;
  cv::split(img_dft, imgs);
  cv::cartToPolar(imgs[0], imgs[1], magnitude, phase);
  magnitude = magnitude + 1.0f;
  cv::log(magnitude, magnitude);
  cv::normalize(magnitude, magnitude, 0, 1, CV_MINMAX);
  cv::imwrite("img_dft.png", magnitude * 255);
  //             cv::imshow("img_dft", magnitude);
  
  // Create a Butterworth low-pass filter of order n and diameter d0 in the frequency domain
  cv::Mat lpf = BLPF(100, 2, wy, wx, cx, cy);
  cv::Mat bsf = BBSF(2, wy, wx, cx, cy);
  cv::Mat nf = BNF(2, wy, wx, cx, cy);
  // Multiply and shift back
  cv::mulSpectrums(nf, img_dft, img_dft, cv::DFT_ROWS);

  dftshift(img_dft);
  
  //Display high pass filter
  cv::Mat realImg[2];
  cv::split(nf,realImg);
  cv::Mat realNF = realImg[0];
  cv::normalize(realNF, realNF, 0.0, 1.0, CV_MINMAX);
  // 	namedWindow("", cv::WINDOW_NORMAL);
  cv::imwrite("NF.png", realNF);
  
  
  
  //----- Compute IDFT of HPF filtered image
  
  //you can do this
  //cv::idft(img_dft, img_dft); //the result is a 2 channel image
  //Mat output;
  // therefore you split it and get the real one
  //split(img_dft, imgs);
  //normalize(imgs[0], output, 0, 1, CV_MINMAX);
  
  //or you can do like this, then you dont need to split
  cv::Mat_<float> output;
  cv::dft(img_dft, output, cv::DFT_INVERSE| cv::DFT_REAL_OUTPUT);
  cv::Mat_<float> croppedOutput(output,cv::Rect(0,0,wxOrig,wyOrig));
  
  cv::normalize(output, output, 0, 1, CV_MINMAX);
  cv::normalize(img, img, 0.0, 1.0, CV_MINMAX);
  
  // 	namedWindow("", cv::WINDOW_NORMAL);
  //     cv::imshow("Input", img);
  cv::imwrite("out.png", croppedOutput * 255);
  // 	namedWindow("", cv::WINDOW_NORMAL);
  //     cv::imshow("High-pass filtered input", croppedOutput);
  cv::waitKey();
  return 0;
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


template<class ImgT>
void dftshift(ImgT& img) {
  const int cx = img.cols/2;
  const int cy = img.rows/2;
  
  ImgT tmp;
  ImgT topLeft(img, cv::Rect(0, 0, cx, cy));
  ImgT topRight(img, cv::Rect(cx, 0, cx, cy));
  ImgT bottomLeft(img, cv::Rect(0, cy, cx, cy));
  ImgT bottomRight(img, cv::Rect(cx, cy, cx, cy));
  
  topLeft.copyTo(tmp);
  bottomRight.copyTo(topLeft);
  tmp.copyTo(bottomRight);
  
  topRight.copyTo(tmp);
  bottomLeft.copyTo(topRight);
  tmp.copyTo(bottomLeft);
}

