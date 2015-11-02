#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
void dftshift(cv::Mat_<float>& magnitude);
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

void shifted_alphatrim(cv::Mat input_matrix,cv::Mat output_matrix, int shift){ // ADD KERNEKL SIZE AND SHIFT
std::vector<int> vectorMean;
const int k_size = 9;
int rows = input_matrix.rows;
int cols = input_matrix.cols;
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
}


void descreteft(const std::string& filename){
    bool highpass = false;
    // A gray image
    cv::Mat_<float> img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

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

   // Output image for HPF
   cv::Mat_<float> imgout;

   if(highpass) {
      // High-pass filter: remove the low frequency parts in the middle of the spectrum
      const int sizef = 50;
      magnitude(cv::Rect(magnitude.cols/2-sizef/2, magnitude.rows/2-sizef/2, sizef, sizef)) = 0.0f;

      // Take logarithm of modified magnitude
      magnitudel = magnitude + 1.0f;
      cv::log(magnitudel, magnitudel);

      // Shift back quadrants of the spectrum
      dftshift(magnitude);

      // Compute complex DFT output from magnitude/phase
      cv::polarToCart(magnitude, phase, imgs[0], imgs[1]);

      // Merge DFT into one image and restore
      cv::merge(imgs, 2, img_dft);
      cv::dft(img_dft, imgout, cv::DFT_INVERSE + cv::DFT_SCALE + cv::DFT_REAL_OUTPUT);

      //Cut away the borders
      imgout = imgout(cv::Rect(0,0,imgCols,imgRows));
   } else {
      // Take logarithm of magnitude
      magnitudel = magnitude + 1.0f;
      cv::log(magnitudel, magnitudel);
   }


   // Show
   cv::normalize(img, img, 0.0, 1.0, CV_MINMAX);
   cv::normalize(magnitudel, magnitudel, 0.0, 1.0, CV_MINMAX);
   cv::normalize(phase, phase, 0.0, 1.0, CV_MINMAX);
   cv::imshow("Input", img);
   cv::imshow("Magnitude", magnitudel);
   if(highpass) {
      cv::normalize(imgout, imgout, 0.0, 1.0, CV_MINMAX);
      cv::imshow("Output", imgout);
   }
 cv::Mat lol(cv::Size(magnitudel.cols,magnitudel.rows),CV_8UC1);

   for(int i = 0;i<magnitudel.rows;i++){
       for(int j = 0;j<magnitudel.cols;j++){
           lol.at<char>(i,j) = magnitudel.at<float>(i,j)*255;
       }
   }
   cv::imwrite("../ImagesForStudents/Magnitude.png", lol);
   cv::waitKey();

  
}

int main()
{

    //draw_histogram(img1,"../ImagesForStudents/hist_original.png");

    //  shifted_alphatrim(img1,img1_dest,2);   //IMAGE 1
    //  cv::medianBlur(img1,output,9); //IMAGE 2
	// IMAGE 3 (uniform noise)
    descreteft("../ImagesForStudents/Image4_1.png");           //IMAGE 4_1 (use notch filter)

    //cv::imwrite("../ImagesForStudents/out.png",output);
    //draw_histogram(output);
/*

    cv::Mat equalhist;
    cv::equalizeHist(img1_dest,equalhist);

    cv::imwrite("../ImagesForStudents/out_hist_eq.png",equalhist);
    draw_histogram(equalhist,"../ImagesForStudents/hist_eq.png");
    */
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
