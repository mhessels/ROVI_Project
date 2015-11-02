#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>



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
