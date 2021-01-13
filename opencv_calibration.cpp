#include<opencv2/core.hpp>
#include<opencv2/ccalib.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/ccalib/randpattern.hpp>
#include<opencv2/ccalib/multicalib.hpp>
#include <opencv2/features2d.hpp>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;


int main(int argc, char** argv)
{
    // read and show a photo
   /* Mat img;
    img = imread("building.jpg");
    namedWindow("figure", WINDOW_AUTOSIZE);
    imshow("figure", img);
    waitKey(0);*/



// print random photo
    /*
    Mat pattern;
    int width =4800 ;
    int height = 3600;
    cv::randpattern::RandomPatternGenerator generator(width, height);
    generator.generatePattern();
    pattern = generator.getPattern();
    namedWindow("figure", WINDOW_AUTOSIZE);
    imshow("figure", pattern);
    imwrite("ramdom_pattern.jpg",pattern);  
 */

    double start_time=static_cast<double>(getTickCount());
    float patternWidth = 0.76;// unit: m (0.41 or 0.76)
    float patternHeight = 0.57;// unit: m (0.29 or 0.57)
    int nMiniMatches = 15;
    int nCamera = 2,  cameraType = 0;
    int showFeatureExtraction = 0, verbose = 1;//verbose: additional info
   // const std::string& inputFilename = "imagelist.xml";
    const string& inputFilename = "imagelist.yaml";
    const string& outputFilename = "multi-camera-results.xml";

    multicalib::MultiCameraCalibration multiCalib(cameraType,
        nCamera, inputFilename, patternWidth, patternHeight, verbose, 
        showFeatureExtraction, nMiniMatches );
    multiCalib.loadImages();
    multiCalib.initialize();
    multiCalib.optimizeExtrinsics();
    multiCalib.writeParameters(outputFilename);
    double end_time=static_cast<double>(getTickCount());
    double c_time=(end_time-start_time)/getTickFrequency();
    cout<<"programm time:"<<c_time<<endl;
    waitKey(0);
   
 }
 
