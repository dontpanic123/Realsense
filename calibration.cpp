#include<opencv2/core.hpp>
#include<opencv2/ccalib.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/ccalib/randpattern.hpp>
#include "opencv2/ccalib/multicalib.hpp"
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
    
    Mat pattern;
    int width = 1080;
    int height = 720;
    cv::randpattern::RandomPatternGenerator generator(width, height);
    generator.generatePattern();
    pattern = generator.getPattern();
    namedWindow("figure", WINDOW_AUTOSIZE);
    imshow("figure", pattern);
    /*imwrite("ramdom_pattern.jpg",pattern); */  
 

    float patternWidth = 27;
    float patternHeight = 18;
    int nMiniMatches = 2;
    int nCamera = 2,  cameraType = 0;
    int showFeatureExtraction = 0, verbose = 0;//verbose: additional info
   // const std::string& inputFilename = "imagelist.xml";
    const string& inputFilename = "imagelist2.xml";
    const string& outputFilename = "multi-camera-results2.xml";
   // vector<string> imagelist;
   // readStringList(inputFilename, imagelist);
    multicalib::MultiCameraCalibration multiCalib(cameraType,
        nCamera, inputFilename, patternWidth, patternHeight, verbose, 
        showFeatureExtraction, nMiniMatches );
    multiCalib.loadImages();
    multiCalib.initialize();
    multiCalib.optimizeExtrinsics();
    multiCalib.writeParameters(outputFilename);

    waitKey(0);
   
 }
 
