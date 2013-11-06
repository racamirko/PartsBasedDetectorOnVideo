/**
  *     PartsBasedDetectorOnVideo
  *
  *  Huge chunks of code shamelessly taken from Hilton Bristow's demo for
  *  PartsBasedDetector.
  *
  *
  *  File:      main.cpp
  *  Author:    Mirko Raca
  *  Created:   November 5, 2013
  *
  */

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <stdio.h>

#include <opencv2/highgui/highgui.hpp>

#include <PartsBasedDetector.hpp>
#include <Candidate.hpp>
#include <FileStorageModel.hpp>

#define WITH_MATLABIO
#ifdef WITH_MATLABIO
    #include <MatlabIOModel.hpp>
#endif

using namespace cv;
using namespace std;

#define OUTPUT_FILENAME_FORMAT "detection_frame%06d.txt"
#define DEFAULT_NMS_THRESHOLD 0.3f

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    DLOG(INFO) << "Execution started";

    if (argc < 4) {
        printf("Usage: PartsBasedDetectorOnVideo model_file video_file output_folder [nmsThreshold]\n");
        exit(-1);
    }

    // determine the type of model to read
    boost::scoped_ptr<Model> model;
    float nmsThreshold = DEFAULT_NMS_THRESHOLD;
    if( argc >= 5 ){
        nmsThreshold = atof(argv[4]);
    }

    string ext = boost::filesystem::path(argv[1]).extension().string();
    if (ext.compare(".xml") == 0 || ext.compare(".yaml") == 0) {
        model.reset(new FileStorageModel);
    }
#ifdef WITH_MATLABIO
    else if (ext.compare(".mat") == 0) {
        model.reset(new MatlabIOModel);
    }
#endif
    else {
        printf("Unsupported model format: %s\n", ext.c_str());
        LOG(FATAL) << "Unsupported model format: " << ext.c_str();
        exit(-2);
    }
    bool ok = model->deserialize(argv[1]);
    if (!ok) {
        printf("Error deserializing file\n");
        LOG(FATAL) << "Error deserializing file.";
        exit(-3);
    }

    // TODO: check
    // check output folder
    string outputFilePattern = (string) argv[3];
    if( outputFilePattern[outputFilePattern.length()-1] != '/' ){
        outputFilePattern.append("/");
    }
    outputFilePattern.append(OUTPUT_FILENAME_FORMAT);

    // create the PartsBasedDetector and distribute the model parameters
    Mat_<float> depth; // we don't have one for the video, so it's just a dummy variable
    PartsBasedDetector<float> pbd;
    pbd.distributeModel(*model);

    // load video sequence
    VideoCapture videoSrc((string)argv[2]);
    if( !videoSrc.isOpened() ){
        printf("Could not read video file\n");
        LOG(FATAL) << "Could not read video file: " << argv[2];
        exit(-4);
    }
    double frameCount = videoSrc.get(CV_CAP_PROP_FRAME_COUNT);
    double frameNo = videoSrc.get(CV_CAP_PROP_POS_FRAMES);
    DLOG(INFO) << "Frame count: " << frameCount;
    DLOG(INFO) << "Start frame no: " << frameNo;

    // main loop
    DLOG(INFO) << "main loop";
    vectorCandidate candidates;
    Mat curFrameIm;
    char outputFilenameBuffer[1024];
    while(frameNo < frameCount){
        candidates.clear();
        videoSrc >> curFrameIm;
        frameNo = videoSrc.get(CV_CAP_PROP_POS_FRAMES);
        pbd.detect(curFrameIm, depth, candidates);
        Candidate::nonMaximaSuppression(curFrameIm, candidates, nmsThreshold);

        sprintf(outputFilenameBuffer, outputFilePattern.c_str(), frameNo);
        // TODO: output part here
        // cleanup
        if(!curFrameIm.empty())
            curFrameIm.release();
    }

    // cleanup
    DLOG(INFO) << "Cleanup part";
    videoSrc.release();

    DLOG(INFO) << "Execution finished";
    return 0;
}
