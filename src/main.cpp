/**
  *     PartsBasedDetectorOnVideo
  *
  *  Huge chuncks of code shamelessly taken from Hilton Bristow's demo for
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

#include <opencv2/highgui/highgui.hpp>

#include "PartsBasedDetector.hpp"
#include "Candidate.hpp"
#include "FileStorageModel.hpp"

#define WITH_MATLABIO
#ifdef WITH_MATLABIO
    #include "MatlabIOModel.hpp"
#endif

using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    DLOG(INFO) << "Execution started";
    if (argc != 5) {
        printf("Usage: PartsBasedDetectorOnVideo model_file video_file output_folder\n");
        exit(-1);
    }

    // determine the type of model to read
    boost::scoped_ptr<Model> model;
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

    // main loop
    // detect potential candidates in the image
    DLOG(INFO) << "main loop";
    vector<Candidate> candidates;
    Mat curFrameIm;
    while(1){
        candidates.clear();
        videoSrc >> curFrameIm;
        pbd.detect(curFrameIm, depth, candidates);
    }

    // cleanup
    DLOG(INFO) << "Cleanup part";
    videoSrc.release();

    DLOG(INFO) << "Execution finished";
    return 0;
}
