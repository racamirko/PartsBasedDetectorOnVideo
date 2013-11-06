/**
  *     PartsBasedDetectorOnVideo
  *
  *  Huge chunks of code shamelessly taken from Hilton Bristow's demo for
  *  PartsBasedDetector.
  *
  *
  *  File:      main.cpp
  *  Author:    Mirko Raca <name.lastname@epfl.ch>
  *  Created:   November 5, 2013
  *
  */

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <stdio.h>
#include <fstream>
#include <ncurses.h>

#include <opencv2/highgui/highgui.hpp>

#include <PartsBasedDetector.hpp>
#include <Candidate.hpp>
#include <FileStorageModel.hpp>

#define WITH_MATLABIO
#ifdef WITH_MATLABIO
    #include <MatlabIOModel.hpp>
#endif

#include "outputFormat.h"
#include "mirrorUtils.h"

using namespace cv;
using namespace std;

#define OUTPUT_FILENAME_FORMAT "facedetect_frame%06d.txt"
#define DEFAULT_NMS_THRESHOLD 0.3f
#define DEFAULT_MIRRORING true

void setupDisplay(char* _model, char* _inputVideo, char* _outputFolder);
void updateDisplay(int _frame, float _perc);

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    DLOG(INFO) << "Execution started";

    if (argc < 4) {
        printf("Usage: PartsBasedDetectorOnVideo model_file video_file output_folder [nmsThreshold]\n");
        exit(-1);
    }

    // process variables
    boost::scoped_ptr<Model> model;
    float nmsThreshold = DEFAULT_NMS_THRESHOLD;
    bool mirroring = DEFAULT_MIRRORING;

    if( argc >= 5 ){
        nmsThreshold = atof(argv[4]);
    }

    // determine the type of model to read
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
        endwin();
        exit(-4);
    }
    double frameCount = videoSrc.get(CV_CAP_PROP_FRAME_COUNT);
    double frameNo = videoSrc.get(CV_CAP_PROP_POS_FRAMES);
    DLOG(INFO) << "Frame count: " << frameCount;
    DLOG(INFO) << "Start frame no: " << frameNo;

    // display initialzation
    setupDisplay(argv[1], argv[2], argv[3]);

    // main loop
    DLOG(INFO) << "main loop";
    vectorCandidate candidates;
    Mat curFrameIm;
    char outputFilenameBuffer[1024];
    while(frameNo < frameCount){
        DLOG(INFO) << "FrameNo " << frameNo;
        updateDisplay(frameNo, ((float)frameNo/(float)frameCount*100.0f));

        candidates.clear();
        frameNo = videoSrc.get(CV_CAP_PROP_POS_FRAMES);
        videoSrc >> curFrameIm;

        pbd.detect(curFrameIm, depth, candidates);
#ifndef NDEBUG
        gOutputFormat = FT_BBOX_BRIEF;
#endif
        DLOG(INFO) << "Found original: " << candidates;
        if(mirroring){
            vectorCandidate mirroredCandidates;
            flip(curFrameIm, curFrameIm, 1); // flip around y-axis
            pbd.detect(curFrameIm, depth, mirroredCandidates);
            DLOG(INFO) << "Found flipped: " << mirroredCandidates;
            flipHorizontaly(mirroredCandidates, curFrameIm.size);
            DLOG(INFO) << "After flipping: " << mirroredCandidates;
            candidates.insert(candidates.end(), mirroredCandidates.begin(), mirroredCandidates.end());
        }

        Candidate::nonMaximaSuppression(curFrameIm, candidates, nmsThreshold);
        DLOG(INFO) << "Final all detections" << candidates;
        // output
        sprintf(outputFilenameBuffer, outputFilePattern.c_str(), (int) frameNo);
        ofstream outFile(outputFilenameBuffer);
#ifndef NDEBUG
        gOutputFormat = FT_FULL_OUTPUT;
#endif
        outFile << candidates;

        // cleanup
        outFile.close();
        if(!curFrameIm.empty())
            curFrameIm.release();
    }

    // cleanup
    DLOG(INFO) << "Cleanup part";
    videoSrc.release();

    DLOG(INFO) << "Execution finished";
    endwin();
    return 0;
}

void setupDisplay(char* _model, char* _inputVideo, char* _outputFolder){
    char buffer[1000];
    initscr();
    cbreak();
    noecho();

    sprintf(buffer, "Model file: %s", _model);
    mwprintw(3, 3, buffer);

    sprintf(buffer, "Input video file: %s", _inputVideo);
    mwprintw(5, 3, buffer);

    sprintf(buffer, "Output folder: %s", _outputFolder);
    mwprintw(7, 3, buffer);

    // make some kind of progress bar

    refresh();
}

void updateDisplay(int _frame, float _perc){
    // update display with information
    char buffer[1000];
    int rows, cols;
    getmaxyx(stdscr, rows, cols); // will use it later

    move(9, 5);
    addch('[');
    for(int runner = 0; runner < _perc; runner+=10 )
        addch('#');
    for(;runner < 100; runner += 10)
        addch(' ');
    addch('] ');
    sprintf(buffer, " %.2f [%d frame]", _perc, _frame);
    printw(buffer);

    refresh();
    //getch(); // not needed - cbreak and system takes care of the rest
}
