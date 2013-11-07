/*
 *     PartsBasedDetectorOnVideo
 *
 *
 *  Huge chunks of code shamelessly taken from Hilton Bristow's demo for
 *  PartsBasedDetector.
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Chili lab, EPFL.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Chili, EPFL nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  File:    main.h
 *  Author:  Mirko Raca <name.lastname@epfl.ch>
 *  Created: November 5, 2013
 */

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <time.h>
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
#define DEFAULT_RESUME false

void setupDisplay(char* _model, char* _inputVideo, char* _outputFolder);
void updateDisplay(int _frame, float _perc, double _time);

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    DLOG(INFO) << "Execution started";

    if (argc < 4) {
        printf("Usage: PartsBasedDetectorOnVideo model_file video_file output_folder [-r] [nmsThreshold]\n");
        exit(-1);
    }

    // process variables
    boost::scoped_ptr<Model> model;
    float nmsThreshold = DEFAULT_NMS_THRESHOLD;
    bool optMirroring = DEFAULT_MIRRORING;
    bool optResume = DEFAULT_RESUME;

    if( argc >= 5 ){
        if(strcmp(argv[4], "-r") == 0){
            optResume = true;
            DLOG(INFO) << "Resume flag: ON";
        }
    }

    if( argc >= 6 ){
        nmsThreshold = atof(argv[5]);
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

    // DEBUG
//    frameCount = 100;

    // display initialzation
    setupDisplay(argv[1], argv[2], argv[3]);

    // main loop
    DLOG(INFO) << "main loop";
    vectorCandidate candidates;
    Mat curFrameIm;
    char outputFilenameBuffer[1024];
    clock_t timeElapsed = clock();
    while(frameNo < frameCount){
        DLOG(INFO) << "FrameNo " << frameNo;
        updateDisplay(frameNo, ((float)frameNo/(float)frameCount*100.0f), (double) ( clock() - timeElapsed )/CLOCKS_PER_SEC );
        timeElapsed = clock();

        candidates.clear();
        frameNo = videoSrc.get(CV_CAP_PROP_POS_FRAMES);
        videoSrc >> curFrameIm;
        sprintf(outputFilenameBuffer, outputFilePattern.c_str(), (int) frameNo);

        if( optResume ){
            if(boost::filesystem::exists(outputFilenameBuffer))
                continue;
        }

        pbd.detect(curFrameIm, depth, candidates);
#ifndef NDEBUG
        gOutputFormat = FT_BBOX_BRIEF;
#endif
        DLOG(INFO) << "Found original: " << candidates;
        if(optMirroring){
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
    initscr();
    cbreak();
    noecho();
    int rows, cols;
    getmaxyx(stdscr, rows, cols); // will use it later

    attron(A_BOLD);
    mvprintw(1, cols/2-19, "[[ PartsBasedDetector (onVideo) v1.0 ]]");
    attroff(A_BOLD);

    mvprintw(3, 3, "Model file: ");
    mvprintw(3, 25, boost::filesystem::path(_model).filename().c_str());

    mvprintw(4, 3, "Input video file: ");
    mvprintw(4, 25, boost::filesystem::path(_inputVideo).filename().c_str());

    mvprintw(5, 3, "Output folder: ");
    mvprintw(5, 25, boost::filesystem::path(_outputFolder).leaf().c_str());

    refresh();
}

void updateDisplay(int _frame, float _perc, double _time){
    // update display with information
    int rows, cols;
    getmaxyx(stdscr, rows, cols); // will use it later
    float runnerStep = 100.0f/((float)cols-40);

    move(10, 5);
    addch('[');
    attron(A_BOLD);
    float runner; int change = 0;
    for(runner = 0; runner < _perc; runner+=runnerStep ){
        switch( change % 4 ){
            case 0:
                addch('.');
                break;
            case 1:
                addch('o');
                break;
            case 2:
                addch('O');
                break;
            case 3:
                addch('o');
                break;
        }
        change++;
    }
    for(;runner < 100.0f; runner += runnerStep)
        addch(' ');
    attroff(A_BOLD);
    printw("] %.2f% [Frame #%d]", _perc, _frame);
    move(11, cols/2 - 3);
    printw("TPF: %.2f sec", _time);

    refresh();
}
