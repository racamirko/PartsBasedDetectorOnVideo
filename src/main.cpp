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

#include "globalIncludes.h"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <ncurses.h>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include <PartsBasedDetector.hpp>
#include <Candidate.hpp>
#include <FileStorageModel.hpp>

#include "filters/GenericFilter.h"
#include "filters/FilterSize.h"
#include "filters/FilterNMS.h"

#define WITH_MATLABIO
#ifdef WITH_MATLABIO
    #include <MatlabIOModel.hpp>
#endif

#include "outputFormat.h"
#include "mirrorUtils.h"

using namespace cv;
using namespace std;
namespace po = boost::program_options;

void parseArguments( int _argc, char* _argv[], // input arguments
                     std::string* _modelFile, std::string* _outputFolder, std::string* _videoFile, // output arguments
                     float* _nmsThreshold, bool* _optMirroring, bool* _optResume);

#define OUTPUT_FILENAME_FORMAT "facedetect_frame%06d.txt"
#define DEFAULT_NMS_THRESHOLD 0.3f
#define DEFAULT_MIRRORING false
#define DEFAULT_RESUME false

#ifdef NDEBUG
    void setupDisplay(char* _model, char* _inputVideo, char* _outputFolder);
    void updateDisplay(int _frame, float _perc, double _time);
    #define FRAME_LIMIT frameCount
#else
    void updateDisplayDebug(int _frame, float _perc, double _time);
    void setupDisplayDebug(char* _model, char* _inputVideo, char* _outputFolder);
    #define FRAME_LIMIT 200
#endif

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    DLOG(INFO) << "Execution started";

    // process variables
    float nmsThreshold = DEFAULT_NMS_THRESHOLD;
    bool optMirroring = DEFAULT_MIRRORING;
    bool optResume = DEFAULT_RESUME;
    string outputFolder = "";
    string modelFile = "";
    string videoFile = "";

    // general variables
    boost::scoped_ptr<Model> model;

    parseArguments(argc, argv, &modelFile, &outputFolder, &videoFile, &nmsThreshold, &optMirroring, &optResume);

    // determine the type of model to read
    string ext = boost::filesystem::path(modelFile).extension().string();
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
    bool ok = model->deserialize(modelFile);
    if (!ok) {
        printf("Error deserializing file\n");
        LOG(FATAL) << "Error deserializing file.";
        exit(-3);
    }

    // check output folder
    if( outputFolder[outputFolder.length()-1] != '/' ){
        outputFolder.append("/");
    }
    outputFolder.append(OUTPUT_FILENAME_FORMAT);

    // create the PartsBasedDetector and distribute the model parameters
    Mat_<float> depth; // we don't have one for the video, so it's just a dummy variable
    PartsBasedDetector<float> pbd;
    pbd.distributeModel(*model);
    std::vector<GenericFilter*> postFilters;
    postFilters.push_back(new FilterSize(Size2f(140,140))); // TODO: exclude the hard-coding for a parameter
    postFilters.push_back(new FilterNMS(nmsThreshold));

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
#ifdef NDEBUG
    setupDisplay(argv[1], argv[2], argv[3]); // release
#else
    setupDisplayDebug(argv[1], argv[2], argv[3]); // debug
#endif
    // main loop
    DLOG(INFO) << "main loop";
    vectorCandidate candidates;
    Mat curFrameIm;
    char outputFilenameBuffer[1024];
    clock_t timeElapsed = clock();
    while(frameNo < FRAME_LIMIT){
        DLOG(INFO) << "FrameNo " << frameNo;
#ifdef NDEBUG
        updateDisplay(frameNo, ((float)frameNo/(float)frameCount*100.0f), (double) ( clock() - timeElapsed )/CLOCKS_PER_SEC );
#else
        updateDisplayDebug(frameNo, ((float)frameNo/(float)frameCount*100.0f), (double) ( clock() - timeElapsed )/CLOCKS_PER_SEC );
#endif
        timeElapsed = clock();

        candidates.clear();
        frameNo = videoSrc.get(CV_CAP_PROP_POS_FRAMES);
        videoSrc >> curFrameIm;
        sprintf(outputFilenameBuffer, outputFolder.c_str(), (int) frameNo);

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
        // filter the results
        for( GenericFilter*& curFilter : postFilters)
            curFilter->process(candidates);

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

void parseArguments( int _argc, char* _argv[], // input arguments
                     std::string* _modelFile, std::string* _outputFolder, std::string* _videoFile, // output arguments
                     float* _nmsThreshold, bool* _optMirroring, bool* _optResume)
{
    po::options_description opts("Program parameters");
    opts.add_options()
            ("help,h","produce help message")
            ("model,m", po::value<string>(_modelFile), "Model file to use. xml, mat or jaml format")
            ("video,v", po::value<string>(_videoFile),"Video file to analyse")
            ("dir,d", po::value<string>(_outputFolder),"Output folder")
            ("resume,r", "Resume option [default: false]")
            ("nms,n", po::value<float>(_nmsThreshold)->default_value(DEFAULT_NMS_THRESHOLD), "NMS filter threshold, default 0.3 (30%)")
            ("mirror", "Mirroring option - mirror the video image horizontally and process 2x (with corrections of the detections) [default: false]");
    po::variables_map vm;
    po::store(po::parse_command_line(_argc, _argv, opts), vm);
    po::notify(vm);

    if( vm.count("help") ){
        cout << opts << endl;
        exit(0);
    }

    *_optMirroring = (vm.count("mirror") > 0) ? true : DEFAULT_MIRRORING;
    *_optResume = (vm.count("resume") > 0) ? true : DEFAULT_RESUME;

    LOG(INFO) << "Model file: " << *_modelFile;
    LOG(INFO) << "Video file: " << *_videoFile;
    LOG(INFO) << "Output folder: " << *_outputFolder;
    LOG(INFO) << "Resume option: " << (*_optResume ? "yes" : "no");
    LOG(INFO) << "Mirror option: " << (*_optMirroring ? "yes" : "no");
    LOG(INFO) << "NMS threshold: " << *_nmsThreshold;
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

////////////  Debug verions of output which are QtCreator friendly \\\\\\\\\\\

void setupDisplayDebug(char* _model, char* _inputVideo, char* _outputFolder){
    DLOG(INFO) << "Model file: " << _model;
    cout << "Model file: " << _model << endl;

    DLOG(INFO) << "Input video: " << _inputVideo;
    cout  << "Input video: " << _inputVideo << endl;

    DLOG(INFO) << "Output folder: " << _outputFolder;
    cout << "Output folder: " << _outputFolder << endl;
}

void updateDisplayDebug(int _frame, float _perc, double _time){
    DLOG(INFO) << "Frame no: " << _frame << "[" << _perc << "%] in TPF: "  << _time;
}
