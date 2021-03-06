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
 *  File:    main.cpp
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


#include "filters/GenericPreFilter.h"
#include "filters/GenericPostFilter.h"
#include "filters/FilterSize.h"
#include "filters/FilterNMS.h"
#include "filters/PreFilterBackgroundMask.h"

#include "dataprovider/CProviderFactory.h"

#include "output_format/CSequentialFormatter.h"
#include "output_format/CNameCopyFormatter.h"

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
                     std::string* _modelFile, std::string* _outputFolder, std::string* _srcFilename, // output arguments
                     float* _nmsThreshold, bool* _optMirroring, bool* _optResume, vector<float>* _optSizeFilter,
                     std::string* _optMaskFilter, float* _optModelThresh, bool* _copyFilename);

#define OUTPUT_FILENAME_FORMAT "facedetect_frame%06d.txt"
#define DEFAULT_MIRRORING false
#define DEFAULT_RESUME false
#define DEFAULT_MODEL_THRESH -100.0f
#define DEFAULT_COPY_FILENAME false

#ifdef NDEBUG
    void setupDisplay(const char* _model, const char* _srcFilename, const char* _outputFolder);
    void updateDisplay(int _frame, float _perc, double _time);
    #define FRAME_LIMIT frameCount
#else
    void updateDisplayDebug(int _frame, float _perc, double _time);
    void setupDisplayDebug(const char* _model, const char* _srcFilename, const char* _outputFolder);
    #define FRAME_LIMIT 200
#endif

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    DLOG(INFO) << "Execution started";

    // process variables
    float nmsThreshold = 0.0f;
    float modelThreshold = DEFAULT_MODEL_THRESH;
    bool optMirroring = DEFAULT_MIRRORING;
    bool optResume = DEFAULT_RESUME;
    bool optCopyFilename = DEFAULT_COPY_FILENAME;
    string outputFolder = "";
    string modelFile = "";
    string srcFilename = "";
    string maskFilterFile = "";
    vector<float> sizeFilter;

    // general variables
    boost::scoped_ptr<Model> model;

    parseArguments(argc, argv, &modelFile, &outputFolder, &srcFilename, &nmsThreshold, &optMirroring, &optResume, &sizeFilter,
                   &maskFilterFile, &modelThreshold, &optCopyFilename);

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

    if( modelThreshold != DEFAULT_MODEL_THRESH ){
        LOG(INFO) << "Setting model threshold to " << modelThreshold << ", instead of model default: " << model->thresh();
        model->setThreshold(modelThreshold);
    }

    // create the PartsBasedDetector and distribute the model parameters
    Mat_<float> depth; // we don't have one for the video, so it's just a dummy variable
    PartsBasedDetector<float> pbd;
    pbd.distributeModel(*model);
    // load video sequence
    CGenericFrameProvider* pFrameSrc = NULL;
    try{
        pFrameSrc = CProviderFactory::getProvider(srcFilename);
    } catch(std::runtime_error& error){
        LOG(FATAL) << "Error opening file: " << error.what();
        printf("Could not open frame source");
        endwin();
        exit(-4);
    }

    double frameCount = pFrameSrc->getFrameCount();
    double frameNo = pFrameSrc->getCurrentFrameNumber();


    DLOG(INFO) << "Frame count: " << frameCount;
    DLOG(INFO) << "Start frame no: " << frameNo;

    // check output folder
    CGenericFormatter* pOutputFormat = NULL;
    if(optCopyFilename)
        pOutputFormat = new CNameCopyFormatter(pFrameSrc, outputFolder);
    else
        pOutputFormat = new CSequentialFormatter(pFrameSrc, outputFolder);

    // pre filters
    std::vector<GenericPreFilter*> preFilters;
    if( maskFilterFile.size() > 0 ){
        preFilters.push_back( new PreFilterBackgroundMask(maskFilterFile) );
    }

    // post filters
    std::vector<GenericPostFilter*> postFilters;
    if( sizeFilter.size() > 0 )
        postFilters.push_back(new FilterSize(Size2f(sizeFilter[0],sizeFilter[1])));
    if( nmsThreshold != 0.0f )
        postFilters.push_back(new FilterNMS(nmsThreshold));

    // display initialzation
#ifdef NDEBUG
    setupDisplay(modelFile.c_str(), srcFilename.c_str(), outputFolder.c_str()); // release
#else
    setupDisplayDebug(modelFile.c_str(), srcFilename.c_str(), outputFolder.c_str()); // debug
#endif
    // main loop
    DLOG(INFO) << "main loop";
    vectorCandidate candidates;
    Mat curFrameIm;
    string outputFilename;
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
        outputFilename = pOutputFormat->getFilename();
        frameNo = pFrameSrc->getCurrentFrameNumber();
        *pFrameSrc >> curFrameIm;

        // check if already exists
        if( optResume ){
            if(boost::filesystem::exists(outputFilename))
                continue;
        }

        // filter the image
        for( GenericPreFilter*& curFilter : preFilters ){
            curFilter->process(curFrameIm);
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
        for( GenericPostFilter*& curFilter : postFilters)
            curFilter->process(candidates);

        DLOG(INFO) << "Final all detections" << candidates;
        // output
        ofstream outFile(outputFilename);
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

    DLOG(INFO) << "Execution finished";
    endwin();
    return 0;
}

/*
 *  Code based on examples from
 *      http://www.boost.org/doc/libs/1_46_0/doc/html/program_options/tutorial.html
 *
 */
void parseArguments( int _argc, char* _argv[], // input arguments
                     std::string* _modelFile, std::string* _outputFolder, std::string* _srcFilename, // output arguments
                     float* _nmsThreshold, bool* _optMirroring, bool* _optResume, vector<float>* _optSizeFilter,
                     std::string* _optMaskFilter, float* _optModelThresh, bool* _copyFilename)
{
    po::options_description opts("Program parameters");
    opts.add_options()
            ("help,h","produce help message")
            ("model,m", po::value<string>(_modelFile), "Model file to use. xml, mat or jaml format")
            ("input,i", po::value<string>(_srcFilename),"Input, source for analysis (video/folder/image)")
            ("dir,d", po::value<string>(_outputFolder),"Output folder")
            ("resume,r", "Resume option [default: false]")
            ("nms,n", po::value<float>(_nmsThreshold)->default_value(0.0f), "NMS filter threshold, percentage in the range 0.0-1.0, default O.0")
            ("mirror", "Mirroring option - mirror the video image horizontally and process 2x (with corrections of the detections) [default: false]")
            ("size,s", po::value< vector<float> >(_optSizeFilter), "Size filter. Eliminate all instances bigger then [width],[height]" )
            ("filter,f", po::value<string>(_optMaskFilter), "Mask filter - binary image. White regions of the image are going to be analyzed.")
            ("thresh,t", po::value<float>(_optModelThresh)->default_value(DEFAULT_MODEL_THRESH), "Theshold of the model. Default value is whatever is used in the model file")
            ("copyFilename,c", "Copy the original filename to the output (with changed extension to .txt)");
    po::variables_map vm;
    po::store(po::parse_command_line(_argc, _argv, opts), vm);
    po::notify(vm);

    if( vm.count("help") ){
        cout << opts << endl;
        exit(0);
    }

    if( vm.count("model") == 0 ){
        cout << "No model specified. Check --help for usage" << endl;
        LOG(FATAL) << "No model specified.";
        exit(-1);
    }

    if( vm.count("input") == 0 ){
        cout << "No input specified. Check --help for usage" << endl;
        LOG(FATAL) << "No input specified.";
        exit(-2);
    }

    if( vm.count("dir") == 0 ){
        cout << "No output folder specified. Check --help for usage" << endl;
        LOG(FATAL) << "No output folder specified.";
        exit(-3);
    }

    if( vm.count("size") > 0 ){
        _optSizeFilter->clear();
        *_optSizeFilter = vm["size"].as< vector< float >>();
    }

    *_optMirroring = (vm.count("mirror") > 0) ? true : false;
    *_optResume = (vm.count("resume") > 0) ? true : false;
    *_copyFilename = (vm.count("copyFilename") > 0) ? true : false;

    LOG(INFO) << "Model file: " << *_modelFile;
    LOG(INFO) << "Input path: " << *_srcFilename;
    LOG(INFO) << "Output folder: " << *_outputFolder;
    LOG(INFO) << "Mask filter file: " << *_optMaskFilter;
    LOG(INFO) << "Resume option: " << (*_optResume ? "yes" : "no");
    LOG(INFO) << "Mirror option: " << (*_optMirroring ? "yes" : "no");
    if( *_optModelThresh == DEFAULT_MODEL_THRESH )
        LOG(INFO) << "Model threshold: [model default]";
    else
        LOG(INFO) << "Model threshold: " << *_optModelThresh;
    if( *_nmsThreshold == 0.0f )
        LOG(INFO) << "NMS threshold: off";
    else
        LOG(INFO) << "NMS threshold:" << *_nmsThreshold ;
}

void setupDisplay(const char* _model, const char* _srcFilename, const char* _outputFolder){
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

    mvprintw(4, 3, "Source: ");
    mvprintw(4, 25, boost::filesystem::path(_srcFilename).filename().c_str());

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
    printw("] %3.2f% [Frame #%d]", _perc, _frame);
    move(11, cols/2 - 3);
    printw("TPF: %2.2f sec", _time);

    refresh();
}

/*             Debug verions of output which are QtCreator friendly             */

#ifndef NDEBUG

void setupDisplayDebug(const char* _model, const char* _srcFilename, const char* _outputFolder){
    DLOG(INFO) << "Model file: " << _model;
    cout << "Model file: " << _model << endl;

    DLOG(INFO) << "Source: " << _srcFilename;
    cout  << "Source: " << _srcFilename << endl;

    DLOG(INFO) << "Output folder: " << _outputFolder;
    cout << "Output folder: " << _outputFolder << endl;
}

void updateDisplayDebug(int _frame, float _perc, double _time){
    DLOG(INFO) << "Frame no: " << _frame << "[" << _perc << "%] in TPF: "  << _time;
}

#endif
