/*
 *     PartsBasedDetectorOnVideo
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Chili lab, EPFL.
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
 *  File:    CFolderFrameProvider.cpp
 *  Author:  Mirko Raca <name.lastname@epfl.ch>
 *  Created: May 22, 2014.
 */
#include "CFolderFrameProvider.h"

#include <boost/filesystem.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>

#include "globalIncludes.h"

CFolderFrameProvider::CFolderFrameProvider(std::string _srcFolder)
    : mSrcFolder(_srcFolder)
    , mCurFrame(-1)
{
    DLOG(INFO) << "CFolderFrameProvider created";
    init();
}

void CFolderFrameProvider::init(){
    DLOG(INFO) << "Initializing the CFolderFrameProvider";
    namespace fs = boost::filesystem;
    fs::path someDir(mSrcFolder);
    fs::directory_iterator end_iter;

    DLOG(INFO) << "Listing files";
    if ( fs::exists(someDir) && fs::is_directory(someDir)) {
        for( fs::directory_iterator dir_iter(someDir) ; dir_iter != end_iter ; ++dir_iter) {
            if (fs::is_regular_file(dir_iter->status()) ) {
                DLOG(INFO) << "Loading: " << dir_iter->path().native();
                mVecFiles.push_back(dir_iter->path().native());
            }
        }
    }
    std::sort(mVecFiles.begin(), mVecFiles.end());

#ifndef NDEBUG
    for( const std::string& curStr : mVecFiles ){
        DLOG(INFO) << "Final order: " << curStr;
    }
#endif

    mPositionIter = mVecFiles.begin();
    mCurFrame = 0;
}

double CFolderFrameProvider::getFrameCount(){
    return mVecFiles.size();
}

double CFolderFrameProvider::getCurrentFrameNumber(){
    return mCurFrame;
}

CGenericFrameProvider& CFolderFrameProvider::operator>>(cv::Mat& _mat){
    _mat = cv::imread(*mPositionIter);
    mPositionIter++;
    ++mCurFrame;
}

