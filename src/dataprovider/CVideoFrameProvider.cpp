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
 *  File:    CVideoFrameProvider.h
 *  Author:  Mirko Raca <name.lastname@epfl.ch>
 *  Created: May 22, 2014.
 */

#include "CVideoFrameProvider.h"

#include <exception>
#include <stdexcept>
#include <boost/filesystem.hpp>

#include "globalIncludes.h"

CVideoFrameProvider::CVideoFrameProvider(std::string _srcFilename)
    : mSrcFilename(_srcFilename)
    , mVideoSrc(_srcFilename)
    , mInitiated(false)
{
    DLOG(INFO) << "CVideoFrameProvider instanced";
    LOG(INFO) << "Src file: " << _srcFilename;
    if(mVideoSrc.isOpened()){
        mInitiated = true;
    } else {
        LOG(ERROR) << "Could not open file: " << _srcFilename;
        throw std::runtime_error("Could not find file: " + _srcFilename);
    }
}

double CVideoFrameProvider::getFrameCount(){
    if(!mInitiated)
        return -1;
    return mVideoSrc.get(CV_CAP_PROP_FRAME_COUNT);
}

double CVideoFrameProvider::getCurrentFrameNumber(){
    if(!mInitiated)
        return -1;
    return mVideoSrc.get(CV_CAP_PROP_POS_FRAMES);
}

CGenericFrameProvider& CVideoFrameProvider::operator>>(cv::Mat& _mat){
    if(mInitiated)
        mVideoSrc >> _mat;
}

CVideoFrameProvider::~CVideoFrameProvider(){
    if(mInitiated)
        mVideoSrc.release();
}

std::string CVideoFrameProvider::getCurrentFilename(){
    return boost::filesystem::path(mSrcFilename).filename().native();
}
