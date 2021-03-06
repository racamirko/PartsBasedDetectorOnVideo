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
 *  File:    CSequentialFormatter.cpp
 *  Author:  Mirko Raca <name.lastname@epfl.ch>
 *  Created: May 23, 2014.
 */
#include "CSequentialFormatter.h"

#include "globalIncludes.h"

#include <boost/filesystem.hpp>

CSequentialFormatter::CSequentialFormatter(CGenericFrameProvider* _frameProv,
                                           std::string _baseFolder,
                                           std::string _outputFileFormat)
    : CGenericFormatter(_frameProv)
    , mBaseFolder(_baseFolder)
    , mOutputFileFormat(_outputFileFormat)
{
    LOG(INFO) << "CSequentialFormatter created:";
    LOG(INFO) << "\tBase folder: " << mBaseFolder;
    LOG(INFO) << "\tOutput file format: " << mOutputFileFormat;

    boost::filesystem::path tmpPath(mBaseFolder);
    tmpPath /= mOutputFileFormat;
    mWholeTemplate = tmpPath.native();
}


std::string CSequentialFormatter::getFilename(){
    char buffer[1000];
    std::sprintf(buffer, mWholeTemplate.c_str(), (unsigned)mpFrameProv->getCurrentFrameNumber());
    DLOG(INFO) << "Formatted filename: " << buffer;
    return std::string(buffer);
}
