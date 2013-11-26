/*
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
 *  File:    FilterSize.cpp
 *  Author:  Mirko Raca <name.lastname@epfl.ch>
 *  Created: November 18, 2013
 */

#include "FilterSize.h"

#include "globalIncludes.h"
#include <algorithm>

FilterSize::FilterSize(cv::Size2f _maxSize )
    : mMaxSize(_maxSize)
{
    DLOG(INFO) << "Created FilterSize [" << mMaxSize.width << ", " << mMaxSize.height << "]";
}

void FilterSize::process(vectorCandidate& _candidates){
    vectorCandidate tmpResult;
#ifndef NDEBUG
    DLOG(INFO) << "Originaly " << _candidates.size() << " candidates";
    cv::Rect minSize = cv::Rect(0,0,1000,1000);
#endif

    for( Candidate& curCandidate : _candidates ){
        cv::Rect rect = curCandidate.boundingBox();
#ifndef NDEBUG
        if( rect.height < minSize.height )
            minSize.height = rect.height;
        if( rect.width < minSize.width )
            minSize.width = rect.width;
#endif
        if( rect.width > mMaxSize.width || rect.height > mMaxSize.height )
            continue;
        tmpResult.push_back(curCandidate);
    }
#ifndef NDEBUG
    DLOG(INFO) << "Minimum rect size is: " << minSize.width << ", " << minSize.height;
    DLOG(INFO) << "Now " << tmpResult.size() << " candidates";
#endif
    _candidates.clear();
    std::copy(tmpResult.begin(), tmpResult.end(), std::back_inserter(_candidates));
}
