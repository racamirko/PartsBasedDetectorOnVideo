/*
 *     PartsBasedDetectorOnVideo
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
 *  File:    FilterNMS.cpp
 *  Author:  Mirko Raca <name.lastname@epfl.ch>
 *  Created: November 26, 2013
 */

#include "FilterNMS.h"

#include <opencv2/core/core.hpp>
#include <exception>
#include <list>
#include <algorithm>
#include <utility>
#include <string>
#include <stdio.h>

#include "globalIncludes.h"

typedef std::pair<float, int> tOrderPair;
bool compareSortOrder(const tOrderPair& _l, const tOrderPair& _r);
void calculateOverlapMatrix( vectorCandidate& _candidates, cv::Mat & _overlapMat );
float calcOverlap(const cv::Rect _r1, const cv::Rect _r2);
std::string rect2str(const cv::Rect _r);

FilterNMS::FilterNMS(float _overlap)
    : mOverlap(_overlap)
{
    if(_overlap > 1.0f)
        _overlap /= 100.0f;
    DLOG(INFO) << "Created NMS filter for overlap greater then " << _overlap*100.0 << " %";
}

/*
  Sorting trick found here
        http://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
*/
void FilterNMS::process(vectorCandidate& _candidates){
    // sort all candidates in descending order based on the result
    vectorCandidate orderedCandidates;
    {
        std::list<Candidate> orderedCandidatesList; // just to control the memory consumption a bit. The result ends in orderedCandidates
        std::vector<tOrderPair> resortBuffer; // <score, original_index>
        int idx = 0;
        for( const Candidate& cand : _candidates )
            resortBuffer.push_back( tOrderPair(cand.score(), idx++) );
        std::sort(resortBuffer.begin(), resortBuffer.end(), compareSortOrder);
        for( const tOrderPair& elem : resortBuffer )
            orderedCandidatesList.push_back( _candidates.at(elem.second) );
        std::copy(orderedCandidatesList.begin(), orderedCandidatesList.end(), std::back_inserter(orderedCandidates));
    }
    // create overlap matrix
    cv::Mat overlapMat = cv::Mat::zeros(orderedCandidates.size(), orderedCandidates.size(), CV_32FC1);
    calculateOverlapMatrix(orderedCandidates, overlapMat);
    // interate and insert only the top-scoring
    std::list<Candidate> outputResults;
    {
        for(int idx = 0; idx < orderedCandidates.size(); ++idx){
            cv::Mat curRow = overlapMat.row(idx);
            cv::Mat boolSelection = curRow > mOverlap;
            if( cv::sum(boolSelection).val[0] == 0 )
                outputResults.push_back(orderedCandidates[idx]);
        }
    }
    // re-insert the top-scoring candidates in the output vector
    _candidates.clear();
    std::copy(outputResults.begin(), outputResults.end(), std::back_inserter(_candidates));

    // warning system, to remove at the end
    LOG(FATAL) << "Not implemented";
    throw std::exception();
}

// made for descending sort
bool compareSortOrder(const tOrderPair& _l, const tOrderPair& _r){
    return _l.first > _r.first;
}

/*
 *  Important to note that the overlapMat encodes the information
 *      overlap.at<float>(x,y) = (x & y)/x;
 *  The matrix will be 0.0 for any idx <= of the current outer idx
 */
void calculateOverlapMatrix( vectorCandidate& _candidates, cv::Mat &_overlapMat ){
    int outerIdx = 0;
    for( const Candidate& _outerC : _candidates ){
        int innerIdx = 0;
        for( const Candidate& _innerC : _candidates ){
            if(innerIdx <= outerIdx)
                continue;
            _overlapMat.at<float>(outerIdx, innerIdx) = calcOverlap(_outerC.boundingBox(), _innerC.boundingBox());
            ++innerIdx;
        }
        ++outerIdx;
    }
    DLOG(INFO) << "Overlap matrix: " << _overlapMat;
}

/*
 * Overlap percentage from the "viewpoint" of the first rectangle.
 *  (meaning, the intersection coverage area is devided by the area of the
 *  of the first rectangle.
 */
float calcOverlap(const cv::Rect _r1, const cv::Rect _r2){
    DLOG(INFO) << "Overlap between " << rect2str(_r1) << " and " << rect2str(_r2);
    cv::Rect intersection = _r1 & _r2;
    DLOG(INFO) << "    is " << rect2str(intersection);
    float overlapPct = (float)intersection.area()/(float)_r1.area();
    DLOG(INFO) << "    in the pct of the first rect: " << overlapPct;
    return overlapPct;
}

std::string rect2str(const cv::Rect _r){
    char buffer[100];
    sprintf(buffer, "[%d, %d, %d, %d]", _r.x, _r.y, _r.width, _r.height);
    return std::string(buffer);
}
