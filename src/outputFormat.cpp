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
 *  File:    outputFormat.cpp
 *  Author:  Mirko Raca <name.lastname@epfl.ch>
 *  Created: November 6, 2013
 */

#include "outputFormat.h"
#include <iomanip>

enumFormatType gOutputFormat = FT_FULL_OUTPUT;

std::ostream& operator<<(std::ostream& _stream, const vectorCandidate& _vecCandidates){
    // format [ frameNo, compNo, mixtureNo, score (.4f), comp1_x1 (.2f), comp1_y1, comp1_x2, comp1_y2 ...]
    for( auto& c : _vecCandidates ){
        switch(gOutputFormat){
            case FT_FULL_OUTPUT:
                _stream << "0, " << c.parts().size() << ", " << c.component() << ", "
                        << std::setprecision(4) << c.score();
                // sub-parts
                _stream << std::setprecision(2);
                for( const cv::Rect& r : c.parts() )
                    _stream << ", " << r.x << ", " << r.y << ", " << r.x+r.width << ", " << r.y+r.height;
                _stream << std::endl;
                break;
            case FT_BBOX_BRIEF:
                cv::Rect bBox = c.boundingBox();
                _stream << "[ {" << c.component() << " = " << std::setprecision(4) << c.score() <<"}, " << bBox.x << ", " << bBox.y << ", " << bBox.x+bBox.width << ", " << bBox.y+bBox.height << "]" << std::endl;
        }
    }
}


