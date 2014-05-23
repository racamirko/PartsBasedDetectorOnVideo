#ifndef CSEQUENTIALFORMATTER_H
#define CSEQUENTIALFORMATTER_H

#include "CGenericFormatter.h"

class CSequentialFormatter : public CGenericFormatter
{
protected:
    std::string mBaseFolder, mOutputFileFormat, mWholeTemplate;

public:
    CSequentialFormatter(CGenericFrameProvider* _frameProv,
                         std::string _baseFolder,
                         std::string _outputFileFormat = "facedetect_frame%06d.txt");

    std::string getFilename();
};

#endif // CSEQUENTIALFORMATTER_H
