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
