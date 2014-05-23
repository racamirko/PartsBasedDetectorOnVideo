#ifndef CGENERICFORMATTER_H
#define CGENERICFORMATTER_H

#include <string>
#include "dataprovider/CGenericFrameProvider.h"

class CGenericFormatter {
protected:
    CGenericFrameProvider* mpFrameProv;
public:
    CGenericFormatter(CGenericFrameProvider* _frameProv)
        : mpFrameProv(_frameProv)
    { }

    virtual std::string getFilename() = 0;
};

#endif // CGENERICFORMATTER_H
