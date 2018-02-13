#ifndef __JsonConfig_H
#define __JsonConfig_H

#include <string>
#include <fstream>

#include <json/json.h>

#include "Common.h"

class JsonConfig
{

public:
    JsonConfig(const std::string &strFilePath):mFilePath(strFilePath)
    {
    }

    inline int Init()
    {
        mFile.open(mFilePath);
        if(!mFile.is_open())
        {
            return GS::RET_FAILED;
        }
        else
        {
            Json::Reader reader;
            if(reader.parse(mFile,mRootValue))
                return GS::RET_SUCESS;
            else
                return GS::RET_FAILED;
        }
    }

    inline int GetInt(const std::string &strKey)
    {
        return mRootValue[strKey].asInt();
    }

    inline std::string GetString(const std::string &strKey)
    {
        return mRootValue[strKey].asString();
    }

private:
    std::string mFilePath;
    std::ifstream mFile;
    Json::Value mRootValue;
};

#endif
