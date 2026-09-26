#pragma once

#include "orb_slam/common_include.h"

namespace orb_slam{


class Config{

private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config(){}
public:

    ~Config();

    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameter values
    template <typename T>
    static T Get(const std::string& key)
    {
        return T(Config::config_->file_[key]);
    }
};
}