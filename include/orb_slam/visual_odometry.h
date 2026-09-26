#pragma once

#include "orb_slam/backend.h"
#include "orb_slam/common_include.h"
#include "orb_slam/dataset.h"
#include "orb_slam/frontend.h"
#include "orb_slam/viewer.h"

namespace orb_slam {

class VisualOdometry {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    VisualOdometry(std::string &config_path);

    bool Init();

    void Run();

    bool Step();

    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }
    
private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    // dataset
    Dataset::Ptr dataset_ = nullptr;


};

}