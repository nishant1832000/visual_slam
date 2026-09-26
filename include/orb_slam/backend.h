#pragma once

#include "orb_slam/common_include.h"
#include "orb_slam/frame.h"
#include "orb_slam/map.h"

namespace orb_slam{

class Map;

class Backend {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;
    
    Backend();

    void SetCameras(Camera::Ptr left, Camera::Ptr right){
        cam_left_ = left;
        cam_right_ = right;
    }

    void SetMap(std::shared_ptr<Map> map) {map_ = map; }

    void UpdateMap();

    void Stop();

private:
    void BackendLoop();

    void Optimize(Map::KeyframesType& keyframes, Map::LandmarksType& landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
};

}