add_library(orb_slam SHARED
            camera.cpp
            config.cpp
            frame.cpp
            feature.cpp
            mappoint.cpp
            map.cpp
            dataset.cpp
            viewer.cpp
            backend.cpp
            frontend.cpp
            visual_odometry.cpp)


target_link_libraries(orb_slam ${THIRD_PARTY_LIBS} glog::glog) 