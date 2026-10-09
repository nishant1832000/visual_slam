#include "orb_slam/backend.h"
#include "orb_slam/algorithm.h"
#include "orb_slam/feature.h"
#include "orb_slam/g2o_types.h"
#include "orb_slam/map.h"
#include "orb_slam/mappoint.h"
#include "orb_slam/config.h"

namespace orb_slam{

Backend::Backend() {
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap()
{
    std::unique_lock<std::mutex> lck(data_mutex_);
    map_update_.notify_one();
}

void Backend::Stop()
{
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void Backend::BackendLoop()
{
    while(backend_running_.load()) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);


        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();

    
        Optimize(active_kfs, active_landmarks);
    }
}

void Backend::Optimize(Map::KeyframesType &keyframes,
                        Map::LandmarksType &landmarks)
{
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<BlockSolverType>(
            std::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // Keyframe id
    std::map<unsigned long, VertexPose *> vertices;
    std::deque< VertexPose *> pose_deque;
    
    unsigned long max_kf_id = 0;
    for (auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->Pose());
        optimizer.addVertex(vertex_pose);
        if (kf->keyframe_id_ > max_kf_id) {
            max_kf_id = kf->keyframe_id_;
        }
        vertices.insert({kf->keyframe_id_, vertex_pose});

        if (pose_deque.size() == 2)
        {
            pose_deque.pop_front();
        }
        pose_deque.push_back(vertex_pose);

        if (pose_deque.size() == 2)
        {
            auto *v0 = pose_deque[0];
            auto *v1 = pose_deque[1];

            auto *edge = new PoseGraphEdge();

            edge->setVertex(0, v0);
            edge->setVertex(1, v1);

            // Expected relative vertical displacement
            edge->setMeasurement(0.0);

            Eigen::Matrix<double, 1, 1> information;
            information << 150.0;

            edge->setInformation(information);
            double robust_delta = 8.19;
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(robust_delta);
            edge->setRobustKernel(rk);

            optimizer.addEdge(edge);
        }

    }

    
    std::map<unsigned long, VertexXYZ *> vertices_landmarks;

    Mat33 K = cam_left_->K();
    SE3 left_ext = cam_left_->pose();
    SE3 right_ext = cam_right_->pose();

    // edges
    int index = 1;
    double chi2_th = 8.19;  
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

    for (auto &landmark : landmarks) {
        if (landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->GetObs();
        // Get the landmark position in world coordinates
        Eigen::Vector3d P_w = landmark.second->Pos();
        for (auto &obs : observations) {
            if (obs.lock() == nullptr) continue;
            auto feat = obs.lock();
            if (feat->is_outlier_ || feat->frame_.lock() == nullptr) continue;
            SE3 T_c_b = feat->is_on_left_image_ ? left_ext : right_ext;
            auto frame = feat->frame_.lock();
            SE3 T_c_w = T_c_b * frame->Pose();
            Eigen::Vector3d P_c = T_c_w * P_w;
            double depth = P_c.z();

            // if (depth <= 0.1) {
            //     continue;
            // }

            double reference_depth = Config::Get<double>("reference_depth");
            double information_weight =
                (reference_depth * reference_depth) / (depth * depth);

            information_weight = std::clamp(information_weight, Config::Get<double>("lower_information_value"), Config::Get<double>("upper_information_value"));
            Mat22 information_matrix =
                    information_weight * Mat22::Identity();    
            EdgeProjection *edge = nullptr;
            if (feat->is_on_left_image_) {
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            if (vertices_landmarks.find(landmark_id) ==
                vertices_landmarks.end()) {
                VertexXYZ *v = new VertexXYZ;
                v->setEstimate(landmark.second->Pos());
                v->setId(landmark_id + max_kf_id + 1);
                v->setMarginalized(true);
                vertices_landmarks.insert({landmark_id, v});
                optimizer.addVertex(v);
            }


            if (vertices.find(frame->keyframe_id_) !=
                vertices.end() && 
                vertices_landmarks.find(landmark_id) !=
                vertices_landmarks.end()) {
                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id_));    // pose
                    edge->setVertex(1, vertices_landmarks.at(landmark_id));  // landmark
                    edge->setMeasurement(toVec2(feat->position_.pt));
                    edge->setInformation(information_matrix);
                    double robust_delta = chi2_th;
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(robust_delta);
                    edge->setRobustKernel(rk);
                    edges_and_features.insert({edge, feat});
                    optimizer.addEdge(edge);
                    index++;
                }
            else delete edge;
                
        }
    }

    // do optimize and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;

    while (iteration < 5) {
        cnt_outlier = 0;
        cnt_inlier = 0;
        // determine if we want to adjust the outlier threshold
        for (auto &ef : edges_and_features) {
            if (ef.first->chi2() > chi2_th) {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }
        double inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if (inlier_ratio > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for (auto &ef : edges_and_features) {
        if (ef.first->chi2() > chi2_th) {
            ef.second->is_outlier_ = true;
            // remove the observation
            ef.second->map_point_.lock()->RemoveObservation(ef.second);
        } else {
            ef.second->is_outlier_ = false;
        }
    }

    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/"
              << cnt_inlier;

    // Set pose and landmark position
    for (auto &v : vertices) {
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for (auto &v : vertices_landmarks) {
        landmarks.at(v.first)->SetPos(v.second->estimate());
    }
    
} 


}