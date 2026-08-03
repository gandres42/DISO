//
// Created by da on 09/08/23.
//

#ifndef SRC_LOCALMAPPING_H
#define SRC_LOCALMAPPING_H
#include <memory>
#include <utility>
#include <map>
#include <set>
#include <deque>
#include <shared_mutex>
#include <atomic>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std;
class Frame;

class MapPoint;

class Track;

class LocalMapping;

// class LocalMapState{
// protected:
//     shared_ptr<LocalMapping> mpLocalMaper;
// public:
//     LocalMapState(shared_ptr<LocalMapping> pLocalMaper):mpLocalMaper(pLocalMaper){};
//     virtual void Update() = 0;
// };
//
// class LocalMapToUpdate : public LocalMapState{
// public:
//     LocalMapToUpdate(shared_ptr<LocalMapping> pLocalMaper):LocalMapState(pLocalMaper){};
//     void Update() override;
// };
// class LocalMapUpToDate : public LocalMapState{
// public:
//     LocalMapUpToDate(shared_ptr<LocalMapping> pLocalMaper):LocalMapState(pLocalMaper){};
//     void Update() override;
// };

class LocalMapping : public enable_shared_from_this<LocalMapping>
{
public:
    LocalMapping(rclcpp::Node* node, shared_ptr<Track> pTracker);
    LocalMapping(rclcpp::Node* node, shared_ptr<Track> pTracker, Eigen::Isometry3d T_bw_b0);
    void Run();
    void RequestStop();
    void SetDebugDir(const std::string &debug_dir);
    void NotifyTracker();
    // void SetState(shared_ptr<LocalMapState> pState);
    void InsertKeyFrame(shared_ptr<Frame> pF);
    void ProcessKeyFrame();
    void OptimizeWindow();
    void PoseEstimationWindow2Frame(shared_ptr<Frame> pF_pre, shared_ptr<Frame> pF,
                                    set<pair<double, double>> &inliers_last,
                                    set<pair<double, double>> &inliers_current,
                                    map<pair<double, double>, pair<double, double>> &association, double range, double fov);
    void Visualize();
    shared_ptr<Frame> GetLastFrameInWindow();
    deque<shared_ptr<Frame>> GetWindow();
    void Reset();
    void SaveMapCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                         std::shared_ptr<std_srvs::srv::Empty::Response> res);
    void PubMap();


protected:
    rclcpp::Node* mpNode;
    shared_ptr<Track> mpTracker;
    // shared_ptr<LocalMapState> mState;

    // protected by mWindowMutex
    shared_mutex mWindowMutex;
    int mWindowSize = 10;
    deque<shared_ptr<Frame>> mActiveFrameWindow;

    //Frame to process, protected by mProcessingQueueMutex
    shared_mutex mProcessingQueueMutex;
    deque<shared_ptr<Frame>> mProcessingQueue;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mMarkerPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mPointCloudPub;
    std::map<int,shared_ptr<MapPoint>> mActiveMapPoints;
    //ros save service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mSaveService;

    std::atomic<bool> mbStopRequested{false};
    std::string mDebugDir;
public:
    Eigen::Isometry3d mT_bw_b0;

};


#endif //SRC_LOCALMAPPING_H
