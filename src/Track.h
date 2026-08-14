//
// Created by da on 03/08/23.
//

#ifndef SRC_TRACK_H
#define SRC_TRACK_H

#include <memory>
#include <utility>
#include <map>
#include <set>
#include <deque>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include "nanoflann.hpp"
#include "AffineSeed.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <shared_mutex>

using namespace std;
using namespace nanoflann;

class Frame;

class MapPoint;

class Track;

class LocalMapping;

class TrackState{
protected:
    shared_ptr<Track> mpTracker;
public:
    TrackState(shared_ptr<Track> pTracker):mpTracker(pTracker){};
    virtual void TrackFrame(const Frame& f) = 0;
};

class TrackToUpdate : public TrackState{
public:
    TrackToUpdate(shared_ptr<Track> pTracker):TrackState(pTracker){};
    void TrackFrame(const Frame &f) override;
};
class TrackUpToDate : public TrackState{
public:
    TrackUpToDate(shared_ptr<Track> pTracker):TrackState(pTracker){};
    void TrackFrame(const Frame &f) override;
};

class Track : public enable_shared_from_this<Track>
{
public:
    Track(rclcpp::Node* node, double range, double fov, int pyramid_layer, int loss_threshold,
          double gradient_threshold, Eigen::Isometry3d& T_b_s, bool use_odom = true,
          const AffineSeedParams& affine_params = AffineSeedParams());

    shared_ptr<Frame> mpCurrentFrame;
    shared_ptr<Frame> mpLastFrame;

    void TrackFrame2Frame(const Frame &f);

    Eigen::Isometry3d TrackFrame(const Frame &f);

    void TrackFromLastFrame(const Frame &f);

    void TrackFromWindow(const Frame &f);

    //Seeds f_cur's pose with the estimated motion from f_pre.  Returns true when that
    //seed came from a real measurement (odometry, or the affine image alignment) rather
    //than from extrapolating the last relative motion.
    bool PredictCurrentPose(shared_ptr<Frame> f_pre, shared_ptr<Frame> f_cur);

    // void OptimizeWindow();

    bool PoseEstimationFrame2Frame(cv::Mat &pre_img, cv::Mat &img, Eigen::Isometry3d &Tsj_si,
                                   set<pair<double, double>> &obs, int layer,
                                   set<pair<double, double>> &inliers_last,
                                   set<pair<double, double>> &inliers_current,
                                   map<pair<double, double>, pair<double, double>> &association);

    void PoseEstimationWindow2Frame(shared_ptr<Frame> pF_pre, shared_ptr<Frame> pF,
                                    set<pair<double, double>> &inliers_last,
                                    set<pair<double, double>> &inliers_current,
                                    map<pair<double, double>, pair<double, double>> &association);

    void PoseEstimationWindow2FramePyramid(shared_ptr<Frame> pF_pre, shared_ptr<Frame> pF,
                                    set<pair<double, double>> &inliers_last,
                                    set<pair<double, double>> &inliers_current,
                                    map<pair<double, double>, pair<double, double>> &association);



    void InitializeMap();

    void Reset(shared_ptr<Frame> f_pre);

    bool NeedNewKeyFrame(shared_ptr<Frame> pF);

    void InsertKeyFrame(shared_ptr<Frame> pF);

    void DrawTrack();

    void PublishPose();

    // void Visualize();

    void SavePath(bool force = false);

    void SetOutputConfig(const std::string &output_dir, const std::string &debug_dir);

private:
    rclcpp::Node* mpNode;
    // protected by mStateMutex
    shared_ptr<TrackState> mState;
    shared_mutex mStateMutex;
public:
    void SetState(const shared_ptr<TrackState> &State);

private:

    shared_ptr<LocalMapping> mpLocalMaper;
public:
    void SetLocalMaper(const shared_ptr<LocalMapping> &pLocalMaper);

private:

    // double mScale, mTheta, mTx, mTy;

    int mPyramidLayer;
    map<double, Eigen::Isometry3d> mPath;
    map<double, Eigen::Isometry3d> mOdomPath;



    //current pose
    Eigen::Isometry3d mT_w_sj = Eigen::Isometry3d::Identity();

    //relative pose from the last successful frame-to-frame track.  Only a fallback now:
    //PredictCurrentPose prefers the affine image alignment and drops back to
    //extrapolating this when the alignment fails.
    Eigen::Isometry3d mLastRelativeMotion = Eigen::Isometry3d::Identity();

    //affine-alignment seed configuration, and the result of the most recent estimate
    AffineSeedParams mAffineParams;
    AffineSeedResult mLastSeed;

    //Local Frame window
    // int mWindowSize = 5;
    // deque<shared_ptr<Frame>> mActiveFrameWindow;

    //Local map points
    map<int, shared_ptr<MapPoint>> mMapPoints;

    //ros
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mPosePub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mPathPub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mOdomPub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mOdomPathPub;

    //output paths
    std::string mOutputDir, mDebugDir;
    int mSavePathCounter = 0;
public:
    Eigen::Isometry3d mT_b_s;
    //inlier threshold
    double mGradientInlierThreshold;
    //loss threshold
    int mLossThreshold;
    //if false, PredictCurrentPose seeds from mLastRelativeMotion instead of odom
    bool mUseOdom;
    double mRange, mFOV;
    shared_mutex mTrackMutex;

};

void AssociateFrameToLandmark(shared_ptr<Frame> pF, shared_ptr<MapPoint> pMP,
                              pair<double, double> p_pixel);

void UnassociateFrameToLandmark(shared_ptr<Frame> pF, shared_ptr<MapPoint> pMP,
                                pair<double, double> p_pixel);

void BuildAssociation(shared_ptr<Frame> pF_last, shared_ptr<Frame> pF_cur,
                      map<pair<double, double>, pair<double, double>> &association, int loss_threshold);

void IncreaseMapPoints(shared_ptr<Frame> pF);

struct Point2D
{
    double x, y;
};

struct PointCloud
{
    std::vector<Point2D> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim == 0) { return pts[idx].x; }
        else { return pts[idx].y; }
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template<class BBOX>
    bool kdtree_get_bbox(BBOX &) const { return false; }
};

typedef KDTreeSingleIndexAdaptor<L2_Simple_Adaptor<double, PointCloud>, PointCloud, 2> KDTree;


#endif //SRC_TRACK_H
