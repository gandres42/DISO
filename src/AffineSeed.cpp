//
// See AffineSeed.h for why this exists.
//

#include "AffineSeed.h"
#include "Frame.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>

#include <cmath>
#include <vector>

using namespace std;

Eigen::Isometry3d AffineToSonarSE3(const Eigen::Matrix<double, 2, 3> &A, double theta, double tx,
                                   double ty, double scale, double* yaw_out)
{
    //estimateAffinePartial2D returns a similarity, but a sonar fan has no scale
    //freedom -- range is metric.  Project the 2x2 block onto the nearest rotation
    //so the spurious scale factor is dropped instead of leaking into the seed.
    Eigen::Matrix2d block = A.block<2, 2>(0, 0);
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(block, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    const Eigen::Matrix2d V = svd.matrixV();
    if ((U * V.transpose()).determinant() < 0) {
        U.col(1) *= -1;
    }
    const Eigen::Matrix2d R_s = U * V.transpose();

    const Eigen::Vector2d t_p(tx, ty);
    Eigen::Matrix2d M;
    M << cos(theta), -sin(theta), sin(theta), cos(theta);
    M *= scale;

    const Eigen::Vector2d t_s =
            M.inverse() * (A.block<2, 1>(0, 2) - (Eigen::Matrix2d::Identity() - R_s) * t_p);

    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.matrix().block<2, 2>(0, 0) = R_s;
    T.matrix().block<2, 1>(0, 3) = t_s;
    if (yaw_out) {
        *yaw_out = atan2(R_s(1, 0), R_s(0, 0));
    }
    return T;
}

AffineSeedResult EstimateAffineSeed(const Frame &f_pre, const Frame &f_cur,
                                    const AffineSeedParams &params)
{
    AffineSeedResult result;
    if (!params.enabled) {
        result.reason = "disabled";
        return result;
    }
    if (f_pre.mKeyPoints.empty()) {
        result.reason = "no keypoints";
        return result;
    }
    if (f_pre.mImg.empty() || f_cur.mImg.empty() || f_pre.mImg.size() != f_cur.mImg.size()) {
        result.reason = "image size mismatch";
        return result;
    }

    cv::Mat pre_gray, cur_gray;
    cv::cvtColor(f_pre.mImg, pre_gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(f_cur.mImg, cur_gray, cv::COLOR_BGR2GRAY);

    //DetectKeyPoints has already dropped everything within 0.3m of the range limits
    //and within ~3 degrees of the fan edge, so the static wedge boundary -- easily
    //the strongest gradient in the image, and one that does not move with the
    //vehicle -- is not in this set to begin with.
    vector<cv::Point2f> pts_pre;
    pts_pre.reserve(f_pre.mKeyPoints.size());
    for (const auto &kp: f_pre.mKeyPoints) {
        pts_pre.emplace_back(static_cast<float>(kp.first), static_cast<float>(kp.second));
    }

    const cv::Size win(params.lk_window, params.lk_window);
    const cv::TermCriteria crit(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01);

    vector<cv::Point2f> pts_cur, pts_back;
    vector<uchar> status_fwd, status_bwd;
    vector<float> err_fwd, err_bwd;
    cv::calcOpticalFlowPyrLK(pre_gray, cur_gray, pts_pre, pts_cur, status_fwd, err_fwd, win,
                             params.lk_levels, crit);
    cv::calcOpticalFlowPyrLK(cur_gray, pre_gray, pts_cur, pts_back, status_bwd, err_bwd, win,
                             params.lk_levels, crit);

    //Forward-backward consistency is the only cheap outlier test that works on sonar
    //speckle: a point that tracks somewhere plausible but wrong rarely tracks back.
    const double fb_sq = params.forward_backward_px * params.forward_backward_px;
    vector<cv::Point2f> src, dst;
    src.reserve(pts_pre.size());
    dst.reserve(pts_pre.size());
    for (size_t i = 0; i < pts_pre.size(); i++) {
        if (!status_fwd[i] || !status_bwd[i]) {
            continue;
        }
        const cv::Point2f d = pts_back[i] - pts_pre[i];
        if (static_cast<double>(d.x * d.x + d.y * d.y) > fb_sq) {
            continue;
        }
        //a point that flowed onto the fan boundary is tracking the sensor, not the scene
        if (f_cur.IsMarginalPoint(pts_cur[i].x, pts_cur[i].y)) {
            continue;
        }
        src.push_back(pts_pre[i]);
        dst.push_back(pts_cur[i]);
    }
    result.tracked = static_cast<int>(src.size());

    if (result.tracked < params.min_inliers) {
        result.reason = "too few tracked points";
        return result;
    }

    cv::Mat inlier_mask;
    const cv::Mat A_cv = cv::estimateAffinePartial2D(src, dst, inlier_mask, cv::RANSAC,
                                                     params.ransac_px, 2000, 0.995, 10);
    if (A_cv.empty()) {
        result.reason = "affine fit failed";
        return result;
    }
    result.inliers = cv::countNonZero(inlier_mask);
    if (result.inliers < params.min_inliers) {
        result.reason = "too few affine inliers";
        return result;
    }

    Eigen::Matrix<double, 2, 3> A;
    for (int r = 0; r < 2; r++) {
        for (int c = 0; c < 3; c++) {
            A(r, c) = A_cv.at<double>(r, c);
        }
    }
    if (!A.allFinite() || A.block<2, 2>(0, 0).determinant() <= 0) {
        result.reason = "degenerate affine";
        return result;
    }

    double yaw = 0.0;
    const Eigen::Isometry3d T = AffineToSonarSE3(A, f_pre.mTheta, f_pre.mTx, f_pre.mTy, f_pre.mScale,
                                                 &yaw);

    //A wildly out-of-family estimate means the correspondences were garbage, not that
    //the vehicle teleported; better to report failure and let the caller fall back.
    if (T.translation().norm() > params.max_translation_m ||
        fabs(yaw) > params.max_yaw_deg * M_PI / 180.0) {
        result.reason = "seed outside sanity limits";
        return result;
    }

    result.T_cur_pre = T;
    result.yaw = yaw;
    result.ok = true;
    return result;
}
