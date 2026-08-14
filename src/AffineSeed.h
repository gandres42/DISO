//
// Frame-to-frame motion prior from a 2D affine alignment of sonar features.
//
// The direct photometric solve in Track has a convergence basin of a couple of
// pixels, so it only works when it is handed an initial guess that is already
// close.  With an external odometry prior that guess comes for free; without one
// the constant-velocity model has to bootstrap itself from zero motion, which it
// cannot do -- a failed solve leaves the velocity at zero, which seeds the next
// solve at zero as well.  This estimates the guess straight from the imagery
// instead: track the frame's gradient keypoints into the next image, fit a rigid
// 2D transform to the correspondences, and lift that to SE3.
//

#ifndef SRC_AFFINESEED_H
#define SRC_AFFINESEED_H

#include <Eigen/Core>
#include <Eigen/Geometry>

class Frame;

struct AffineSeedParams
{
    //estimate the seed at all; when false Track falls back to constant velocity
    bool enabled = true;
    //Lucas-Kanade search window (pixels, square) and pyramid depth
    int lk_window = 21;
    int lk_levels = 3;
    //a point is kept only if tracking it forward then backward lands within this
    //many pixels of where it started
    double forward_backward_px = 1.0;
    //RANSAC reprojection threshold for the affine fit, in pixels
    double ransac_px = 2.0;
    //below this many RANSAC inliers the seed is rejected as unreliable
    int min_inliers = 20;
    //sanity limits; a seed beyond either is treated as a failed estimate
    double max_translation_m = 1.0;
    double max_yaw_deg = 45.0;
};

struct AffineSeedResult
{
    bool ok = false;
    //motion of the previous sonar frame into the current one, i.e. the same
    //T_sjcur_sipre the direct solver refines
    Eigen::Isometry3d T_cur_pre = Eigen::Isometry3d::Identity();
    //recovered in-plane rotation, radians (for logging/diagnostics)
    double yaw = 0.0;
    //points that survived the forward-backward flow check
    int tracked = 0;
    //of those, the ones RANSAC kept
    int inliers = 0;
    //why the estimate was rejected, empty when ok
    const char* reason = "";
};

//Estimate the relative motion between two frames from their imagery alone.
AffineSeedResult EstimateAffineSeed(const Frame &f_pre, const Frame &f_cur,
                                    const AffineSeedParams &params);

//Lift a 2x3 image-space transform mapping previous-image pixels onto
//current-image pixels into the sonar-frame SE3 that produces it.
//
//  p_pixel = t_p + M p_sonar,  M = scale * R(theta),  t_p = (tx, ty)
//
//so an in-plane rigid motion p_sonar' = R_s p_sonar + t_s appears in pixel space as
//
//  p_pixel' = M R_s M^-1 (p_pixel - t_p) + t_p + M t_s
//           = R_s (p_pixel - t_p) + t_p + M t_s
//
//because planar rotations commute with M.  The image rotation block therefore *is*
//R_s, and t_s = M^-1 [ t_A - (I - R_s) t_p ].  Only yaw and x/y are recoverable --
//the sonar model is planar (z = 0), so roll, pitch and z stay identity.
Eigen::Isometry3d AffineToSonarSE3(const Eigen::Matrix<double, 2, 3> &A, double theta, double tx,
                                   double ty, double scale, double* yaw_out = nullptr);

#endif //SRC_AFFINESEED_H
