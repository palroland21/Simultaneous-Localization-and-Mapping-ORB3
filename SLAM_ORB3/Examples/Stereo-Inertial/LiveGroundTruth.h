#ifndef ORB3_LIVE_GROUND_TRUTH_H
#define ORB3_LIVE_GROUND_TRUTH_H

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace orb3_live {

// Evaluation only: positions are body/IMU origins in their respective worlds.
// One rigid alignment is fitted to ALL pairs seen so far, with scale fixed to 1.
class GroundTruth {
public:
    struct Result {
        std::string status = "GT_NOT_LOADED";
        size_t samples = 0;
        double positionError = 0.0, rmse = 0.0, lag = 0.0;
        bool positionValid = false;
        bool valid() const { return status == "OK"; }
    };

    // Rebuild statistics from ONE consistent optimized snapshot. Earlier poses
    // can change after BA; never mix them with poses from an older snapshot.
    Result evaluate(const std::vector<std::pair<double, Eigen::Vector3d>>& trajectory,
                    double startTime, double endTime) {
        reset();
        Result result, latest;
        double latestPoseTime = -1.0;
        if (!points_.empty()) latest.status = "NO_GT_MATCH";
        for (const auto& pose : trajectory) {
            if (pose.first < startTime || pose.first > endTime) continue;
            latestPoseTime = pose.first;
            latest = update(pose.first, pose.second, true, true, 0, 0);
            if (latest.valid()) result = latest;
        }
        if (!result.valid()) return latest;
        // GT can end before the images: retain accumulated RMSE, but do not
        // present the last matched residual as the current position error.
        result.positionValid = latest.valid() && std::abs(latestPoseTime - endTime) < 1e-6;
        result.lag = latest.lag;
        return result;
    }

    bool load(const std::string& path) {
        points_.clear();
        reset();
        std::ifstream file(path.c_str());
        if (!file) return false;
        std::string line;
        while (std::getline(file, line)) {
            const size_t start = line.find_first_not_of(" \t\r");
            if (start == std::string::npos || line[start] == '#') continue;
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream row(line);
            Point point;
            if (!(row >> point.time >> point.position.x() >> point.position.y() >> point.position.z()) ||
                !std::isfinite(point.time) || !point.position.allFinite() || point.time < 0.0) {
                points_.clear();
                return false;
            }
            // EuRoC ASL CSV uses epoch nanoseconds; trajectory TXT uses seconds.
            if (point.time >= 1e12) point.time /= 1e9;
            if (!points_.empty() && point.time <= points_.back().time) {
                points_.clear();
                return false;
            }
            points_.push_back(point);
        }
        return !points_.empty();
    }

    void reset() {
        count_ = 0;
        havePrevious_ = false;
        lastMatch_ = -1.0;
        meanEstimate_.setZero(); meanTruth_.setZero();
        scatterEstimate_.setZero(); scatterTruth_.setZero(); cross_.setZero();
    }

    Result update(double time, const Eigen::Vector3d& bodyPosition, bool tracking,
                  bool imuInitialized, unsigned long mapId, unsigned long originId) {
        if (!haveMap_ || mapId != mapId_ || originId != originId_ ||
            (wasInitialized_ && !imuInitialized) ||
            (havePrevious_ && time <= previous_.time)) reset();
        haveMap_ = true; mapId_ = mapId; originId_ = originId;
        wasInitialized_ = imuInitialized;
        Result result;
        result.samples = count_;
        if (!tracking || !imuInitialized || !std::isfinite(time) || !bodyPosition.allFinite()) {
            havePrevious_ = false; // Never interpolate across a tracking gap.
            result.status = !tracking ? "WAIT_TRACKING" : "WAIT_IMU";
            return result;
        }
        const Point old = previous_;
        const bool hadPrevious = havePrevious_;
        previous_ = Point{time, bodyPosition}; havePrevious_ = true;
        if (points_.empty()) return result;
        result.status = "NO_GT_MATCH";
        auto next = std::upper_bound(points_.begin(), points_.end(), time,
            [](double t, const Point& p) { return t < p.time; });
        if (next == points_.begin()) return result;
        const Point& truth = *--next; // Only ground truth available at/before this frame.
        result.lag = time - truth.time;
        if (result.lag > 0.010001 || truth.time <= lastMatch_) return result;
        Eigen::Vector3d estimate = bodyPosition;
        if (result.lag > 1e-7) {
            if (!hadPrevious || old.time > truth.time || time - old.time > 0.2) return result;
            const double alpha = (truth.time - old.time) / (time - old.time);
            estimate = old.position + alpha * (bodyPosition - old.position);
        }
        lastMatch_ = truth.time;
        // Stable online centered moments; constant work/memory per evaluated frame.
        ++count_;
        const Eigen::Vector3d dx = estimate - meanEstimate_;
        const Eigen::Vector3d dy = truth.position - meanTruth_;
        meanEstimate_ += dx / double(count_);
        meanTruth_ += dy / double(count_);
        scatterEstimate_ += dx * (estimate - meanEstimate_).transpose();
        scatterTruth_ += dy * (truth.position - meanTruth_).transpose();
        cross_ += dx * (truth.position - meanTruth_).transpose();
        result.samples = count_;
        result.status = "WAIT_ALIGNMENT";
        if (count_ < 20) return result;
        // Reject stationary/collinear paths: rotation cannot yet be determined.
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ex(scatterEstimate_);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ey(scatterTruth_);
        if (ex.eigenvalues()[1] / count_ < 1e-4 || ey.eigenvalues()[1] / count_ < 1e-4)
            return result;
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross_, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d sign = Eigen::Matrix3d::Identity();
        sign(2,2) = (svd.matrixV() * svd.matrixU().transpose()).determinant() < 0.0 ? -1.0 : 1.0;
        const Eigen::Matrix3d rotation = svd.matrixV() * sign * svd.matrixU().transpose();
        result.positionError = (rotation * (estimate - meanEstimate_) -
                                (truth.position - meanTruth_)).norm();
        const double sse = scatterEstimate_.trace() + scatterTruth_.trace() -
                          2.0 * svd.singularValues().dot(sign.diagonal());
        result.rmse = std::sqrt(std::max(0.0, sse) / count_);
        result.status = "OK";
        result.positionValid = true;
        return result;
    }

private:
    struct Point { double time = 0.0; Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Point() = default;
        Point(double t, const Eigen::Vector3d& p) : time(t), position(p) {} };
    std::vector<Point> points_;
    Point previous_;
    bool havePrevious_ = false, haveMap_ = false, wasInitialized_ = false;
    unsigned long mapId_ = 0, originId_ = 0;
    size_t count_ = 0;
    double lastMatch_ = -1.0;
    Eigen::Vector3d meanEstimate_ = Eigen::Vector3d::Zero(), meanTruth_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d scatterEstimate_ = Eigen::Matrix3d::Zero(), scatterTruth_ = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d cross_ = Eigen::Matrix3d::Zero();
};
} // namespace orb3_live
#endif
