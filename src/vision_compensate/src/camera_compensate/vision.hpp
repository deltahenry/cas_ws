#pragma once
#include <opencv2/core.hpp>
#include <string>
namespace bfc {
    // Return codes (match your C#-style negatives; 0 = OK)
    enum : int {
        kOk = 0,
        kErrReadImage = -10,
        kErrChannelMissing = -11,
        kErrRightSideDetect = -2,
        kErrLeftSideDetect = -3,
        kErrFourPointsNotFound = -4,
        kErrLineIntersection = -5,
    };
    /**
     * Port of your C# EmguCV routine to C++17 + OpenCV.
     * Computes translational offsets X (image x) and Z (image y) by:
     *   threshold -> morphology -> CC stats -> robust edge fits -> band mask ->
     *   4 feature blobs -> order corners -> line intersections -> average delta.
     *
     * @param golden_path_name   path to golden color image
     * @param current_image_path path to current color image
     * @param X_out              (output) Δx = avg(golden corners) - avg(current corners)
     * @param Z_out              (output) Δz = avg(golden corners) - avg(current corners)
     * @return                   0 on success, negative error code on failure
     */
    int batteryFrameLocationCompensation(const std::string& golden_path_name,
        const std::string& current_image_path,
        double& X_out, double& Z_out);
    int batteryFrameLocationCompensation(const std::string& golden_path_name,
        cv::Mat current,
        double& X_out, double& Z_out);
} // namespace bfc
