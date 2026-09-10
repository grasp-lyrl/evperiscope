#pragma once

#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

namespace epa {

/**
 * Event-based Propeller Detector using exponential filtering and clustering.
 *
 * Pipeline:
 * 1. Exponential temporal filtering (first-order low-pass)
 * 2. Thresholding (mean + std)
 * 3. Morphological operations (dilate/erode)
 * 4. K-means initialization + nearest-centroid tracking
 * 5. Centroid distance stability check
 */
class EvPropDet {
public:
    /**
     * @param h Image height
     * @param w Image width
     * @param tau Time constant for exponential decay (seconds)
     * @param n_centroids Number of propeller blade centroids to track
     */
    EvPropDet(int h = 480, int w = 640, double tau = 0.5, int n_centroids = 4);

    /// Reset all internal state
    void reset();

    /// Read-only view of the filter accumulator, for tests and diagnostics
    const cv::Mat& debugState() const { return S_; }

    /**
     * Process a batch of events and detect propeller centroids.
     *
     * @param p Polarity array
     * @param t Timestamp array (seconds)
     * @param x X coordinate array
     * @param y Y coordinate array
     * @param n Number of events
     * @param t_end End time of the processing window
     */
    struct Result {
        cv::Mat image;                        ///< Visualization image (BGR, uint8)
        std::vector<Eigen::Vector2f> centroids; ///< Detected centroids (row, col)
        bool is_stable;                       ///< Whether detection is stable
    };

    Result processFrame(const float* p, const double* t, const int* x, const int* y,
                        int n, double t_end, bool render_image = true);

private:
    int h_, w_;
    double tau_;
    int n_centroids_;

    // Persistent filter state
    cv::Mat S_;         ///< Filter accumulator (h x w, float32)
    double last_t_end_; ///< Timestamp of the end of the previous processing window
    bool initialized_;

    // Exponential lookup table for fast processing
    std::vector<float> exp_lut_;
    double lut_resolution_;
    double lut_max_dt_;
    void precomputeExpLut(double max_dt, double resolution);

    // Tracking state
    std::vector<Eigen::Vector2f> previous_centroids_;
    int stable_count_;

    // Morphological kernels
    cv::Mat dilate_kernel_;
    cv::Mat erode_kernel_;

    /// Apply exponential filter to events
    void exponentialFilter(const float* p, const double* t, const int* x, const int* y,
                           int n, double t_end);

    /// Update centroids using nearest-centroid assignment
    std::vector<Eigen::Vector2f> updateCentroids(
        const std::vector<Eigen::Vector2f>& coords,
        const std::vector<Eigen::Vector2f>& prev_centroids);

    /// Compute all pairwise distances between centroids
    std::vector<float> computeCentroidDistances(const std::vector<Eigen::Vector2f>& centroids);
};

}  // namespace epa
