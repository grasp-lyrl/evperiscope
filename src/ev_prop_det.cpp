#include "epa/ev_prop_det.hpp"
#include <algorithm>
#include <numeric>

namespace epa {

EvPropDet::EvPropDet(int h, int w, double tau, int n_centroids)
    : h_(h), w_(w), tau_(tau), n_centroids_(n_centroids),
      last_t_end_(-1.0), initialized_(false), stable_count_(0) {
    dilate_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    erode_kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // Precompute LUT for 100ms with 10us resolution
    precomputeExpLut(0.1, 10e-6);
}

void EvPropDet::precomputeExpLut(double max_dt, double resolution) {
    lut_max_dt_ = max_dt;
    lut_resolution_ = resolution;
    int size = static_cast<int>(std::ceil(max_dt / resolution)) + 1;
    exp_lut_.resize(size);
    for (int i = 0; i < size; ++i) {
        double dt = i * resolution;
        exp_lut_[i] = static_cast<float>(std::exp(-dt / tau_));
    }
}

void EvPropDet::reset() {
    S_ = cv::Mat();
    last_t_end_ = -1.0;
    previous_centroids_.clear();
    stable_count_ = 0;
    initialized_ = false;
}

void EvPropDet::exponentialFilter(const float* p, const double* t,
                                   const int* x, const int* y,
                                   int n, double t_end) {
    (void)p;  // All events treated as positive

    if (!initialized_) {
        S_ = cv::Mat::zeros(h_, w_, CV_32F);
        last_t_end_ = (n > 0) ? t[0] : t_end;
        initialized_ = true;
    }

    // 1. Global decay of the entire state matrix from the last batch
    double global_dt = t_end - last_t_end_;
    if (global_dt > 0.0) {
        S_ *= static_cast<float>(std::exp(-global_dt / tau_));
    }
    last_t_end_ = t_end;

    // 2. Add each event's pre-decayed contribution
    float* s_ptr = S_.ptr<float>();
    for (int i = 0; i < n; ++i) {
        int xi = x[i];
        int yi = y[i];

        if (xi < 0 || xi >= w_ || yi < 0 || yi >= h_) continue;

        double dt = t_end - t[i];
        if (dt < 0.0) dt = 0.0;

        float weight;
        if (dt <= lut_max_dt_) {
            int idx_lut = static_cast<int>(dt / lut_resolution_);
            if (idx_lut >= 0 && idx_lut < static_cast<int>(exp_lut_.size())) {
                weight = exp_lut_[idx_lut];
            } else {
                weight = static_cast<float>(std::exp(-dt / tau_));
            }
        } else {
            weight = static_cast<float>(std::exp(-dt / tau_));
        }

        int idx = yi * w_ + xi;
        s_ptr[idx] += weight;
    }
}

std::vector<Eigen::Vector2f> EvPropDet::updateCentroids(
    const std::vector<Eigen::Vector2f>& coords,
    const std::vector<Eigen::Vector2f>& prev_centroids) {

    int n_points = static_cast<int>(coords.size());
    int n_k = n_centroids_;
    std::vector<Eigen::Vector2f> current_centroids = prev_centroids;
    std::vector<int> labels(n_points);

    // Outlier rejection: maximum squared distance from centroid (e.g. 50 pixels radius)
    const float MAX_DIST_SQ = 2500.0f;

    // Run up to 5 iterations of K-Means to converge on the true centers
    const int MAX_ITERS = 5;
    for (int iter = 0; iter < MAX_ITERS; ++iter) {
        // Assign each point to nearest centroid
        for (int i = 0; i < n_points; ++i) {
            float min_dist_sq = std::numeric_limits<float>::infinity();
            int best_k = -1;
            for (int k = 0; k < n_k; ++k) {
                float dy = coords[i](0) - current_centroids[k](0);
                float dx = coords[i](1) - current_centroids[k](1);
                float dist_sq = dy * dy + dx * dx;
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    best_k = k;
                }
            }

            // Reject outliers that are too far from any propeller center
            if (min_dist_sq <= MAX_DIST_SQ) {
                labels[i] = best_k;
            } else {
                labels[i] = -1; // Outlier
            }
        }

        // Accumulate sums and counts
        std::vector<Eigen::Vector2d> sums(n_k, Eigen::Vector2d::Zero());
        std::vector<int> counts(n_k, 0);

        for (int i = 0; i < n_points; ++i) {
            int k = labels[i];
            if (k >= 0) {
                sums[k](0) += coords[i](0);
                sums[k](1) += coords[i](1);
                counts[k]++;
            }
        }

        // Compute means
        bool converged = true;
        for (int k = 0; k < n_k; ++k) {
            if (counts[k] > 0) {
                Eigen::Vector2f new_c(
                    static_cast<float>(sums[k](0) / counts[k]),
                    static_cast<float>(sums[k](1) / counts[k])
                );

                // Check if the centroid moved significantly
                float dy = new_c(0) - current_centroids[k](0);
                float dx = new_c(1) - current_centroids[k](1);
                if (dy * dy + dx * dx > 0.1f) {
                    converged = false;
                }
                current_centroids[k] = new_c;
            }
        }

        if (converged) {
            break;
        }
    }

    return current_centroids;
}

std::vector<float> EvPropDet::computeCentroidDistances(
    const std::vector<Eigen::Vector2f>& centroids) {
    int n = static_cast<int>(centroids.size());
    std::vector<float> dists;
    dists.reserve(n * (n - 1) / 2);
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            float dy = centroids[i](0) - centroids[j](0);
            float dx = centroids[i](1) - centroids[j](1);
            dists.push_back(std::sqrt(dy * dy + dx * dx));
        }
    }
    return dists;
}

EvPropDet::Result EvPropDet::processFrame(const float* p, const double* t,
                                           const int* x, const int* y,
                                           int n, double t_end, bool render_image) {
    Result result;
    result.is_stable = false;

    // Apply exponential filter
    exponentialFilter(p, t, x, y, n, t_end);

    // Threshold based on mean + std of active pixels
    cv::Mat viz = S_.clone();
    cv::Mat positive_mask = viz > 0;

    if (cv::countNonZero(positive_mask) > 0) {
        cv::Scalar mean_s, std_s;
        cv::meanStdDev(viz, mean_s, std_s, positive_mask);
        float threshold = static_cast<float>(mean_s[0] + std_s[0]);
        viz.setTo(0, viz < threshold);
        viz.setTo(1.0f, viz >= threshold);
    } else {
        viz.setTo(0);
    }

    // Morphological operations: Opening (Erode then Dilate) removes small noise
    cv::Mat viz_u8;
    viz.convertTo(viz_u8, CV_8U, 255.0);
    cv::morphologyEx(viz_u8, viz_u8, cv::MORPH_OPEN, erode_kernel_, cv::Point(-1,-1), 1);
    cv::dilate(viz_u8, viz_u8, dilate_kernel_, cv::Point(-1, -1), 1); // Expand propellers slightly

    // Convert to BGR for visualization
    if (render_image) {
        cv::cvtColor(viz_u8, result.image, cv::COLOR_GRAY2BGR);
    }

    // Find coordinates of the surviving blob pixels
    std::vector<cv::Point> locations;
    cv::findNonZero(viz_u8, locations);

    if (static_cast<int>(locations.size()) >= n_centroids_) {
        std::vector<Eigen::Vector2f> coords;
        coords.reserve(locations.size());
        for (const auto& pt : locations) {
            coords.emplace_back(static_cast<float>(pt.y), static_cast<float>(pt.x));
        }

        if (previous_centroids_.empty() ||
            static_cast<int>(previous_centroids_.size()) != n_centroids_) {
            // Initialize with OpenCV kmeans
            cv::Mat coords_mat(static_cast<int>(locations.size()), 2, CV_32F);
            for (size_t i = 0; i < locations.size(); ++i) {
                coords_mat.at<float>(static_cast<int>(i), 0) = static_cast<float>(locations[i].y);
                coords_mat.at<float>(static_cast<int>(i), 1) = static_cast<float>(locations[i].x);
            }
            cv::Mat labels_mat, centers;
            cv::kmeans(coords_mat, n_centroids_, labels_mat,
                       cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 20, 1.0),
                       1, cv::KMEANS_PP_CENTERS, centers);

            previous_centroids_.resize(n_centroids_);
            for (int k = 0; k < n_centroids_; ++k) {
                previous_centroids_[k](0) = centers.at<float>(k, 0);
                previous_centroids_[k](1) = centers.at<float>(k, 1);
            }
        }

        auto new_centroids = updateCentroids(coords, previous_centroids_);
        previous_centroids_ = new_centroids;
        result.centroids = new_centroids;

        if (render_image) {
            // Draw centroids (red)
            for (const auto& c : new_centroids) {
                cv::circle(result.image,
                           cv::Point(static_cast<int>(c(1)), static_cast<int>(c(0))),
                           5, cv::Scalar(0, 0, 255), -1);
            }
        }

        // Check centroid consistency (equidistant check)
        auto dists = computeCentroidDistances(new_centroids);
        if (!dists.empty()) {
            float mean_dist = std::accumulate(dists.begin(), dists.end(), 0.0f) /
                              static_cast<float>(dists.size());
            bool all_close = true;
            for (float d : dists) {
                if (std::abs(d - mean_dist) >= 0.5f * mean_dist) {
                    all_close = false;
                    break;
                }
            }
            if (all_close) {
                stable_count_++;
                result.is_stable = true;
            } else {
                stable_count_ = 0;
            }
        }

        // If stable for 5+ frames, draw blue
        if (render_image && stable_count_ >= 5 && result.is_stable) {
            for (const auto& c : new_centroids) {
                cv::circle(result.image,
                           cv::Point(static_cast<int>(c(1)), static_cast<int>(c(0))),
                           5, cv::Scalar(255, 0, 0), -1);
            }
        }

        result.is_stable = result.is_stable && (stable_count_ >= 5);
    } else {
        stable_count_ = 0;
    }

    return result;
}

}  // namespace epa
