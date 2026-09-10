"""Event-based propeller detector.

Reference Python implementation of ``epa::EvPropDet`` (``src/ev_prop_det.cpp``).
Both implementations run the same pipeline and are kept numerically equivalent:

1. Exponential temporal filtering (first-order low-pass over the event stream)
2. Thresholding at ``mean + std`` of the active pixels
3. Morphological opening followed by a dilation
4. k-means initialisation, then nearest-centroid tracking across frames
5. Equidistance check on the centroids to decide whether a detection is stable
"""

import numpy as np
import cv2


class EvPropDet:
    """Detects propeller blade centroids in a stream of events.

    The filter state persists across calls to :meth:`process_frame`, so batches
    must be fed in chronological order.
    """

    def __init__(self, h=480, w=640, tau=0.5, n_centroids=4):
        """
        Args:
            h: Image height in pixels.
            w: Image width in pixels.
            tau: Time constant of the exponential decay, in seconds.
            n_centroids: Number of propeller centroids to track.
        """
        self.h = h
        self.w = w
        self.tau = tau
        self.n_centroids = n_centroids

        # Persistent filter state
        self.S = None            # accumulator, (h, w) float32
        self.last_t_end = -1.0   # end of the previous processing window
        self.previous_centroids = None
        self.stable_count = 0

        # Exponential lookup table: 100 ms span at 10 us resolution
        self._lut_resolution = 10e-6
        self._lut_max_dt = 0.1
        n_lut = int(np.ceil(self._lut_max_dt / self._lut_resolution)) + 1
        self._exp_lut = np.exp(
            -np.arange(n_lut, dtype=np.float64) * self._lut_resolution / self.tau
        ).astype(np.float32)

        self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    def reset(self):
        """Reset all internal state."""
        self.S = None
        self.last_t_end = -1.0
        self.previous_centroids = None
        self.stable_count = 0

    def exponential_filter(self, p_events, t_events, x_events, y_events, t_end):
        """Accumulate one batch of events into the exponentially decaying state.

        Every event contributes ``exp(-(t_end - t_i) / tau)``, and the state
        carried over from the previous batch is decayed by the elapsed time.
        Polarity is ignored: all events count as positive.

        Args:
            p_events: Polarity array (unused, kept for interface symmetry).
            t_events: Timestamps in seconds, ascending.
            x_events: Column coordinates.
            y_events: Row coordinates.
            t_end: End of the current window, in seconds.

        Returns:
            The updated (h, w) float32 accumulator.
        """
        del p_events  # all events are treated as positive

        n = len(t_events)
        if self.S is None:
            self.S = np.zeros((self.h, self.w), dtype=np.float32)
            self.last_t_end = float(t_events[0]) if n > 0 else float(t_end)

        # Decay the whole state from the end of the previous window
        global_dt = t_end - self.last_t_end
        if global_dt > 0.0:
            self.S *= np.float32(np.exp(-global_dt / self.tau))
        self.last_t_end = float(t_end)

        if n == 0:
            return self.S

        x = np.asarray(x_events, dtype=np.int64)
        y = np.asarray(y_events, dtype=np.int64)
        valid = (x >= 0) & (x < self.w) & (y >= 0) & (y < self.h)
        if not np.any(valid):
            return self.S

        x = x[valid]
        y = y[valid]
        dt = np.maximum(t_end - np.asarray(t_events, dtype=np.float64)[valid], 0.0)

        # Table lookup for the common case, exact exponential beyond its range
        weights = np.exp(-dt / self.tau).astype(np.float32)
        in_lut = dt <= self._lut_max_dt
        if np.any(in_lut):
            idx = (dt[in_lut] / self._lut_resolution).astype(np.int64)
            idx = np.clip(idx, 0, len(self._exp_lut) - 1)
            weights[in_lut] = self._exp_lut[idx]

        # bincount handles repeated pixels, unlike fancy-indexed assignment
        flat = np.bincount(y * self.w + x, weights=weights.astype(np.float64),
                           minlength=self.h * self.w)
        self.S += flat.reshape(self.h, self.w).astype(np.float32)
        return self.S

    def _update_centroids(self, coords, previous_centroids):
        """Refine centroids with a few bounded k-means iterations.

        Points further than 50 px from every centroid are treated as outliers
        and excluded, which keeps background noise from dragging a cluster off
        its propeller.

        Args:
            coords: (N, 2) float32 array of (row, col) pixel coordinates.
            previous_centroids: (K, 2) float32 array to start from.

        Returns:
            (K, 2) float32 array of refined centroids.
        """
        MAX_DIST_SQ = 2500.0  # 50 px radius
        MAX_ITERS = 5
        CONVERGED_SQ = 0.1    # squared pixel movement below which we stop

        centroids = previous_centroids.astype(np.float32).copy()

        for _ in range(MAX_ITERS):
            diff = coords[:, None, :] - centroids[None, :, :]
            dist_sq = np.einsum('nkd,nkd->nk', diff, diff)
            labels = np.argmin(dist_sq, axis=1)
            inlier = dist_sq[np.arange(len(coords)), labels] <= MAX_DIST_SQ

            counts = np.bincount(labels[inlier], minlength=self.n_centroids)
            sums = np.stack([
                np.bincount(labels[inlier], weights=coords[inlier, d].astype(np.float64),
                            minlength=self.n_centroids)
                for d in range(2)
            ], axis=1)

            occupied = counts > 0
            if not np.any(occupied):
                break

            new_centroids = centroids.copy()
            new_centroids[occupied] = (
                sums[occupied] / counts[occupied, None]).astype(np.float32)

            moved = np.sum((new_centroids - centroids) ** 2, axis=1)
            centroids = new_centroids
            if not np.any(moved[occupied] > CONVERGED_SQ):
                break

        return centroids

    @staticmethod
    def _compute_centroid_distances(centroids):
        """Return all pairwise distances between centroids as a flat array."""
        i, j = np.triu_indices(len(centroids), k=1)
        return np.linalg.norm(centroids[i] - centroids[j], axis=1)

    def process_frame(self, p_events, t_events, x_events, y_events, t_end,
                      render_image=True):
        """Process one batch of events and detect propeller centroids.

        Args:
            p_events: Polarity array.
            t_events: Timestamps in seconds, ascending.
            x_events: Column coordinates.
            y_events: Row coordinates.
            t_end: End of the current window, in seconds.
            render_image: Whether to build the BGR debug visualisation.

        Returns:
            dict with keys:
                ``image``     -- BGR uint8 visualisation, or None if not rendered
                ``centroids`` -- (K, 2) float32 (row, col) centroids, or None
                ``is_stable`` -- True once the detection has been consistent
                                 for at least 5 consecutive frames
                ``S``         -- the raw filter accumulator
        """
        self.exponential_filter(p_events, t_events, x_events, y_events, t_end)

        # Threshold at mean + std over the pixels that carry any energy
        viz = self.S.copy()
        positive = viz > 0
        if np.any(positive):
            vals = viz[positive]
            threshold = vals.mean() + vals.std()
            viz = (viz >= threshold).astype(np.float32)
        else:
            viz[:] = 0

        # Opening removes speckle, the extra dilation grows the blades back
        viz_u8 = (viz * 255).astype(np.uint8)
        viz_u8 = cv2.morphologyEx(viz_u8, cv2.MORPH_OPEN, self._kernel, iterations=1)
        viz_u8 = cv2.dilate(viz_u8, self._kernel, iterations=1)

        image = cv2.cvtColor(viz_u8, cv2.COLOR_GRAY2BGR) if render_image else None

        coords = np.column_stack(np.nonzero(viz_u8)).astype(np.float32)  # (row, col)
        new_centroids = None
        is_stable = False

        if len(coords) >= self.n_centroids:
            if (self.previous_centroids is None or
                    self.previous_centroids.shape != (self.n_centroids, 2)):
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
                _, _, centers = cv2.kmeans(
                    coords, self.n_centroids, None, criteria, 1, cv2.KMEANS_PP_CENTERS)
                self.previous_centroids = centers.astype(np.float32)

            new_centroids = self._update_centroids(coords, self.previous_centroids)
            self.previous_centroids = new_centroids

            if render_image:
                for c in new_centroids.astype(np.int32):
                    cv2.circle(image, (c[1], c[0]), 5, (0, 0, 255), -1)

            # A real quadrotor gives four roughly equidistant centroids
            dists = self._compute_centroid_distances(new_centroids)
            if len(dists) > 0:
                mean_dist = dists.mean()
                if np.all(np.abs(dists - mean_dist) < 0.5 * mean_dist):
                    self.stable_count += 1
                    is_stable = True
                else:
                    self.stable_count = 0

            if render_image and is_stable and self.stable_count >= 5:
                for c in new_centroids.astype(np.int32):
                    cv2.circle(image, (c[1], c[0]), 5, (255, 0, 0), -1)
        else:
            self.stable_count = 0

        return {
            'image': image,
            'centroids': new_centroids,
            'is_stable': is_stable and self.stable_count >= 5,
            'S': self.S,
        }
