/**
 * @file modules/computer_vision/opticflow/opticflow_calculator.h
 * @brief Calculate velocity from optic flow.
 *
 * Using images from a vertical camera and IMU sensor data.
 */



#ifndef OPTICFLOW_CALCULATOR_H
#define OPTICFLOW_CALCULATOR_H

#include "std.h"
#include "inter_thread_data.h"
#include "lib/vision/image.h"
#include "lib/v4l/v4l2.h"
#include "divergence_common.h" // Include the common header

struct opticflow_t {
  bool got_first_img;                 ///< If we got a image to work with
  bool just_switched_method;        ///< Boolean to check if methods has been switched (for reinitialization)
  struct image_t img_gray;              ///< Current gray image frame
  struct image_t prev_img_gray;         ///< Previous gray image frame

  uint8_t method;                   ///< Method to use to calculate the optical flow
  uint8_t corner_method;            ///< Method to use for determining where the corners are
  uint16_t window_size;               ///< Window size for the blockmatching algorithm (general value for all methods)
  uint16_t search_distance;           ///< Search distance for blockmatching alg.
  bool derotation;                    ///< Derotation switched on or off (depended on the quality of the gyroscope measurement)
  bool median_filter;                 ///< Decides to use a median filter on the velocity

  float derotation_correction_factor_x;     ///< Correction factor for derotation in x axis, determined from a fit from the gyros and flow rotation. (wrong FOV, camera not in center)
  float derotation_correction_factor_y;     ///< Correction factor for derotation in Y axis, determined from a fit from the gyros and flow rotation. (wrong FOV, camera not in center)

  bool track_back;                    ///< Whether to track flow vectors back to the previous image, in order to check if the back-tracked flow leads to the original corner position.
  bool show_flow;                     ///< Whether to draw the flow vectors on the image. Watch out! This changes the image as will be received by subsequent processing steps.

  uint16_t subpixel_factor;                 ///< The amount of subpixels per pixel
  uint16_t resolution_factor;                 ///< The resolution in EdgeFlow to determine the Divergence
  uint8_t max_iterations;               ///< The maximum amount of iterations the Lucas Kanade algorithm should do
  uint8_t threshold_vec;                ///< The threshold in x, y subpixels which the algorithm should stop
  uint8_t pyramid_level;              ///< Number of pyramid levels used in Lucas Kanade algorithm (0 == no pyramids used)

  uint16_t max_track_corners;            ///< Maximum amount of corners Lucas Kanade should track
  bool fast9_adaptive;                  ///< Whether the FAST9 threshold should be adaptive
  uint8_t fast9_threshold;              ///< FAST9 corner detection threshold
  uint16_t fast9_min_distance;          ///< Minimum distance in pixels between corners
  uint16_t fast9_padding;               ///< Padding used in FAST9 detector

  uint16_t fast9_rsize;                 ///< Amount of corners allocated
  struct point_t *fast9_ret_corners;    ///< Corners
  bool feature_management;        ///< Decides whether to keep track corners in memory for the next frame instead of re-detecting every time
  bool fast9_region_detect;       ///< Decides whether to detect fast9 corners in specific regions of interest or the whole image (only for feature management)
  uint8_t fast9_num_regions;      ///< The number of regions of interest the image is split into

  float actfast_long_step;        ///< Step size to take when there is no texture
  float actfast_short_step;       ///< Step size to take when there is an edge to be followed
  int actfast_min_gradient;       ///< Threshold that decides when there is sufficient texture for edge following
  int actfast_gradient_method;    ///< Whether to use a simple or Sobel filter

  uint16_t fps;

  const struct video_config_t *camera;
  uint8_t id;
};

#define FAST9_MAX_CORNERS 512

extern void opticflow_calc_init(struct opticflow_t opticflow[]);
extern bool opticflow_calc_frame(struct opticflow_t *opticflow, struct image_t *img,
                          struct opticflow_result_t *result, DivergenceResult *div_result);

extern bool calc_fast9_lukas_kanade(struct opticflow_t *opticflow, struct image_t *img,
                             struct opticflow_result_t *result, DivergenceResult *div_result);
extern bool calc_edgeflow_tot(struct opticflow_t *opticflow, struct image_t *img,
                              struct opticflow_result_t *result);

extern void kalman_filter_opticflow_velocity(float *velocity_x, float *velocity_y, float *acceleration_measurement,
    float fps,
    float *measurement_noise, float process_noise, bool reinitialize_kalman);

#endif /* OPTICFLOW_CALCULATOR_H */


