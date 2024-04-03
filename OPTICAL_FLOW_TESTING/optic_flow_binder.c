/**
 * @file OPTICAL_FLOW_TESTING/optic_flow_binder.c
 * @brief Optical-flow estimation module
 *
 */

/* This file is a modified version of the existing opticflow_module.c already 
 * already implemented in the given computer_vison folder. The main adaptation
 * made are such that it is compatible with the changes made to optical_flow calculator
 * and the size_divergence_calculator. 
 *
 * An RMS calculator is also implemented to help stabalize the optical flow results received 
 * the calculator. The amount of data points able to be taken into account in the 
 * rms determination is adaptable through setting the RMS_LENGTH parameter.
*/

// Includes the necessary header files for functionality
#include "optic_flow_binder.h"
#include <stdio.h>
#include <pthread.h>
#include "state.h"
#include "modules/core/abi.h"
#include "modules/pose_history/pose_history.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"
#include "errno.h"

#include "cv.h"
#include "generated/airframe.h"

uint16_t fps_OF; // Global variable to store frames per second for optical flow

/* ABI messages sender ID */
#ifndef OPTICFLOW_AGL_ID
#define OPTICFLOW_AGL_ID ABI_BROADCAST    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(OPTICFLOW_AGL_ID)

#ifndef OPTICFLOW_FPS
#define OPTICFLOW_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif

#ifndef OPTICFLOW_FPS_CAMERA2
#define OPTICFLOW_FPS_CAMERA2 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(OPTICFLOW_FPS)
PRINT_CONFIG_VAR(OPTICFLOW_FPS_CAMERA2)

#ifdef OPTICFLOW_CAMERA2
#define ACTIVE_CAMERAS 2
#else
#define ACTIVE_CAMERAS 1
#endif

#define RMS_LENGTH 50 // Length of RMS (Root Mean Square) calculation array

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

// Variable declarations for divergence measurements
float left_div_size;
float right_div_size;
float total_div_size;
float rms_left;
float rms_right;
float rms_total;
int counter = 0;
DivergenceResult divergence_left_right_result;

// Calculate RMS for a given list
float calculate_rms(float *values, int size) {
    float sum_squared = 0.0f;

    // Sum the squares of the values
    for (int i = 0; i < size; i++) {
        sum_squared += values[i] * values[i];
    }

    // Calculate the mean of the squares
    float mean_squared = sum_squared / size;

    // Calculate the square root of the mean squared value
    float rms = sqrtf(mean_squared);

    return rms;
}

/* The main opticflow variables */
struct opticflow_t opticflow[ACTIVE_CAMERAS];                         ///< Opticflow calculations
static struct opticflow_result_t opticflow_result[ACTIVE_CAMERAS];    ///< The opticflow result

static bool opticflow_got_result[ACTIVE_CAMERAS];       ///< When we have an optical flow calculation
static pthread_mutex_t opticflow_mutex;                  ///< Mutex lock fo thread safety

/* Static functions */
struct image_t *opticflow_module_calc(struct image_t *img,
                                      uint8_t camera_id);     ///< The main optical flow calculation thread

#if PERIODIC_TELEMETRY
#include "modules/datalink/telemetry.h"
/**
 * Send optical flow telemetry information
 * @param[in] *trans The transport structure to send the information over
 * @param[in] *dev The link to send the data over
 */
static void opticflow_telem_send(struct transport_tx *trans, struct link_device *dev)
{
  pthread_mutex_lock(&opticflow_mutex);
  for (int idx_camera = 0; idx_camera < ACTIVE_CAMERAS; idx_camera++) {
    if (opticflow_result[idx_camera].noise_measurement < 0.8) {
      pprz_msg_send_OPTIC_FLOW_EST(trans, dev, AC_ID,
                                   &opticflow_result[idx_camera].fps, &opticflow_result[idx_camera].corner_cnt,
                                   &opticflow_result[idx_camera].tracked_cnt, &opticflow_result[idx_camera].flow_x,
                                   &opticflow_result[idx_camera].flow_y, &opticflow_result[idx_camera].flow_der_x,
                                   &opticflow_result[idx_camera].flow_der_y, &opticflow_result[idx_camera].vel_body.x,
                                   &opticflow_result[idx_camera].vel_body.y, &opticflow_result[idx_camera].vel_body.z,
                                   &opticflow_result[idx_camera].div_size,
                                   &opticflow_result[idx_camera].surface_roughness,
                                   &opticflow_result[idx_camera].divergence,
                                   &opticflow_result[idx_camera].camera_id); // TODO: no noise measurement here...
    }
  }
  pthread_mutex_unlock(&opticflow_mutex);
}
#endif

/**
 * Initialize the optical flow module for the bottom camera
 */
void opticflow_module_init(void)
{
  opticflow_got_result[0] = false; // Initialize the result flag to false

  // Initialize divergence result storage arrays to 0
  for (int i = 0; i < RMS_LENGTH; i++) {
  divergence_left_right_result.store_left_divergence[i] = 0.0f;
  divergence_left_right_result.store_right_divergence[i] = 0.0f;
  divergence_left_right_result.store_total_divergence[i] = 0.0f;
}
  opticflow_calc_init(opticflow); // Initialize opticflow calculation
  cv_add_to_device(&OPTICFLOW_CAMERA, opticflow_module_calc, OPTICFLOW_FPS, 0); // Add opticflow calculation to camera device
}

/**
 * Update the optical flow state for the calculation thread
 * and update the stabilization loops with the newest result
 */
void opticflow_module_run(void)
{
  pthread_mutex_lock(&opticflow_mutex);
  if (opticflow_got_result[0]) { // Check if a new result is available
      AbiSendMsgOPTICAL_FLOW(FLOW_OPTICFLOW_ID, 
                             rms_left,
                             rms_right,
                             rms_total);
      opticflow_got_result[0] = false; // Reset the result flag
  }
  pthread_mutex_unlock(&opticflow_mutex);
}

/**
 * The main optical flow calculation thread
 * This thread passes the images trough the optical flow
 * calculator
 * @param[in] *img The image_t structure of the captured image
 * @param[in] camera_id The camera index id
 * @return *img The processed image 
 */
struct image_t *opticflow_module_calc(struct image_t *img, uint8_t camera_id) {
    
    struct pose_t pose = get_rotation_at_timestamp(img->pprz_ts); // Get the pose at the image timestamp
    img->eulers = pose.eulers; // Store the pose Euler angles in the image structure

    if (counter >= RMS_LENGTH) { // Reset counter if it exceeds the RMS length
      counter = 0;
    } 

    static struct opticflow_result_t temp_result[ACTIVE_CAMERAS]; // Temporary result storage
    pthread_mutex_lock(&opticflow_mutex);
    // Perform optical flow calculation
    if (opticflow_calc_frame(&opticflow[camera_id], img, &temp_result[camera_id], &divergence_left_right_result)) {
        left_div_size = divergence_left_right_result.left_divergence; // Update divergence values and store them
        right_div_size = divergence_left_right_result.right_divergence;
        total_div_size = divergence_left_right_result.total_divergence;
        divergence_left_right_result.store_left_divergence[counter] = left_div_size;
        divergence_left_right_result.store_right_divergence[counter] = right_div_size;
        divergence_left_right_result.store_total_divergence[counter] = total_div_size;
        opticflow_got_result[0] = true;
    }
    pthread_mutex_unlock(&opticflow_mutex);

    // Calculate RMS values for left, right, and total divergence
    rms_left = calculate_rms(divergence_left_right_result.store_left_divergence, RMS_LENGTH);
    rms_right = calculate_rms(divergence_left_right_result.store_right_divergence, RMS_LENGTH);
    rms_total = calculate_rms(divergence_left_right_result.store_total_divergence, RMS_LENGTH);

    counter++;

    return img;
}