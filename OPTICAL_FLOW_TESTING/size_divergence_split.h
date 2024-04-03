/**
 * @file modules/computer_vision/opticflow/size_divergence.h
 * @brief Calculate divergence from flow vectors by looking at line sizes beteween the points.
 *
 * Uses optical flow vectors as determined with a corner tracker and Lucas Kanade to estimate divergence.
 */

#include "lib/vision/image.h"
#include "divergence_common.h" // Include the common header

#ifndef SIZE_DIVERGENCE
#define SIZE_DIVERGENCE


void get_size_divergence(struct flow_t *vectors, int count, int n_samples, DivergenceResult *div_result);
float get_mean(float *numbers, int n_elements);

#endif


