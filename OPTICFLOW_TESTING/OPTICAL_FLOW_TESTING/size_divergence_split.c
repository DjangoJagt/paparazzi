/**
 * @file modules/computer_vision/opticflow/size_divergence.c
 * @brief Calculate divergence from flow vectors by looking at line sizes between the points.
 *
 * Uses optical flow vectors as determined with a corner tracker and Lucas Kanade to estimate divergence. 
 * A split of the image is used to calculate the divergence of the left and right half of the image.
 */

#include "size_divergence_split.h"
#include <stdlib.h>

#define PRINT(string, ...) fprintf(stderr, "[mav_exercise->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

/**
 * Calculates divergence from optical flow vectors by analyzing the change in distance between corner points.
 * Divergence is calculated for the entire image, as well as separately for the left and right halves,
 * to provide insight into directional movement within the field of view.
 *
 * @param[in] vectors    The optical flow vectors representing movement of corner points.
 * @param[in] count      The total number of optical flow vectors.
 * @param[in] n_samples  The number of line segments to consider for calculating divergence.
 *                       If set to 0, all line segments formed by the vectors are considered.
 * @param[in,out] div_result The structure to store calculated divergence values.
 */
void get_size_divergence(struct flow_t *vectors, int count, int n_samples, DivergenceResult *div_result)
{
  // Initialize variables to store distances, sums of divergences, and sample counts
  float distance_1, distance_2;
  float divs_sum = 0.f;
  float divs_sum_left = 0.f;
  float divs_sum_right = 0.f;
  uint32_t used_samples = 0;
  uint32_t used_samples_left = 0;
  uint32_t used_samples_right = 0;

  float dx, dy;
  int32_t i, j;

  // Calculate the maximum number of samples based on the count of vectors
  int32_t max_samples = (count * count - count) / 2;

  // Set divergence results to 0 if there are less than 2 vectors
  if (count < 2) {
      div_result->total_divergence = 0.f;
      div_result->right_divergence = 0.f;
      div_result->left_divergence = 0.f;
      return;
  } else if (count >= max_samples) {
    n_samples = 0;
  }

  // A threshold to split the image into left and right for divergence calculation
  float center_y = 50000;

  // Calculate divergence for all line segments if n_samples is 0
  if (n_samples == 0) {
    // go through all possible lines:
    for (i = 0; i < count; i++) {
      for (j = i + 1; j < count; j++) {

        // distance in previous image:
        dx = (float)vectors[i].pos.x - (float)vectors[j].pos.x;
        dy = (float)vectors[i].pos.y - (float)vectors[j].pos.y;
        distance_1 = sqrtf(dx * dx + dy * dy);

        if (distance_1 < 1E-5) {
          continue;
        }

        // distance in current image:
        dx = (float)vectors[i].pos.x + (float)vectors[i].flow_x - (float)vectors[j].pos.x - (float)vectors[j].flow_x;
        dy = (float)vectors[i].pos.y + (float)vectors[i].flow_y - (float)vectors[j].pos.y - (float)vectors[j].flow_y;
        distance_2 = sqrtf(dx * dx + dy * dy);

        // Determine if the vector points are on the left or right side and accumulate divergence
        if ((float)vectors[i].pos.y >= center_y && (float)vectors[j].pos.y >= center_y) {
          divs_sum_left += (distance_2 - distance_1) / distance_1;
          used_samples_left++;
          
        } else {
          divs_sum_right += (distance_2 - distance_1) / distance_1;
          used_samples_right++;
        }

        divs_sum += (distance_2 - distance_1) / distance_1;
        used_samples++;
      }
    }
  } else {
    // take random samples:
    for (uint16_t sample = 0; sample < n_samples; sample++) {
      // take two random indices:
      i = rand() % count;
      j = rand() % count;
      // ensure it is not the same index:
      while (i == j) {
        j = rand() % count;
      }

      // distance in previous image:
      dx = (float)vectors[i].pos.x - (float)vectors[j].pos.x;
      dy = (float)vectors[i].pos.y - (float)vectors[j].pos.y;
      distance_1 = sqrtf(dx * dx + dy * dy);

      if (distance_1 < 1E-5) {
        continue;
      }

      // distance in current image:
      dx = (float)vectors[i].pos.x + (float)vectors[i].flow_x - (float)vectors[j].pos.x - (float)vectors[j].flow_x;
      dy = (float)vectors[i].pos.y + (float)vectors[i].flow_y - (float)vectors[j].pos.y - (float)vectors[j].flow_y;
      distance_2 = sqrtf(dx * dx + dy * dy);
    
    // Determine if the vector points are on the left or right side and accumulate divergence
    if ((float)vectors[i].pos.y >= center_y && (float)vectors[j].pos.y >= center_y) {
          divs_sum_left += (distance_2 - distance_1) / distance_1;
          used_samples_left++;
        } 

        else {
          divs_sum_right += (distance_2 - distance_1) / distance_1;
          used_samples_right++;
        }
        divs_sum += (distance_2 - distance_1) / distance_1;
        used_samples++;
  }
  }

  // Calculate average divergence for total, left, and right segments and store in div_result
  div_result->total_divergence = used_samples > 0 ? divs_sum / used_samples : 0.f;
  div_result->right_divergence = used_samples_right > 0 ? divs_sum_right / used_samples_right : 0.f;
  div_result->left_divergence = used_samples_left > 0 ? divs_sum_left / used_samples_left : 0.f;
}
