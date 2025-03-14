

/**
 * @file modules/computer_vision/opticflow_module.h
 * @brief optical-flow calculation for Parrot Drones
 *
 */

#ifndef OPTICFLOW_MODULE_H
#define OPTICFLOW_MODULE_H

// Include opticflow calculator
#include "opticflow/opticflow_calculator.h"
#include "divergence_common.h" // Include the common header

// Needed for settings
extern struct opticflow_t opticflow[];

// Module functions
float calculate_rms(float *values, int size);
extern void opticflow_module_init(void);
extern void opticflow_module_run(void);
extern void opticflow_module_start(void);
extern void opticflow_module_stop(void);

#endif /* OPTICFLOW_MODULE_H */
