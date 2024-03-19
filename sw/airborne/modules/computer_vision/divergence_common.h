// divergence_common.h

#ifndef DIVERGENCE_COMMON_H
#define DIVERGENCE_COMMON_H

typedef struct {
    float total_divergence;
    float right_divergence;
    float left_divergence;
    float store_left_divergence[50];
    float store_right_divergence[50];
    float store_total_divergence[50];
} DivergenceResult;

#endif // DIVERGENCE_COMMON_H
