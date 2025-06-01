//
// Created by sophda on 2025/3/29.
//

#ifndef ACE_DSACSTAR_H
#define ACE_DSACSTAR_H
#include <ATen/ATen.h>

int dsacstar_rgb_forward(
        at::Tensor sceneCoordinatesSrc,
        at::Tensor outPoseSrc,
        int ransacHypotheses,
        float inlierThreshold,
        float focalLength,
        float ppointX,
        float ppointY,
        float inlierAlpha,
        float maxReproj,
        int subSampling);

#endif //ACE_DSACSTAR_H
