//
// Created by wirano on 23-4-18.
//

#include "foc_utils.h"
#include <math.h>
#include "math_table.h"


float angle_normalize(float angle) {
    float normalized = fmodf(angle, _2PI);
    return normalized >= 0 ? normalized : (normalized + M_2_PI);
}
