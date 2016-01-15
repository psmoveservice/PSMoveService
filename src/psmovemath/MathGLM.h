#ifndef MATH_GLM_H
#define MATH_GLM_H

//-- includes -----
#include <glm/glm.hpp>

#include "MathUtility.h"

//-- interface -----
float glm_vec3_normalize_with_default(glm::vec3 &v, const glm::vec3 &default);
glm::vec3 glm_vec3_lerp(const glm::vec3 &a, const glm::vec3 &b, const float u);

#endif // MATH_GLM_H