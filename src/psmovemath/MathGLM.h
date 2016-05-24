#ifndef MATH_GLM_H
#define MATH_GLM_H

//-- includes -----
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "MathUtility.h"

//-- macros -----
#define assert_glm_vector3f_is_normalized(v) assert(is_nearly_equal(v.length(), 1.f, k_normal_epsilon))
#define assert_glm_vectors_are_perpendicular(a,b) assert(is_nearly_equal(glm::dot(a,b), 0.f, k_normal_epsilon))

//-- interface -----
float glm_vec3_normalize_with_default(glm::vec3 &v, const glm::vec3 &default_result);
glm::vec3 glm_vec3_lerp(const glm::vec3 &a, const glm::vec3 &b, const float u);
glm::mat4 glm_mat4_from_pose(const glm::quat &orientation, const glm::vec3 &position);

#endif // MATH_GLM_H