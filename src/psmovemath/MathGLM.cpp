//-- includes -----
#include "MathGLM.h"

//-- public methods -----
float glm_vec3_normalize_with_default(glm::vec3 &v, const glm::vec3 &default)
{
    const float length= glm::length(v);

    // Use the default value if v is too tiny
    v= (length > k_normal_epsilon) ? (v / length) : default;

    return length;
}

glm::vec3 glm_vec3_lerp(const glm::vec3 &a, const glm::vec3 &b, const float u)
{
    return a*(1.f-u) + b*u;
}