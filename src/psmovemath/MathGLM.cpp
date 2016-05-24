//-- includes -----
#include "MathGLM.h"

//-- public methods -----
float glm_vec3_normalize_with_default(glm::vec3 &v, const glm::vec3 &default_result)
{
    const float length= glm::length(v);

    // Use the default value if v is too tiny
    v= (length > k_normal_epsilon) ? (v / length) : default_result;

    return length;
}

glm::vec3 glm_vec3_lerp(const glm::vec3 &a, const glm::vec3 &b, const float u)
{
    return a*(1.f-u) + b*u;
}

glm::mat4 glm_mat4_from_pose(const glm::quat &orientation, const glm::vec3 &position)
{
    glm::mat4 rot = glm::mat4_cast(orientation);
    glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
    glm::mat4 transform = trans * rot;

    return transform;
}