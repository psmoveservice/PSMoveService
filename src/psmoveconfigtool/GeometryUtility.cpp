#include "GeometryUtility.h"

//-- methods -----
glm::vec3 psmove_float_vector3_to_glm_vec3(const PSMoveFloatVector3 &v)
{
    return glm::vec3(v.i, v.j, v.k);
}

glm::vec3 psmove_position_to_glm_vec3(const PSMovePosition &v)
{
    return glm::vec3(v.x, v.y, v.z);
}
