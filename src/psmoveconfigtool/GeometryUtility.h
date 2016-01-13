#ifndef GEOMETRY_UTILITY_H
#define GEOMETRY_UTILITY_H

//-- includes -----
#include "ClientGeometry.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

//-- methods -----
glm::vec3 psmove_float_vector3_to_glm_vec3(const PSMoveFloatVector3 &v);
glm::vec3 psmove_position_to_glm_vec3(const PSMovePosition &v);
glm::quat psmove_quaternion_to_glm_quat(const PSMoveQuaternion &q);

#endif // GEOMETRY_UTILITY_H