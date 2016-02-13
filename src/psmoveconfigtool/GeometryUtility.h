#ifndef GEOMETRY_UTILITY_H
#define GEOMETRY_UTILITY_H

//-- includes -----
#include "ClientGeometry.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include "MathEigen.h"

//-- methods -----
glm::vec3 psmove_float_vector3_to_glm_vec3(const PSMoveFloatVector3 &v);
glm::vec3 psmove_position_to_glm_vec3(const PSMovePosition &v);
glm::quat psmove_quaternion_to_glm_quat(const PSMoveQuaternion &q);

Eigen::Vector3f psmove_int_vector3_to_eigen_vector3(const PSMoveIntVector3 &v);
Eigen::Vector3f psmove_float_vector3_to_eigen_vector3(const PSMoveFloatVector3 &v);

glm::mat3 eigen_matrix3f_to_glm_mat3(const Eigen::Matrix3f &m);
glm::vec3 eigen_vector3f_to_glm_vec3(const Eigen::Vector3f &v);

#endif // GEOMETRY_UTILITY_H