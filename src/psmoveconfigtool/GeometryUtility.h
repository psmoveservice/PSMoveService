#ifndef GEOMETRY_UTILITY_H
#define GEOMETRY_UTILITY_H

//-- includes -----
#include "ClientGeometry.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <opencv2/opencv.hpp>
#include "MathEigen.h"

//-- methods -----
// PSMove types to PSMove Types
PSMoveQuaternion psmove_matrix3x3_to_psmove_quaternion(const PSMoveMatrix3x3 &m);
PSMoveMatrix3x3 psmove_quaternion_to_psmove_matrix3x3(const PSMoveQuaternion &q);

// PSMove types to GLM types
glm::vec3 psmove_float_vector3_to_glm_vec3(const PSMoveFloatVector3 &v);
glm::vec3 psmove_position_to_glm_vec3(const PSMovePosition &v);
glm::quat psmove_quaternion_to_glm_quat(const PSMoveQuaternion &q);
glm::mat3 psmove_matrix3x3_to_glm_mat3(const PSMoveMatrix3x3 &m);
glm::mat4 psmove_pose_to_glm_mat4(const PSMoveQuaternion &quat, const PSMovePosition &pos);
glm::mat4 psmove_pose_to_glm_mat4(const PSMovePose &pose);

// GLM Types to PSMove types
PSMoveFloatVector3 glm_vec3_to_psmove_float_vector3(const glm::vec3 &v);
PSMovePosition glm_vec3_to_psmove_position(const glm::vec3 &v);
PSMoveMatrix3x3 glm_mat3_to_psmove_matrix3x3(const glm::mat3 &m);
PSMoveQuaternion glm_mat3_to_psmove_quaternion(const glm::quat &q);
PSMovePose glm_mat4_to_psmove_pose(const glm::mat4 &m);

// GLM Types to Eigen types
Eigen::Matrix3f glm_mat3_to_eigen_matrix3f(const glm::mat3 &m);
Eigen::Matrix4f glm_mat4_to_eigen_matrix4f(const glm::mat4 &m);

// PSMove types to OpenCV types
cv::Matx33f psmove_matrix3x3_to_cv_mat33f(const PSMoveMatrix3x3 &in);

// PSMoveTypes to Eigen types
Eigen::Vector3f psmove_int_vector3_to_eigen_vector3(const PSMoveIntVector3 &v);
Eigen::Vector3f psmove_float_vector3_to_eigen_vector3(const PSMoveFloatVector3 &v);

// Eigen types to GLM types
glm::mat3 eigen_matrix3f_to_glm_mat3(const Eigen::Matrix3f &m);
glm::mat4 eigen_matrix4f_to_glm_mat4(const Eigen::Matrix4f &m);
glm::vec3 eigen_vector3f_to_glm_vec3(const Eigen::Vector3f &v);

#endif // GEOMETRY_UTILITY_H