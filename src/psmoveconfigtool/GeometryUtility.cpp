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

glm::quat psmove_quaternion_to_glm_quat(const PSMoveQuaternion &q)
{
    return glm::quat(q.w, q.x, q.y, q.z);
}

Eigen::Vector3f psmove_int_vector3_to_eigen_vector3(const PSMoveIntVector3 &v)
{
    return Eigen::Vector3f(static_cast<float>(v.i), static_cast<float>(v.j), static_cast<float>(v.k));
}

Eigen::Vector3f psmove_float_vector3_to_eigen_vector3(const PSMoveFloatVector3 &v)
{
    return Eigen::Vector3f(v.i, v.j, v.k);
}

glm::mat3 eigen_matrix3f_to_glm_mat3(const Eigen::Matrix3f &m)
{
    return glm::make_mat3x3((const float *)m.data());
}

glm::vec3 eigen_vector3f_to_glm_vec3(const Eigen::Vector3f &v)
{
    return glm::vec3(v.x(), v.y(), v.z());
}
