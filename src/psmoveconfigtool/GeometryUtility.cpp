#include "GeometryUtility.h"
#include "MathGLM.h"

//-- methods -----
// PSMove types to PSMove Types
PSMoveQuaternion psmove_matrix3x3_to_psmove_quaternion(const PSMoveMatrix3x3 &m)
{
    glm::mat3 glm_mat3 = psmove_matrix3x3_to_glm_mat3(m);
    glm::quat glm_quat = glm::quat_cast(glm_mat3);
    PSMoveQuaternion result = glm_mat3_to_psmove_quaternion(glm_quat);

    return result;
}

PSMoveMatrix3x3 psmove_quaternion_to_psmove_matrix3x3(const PSMoveQuaternion &q)
{
    glm::quat glm_quat= psmove_quaternion_to_glm_quat(q);
    glm::mat3 glm_mat3 = glm::mat3_cast(glm_quat);
    PSMoveMatrix3x3 result = glm_mat3_to_psmove_matrix3x3(glm_mat3);

    return result;
}

// PSMove types to GLM types
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

glm::mat3 psmove_matrix3x3_to_glm_mat3(const PSMoveMatrix3x3 &m)
{
    // GLM column matrix constructor
    return glm::mat3x3(
        psmove_float_vector3_to_glm_vec3(m.basis_x()),
        psmove_float_vector3_to_glm_vec3(m.basis_y()),
        psmove_float_vector3_to_glm_vec3(m.basis_z()));
}

glm::mat4
psmove_pose_to_glm_mat4(const PSMoveQuaternion &quat, const PSMovePosition &pos)
{
    glm::quat orientation(quat.w, quat.x, quat.y, quat.z);
    glm::vec3 position(pos.x, pos.y, pos.z);
    glm::mat4 transform = glm_mat4_from_pose(orientation, position);

    return transform;
}

glm::mat4
psmove_pose_to_glm_mat4(const PSMovePose &pose)
{
    return psmove_pose_to_glm_mat4(pose.Orientation, pose.Position);
}

// GLM Types to PSMove types
PSMoveFloatVector3 glm_vec3_to_psmove_float_vector3(const glm::vec3 &v)
{
    return PSMoveFloatVector3::create(v.x, v.y, v.z);
}

PSMovePosition glm_vec3_to_psmove_position(const glm::vec3 &v)
{
    return PSMovePosition::create(v.x, v.y, v.z);
}

PSMoveMatrix3x3 glm_mat3_to_psmove_matrix3x3(const glm::mat3 &m)
{
    // Basis vectors are stored in columns in GLM
    return
        PSMoveMatrix3x3::create(
            PSMoveFloatVector3::create(m[0].x, m[0].y, m[0].z),
            PSMoveFloatVector3::create(m[1].x, m[1].y, m[1].z),
            PSMoveFloatVector3::create(m[2].x, m[2].y, m[2].z));
}

PSMoveQuaternion glm_mat3_to_psmove_quaternion(const glm::quat &q)
{
    return PSMoveQuaternion::create(q.w, q.x, q.y, q.z);
}

PSMovePose glm_mat4_to_psmove_pose(const glm::mat4 &m)
{
    const glm::quat q = glm::quat_cast(m);
    const glm::vec3 p = glm::vec3(m[3]);
    PSMovePose result;

    result.Orientation = glm_mat3_to_psmove_quaternion(q);
    result.Position = glm_vec3_to_psmove_position(p);

    return result;
}

// PSMove types to OpenCV types
cv::Matx33f
psmove_matrix3x3_to_cv_mat33f(const PSMoveMatrix3x3 &in)
{
    // Both OpenCV and PSMoveMatrix3x3 matrices are stored row-major
    cv::Matx33f out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out(i, j) = in.m[i][j];
        }
    }

    return out;
}

// GLM Types to Eigen types
Eigen::Matrix3f glm_mat3_to_eigen_matrix3f(const glm::mat3 &m)
{
    const float *raw_m = glm::value_ptr(m);
    float copy_raw_m[3* 3];
    memcpy(copy_raw_m, raw_m, sizeof(copy_raw_m));
    Eigen::Map<Eigen::Matrix3f> result(copy_raw_m);

    return result;
}

Eigen::Matrix4f glm_mat4_to_eigen_matrix4f(const glm::mat4 &m)
{
    const float *raw_m = glm::value_ptr(m);
    float copy_raw_m[4 * 4];
    memcpy(copy_raw_m, raw_m, sizeof(copy_raw_m));
    Eigen::Map<Eigen::Matrix4f> result(copy_raw_m);

    return result;
}

// PSMoveTypes to Eigen types
Eigen::Vector3f psmove_int_vector3_to_eigen_vector3(const PSMoveIntVector3 &v)
{
    return Eigen::Vector3f(static_cast<float>(v.i), static_cast<float>(v.j), static_cast<float>(v.k));
}

Eigen::Vector3f psmove_float_vector3_to_eigen_vector3(const PSMoveFloatVector3 &v)
{
    return Eigen::Vector3f(v.i, v.j, v.k);
}

Eigen::Quaternionf psmove_quaternion_to_eigen_quaternionf(const PSMoveQuaternion &q)
{
    return Eigen::Quaternionf(q.w, q.x, q.y, q.z);
}

// Eigen types to GLM types
glm::mat3 eigen_matrix3f_to_glm_mat3(const Eigen::Matrix3f &m)
{
    return glm::make_mat3x3((const float *)m.data());
}

glm::mat4 eigen_matrix4f_to_glm_mat4(const Eigen::Matrix4f &m)
{
    return glm::make_mat4x4((const float *)m.data());
}

glm::vec3 eigen_vector3f_to_glm_vec3(const Eigen::Vector3f &v)
{
    return glm::vec3(v.x(), v.y(), v.z());
}
