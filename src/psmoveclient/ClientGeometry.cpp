//-- includes -----
#include "ClientGeometry.h"
#include "MathUtility.h"
#include "MathGLM.h"
#include <algorithm>

//-- pre-declarations -----

//-- constants -----
const PSMoveFloatVector3 g_psmove_float_vector3_zero= {0.f, 0.f, 0.f};
const PSMoveFloatVector3 *k_psmove_float_vector3_zero= &g_psmove_float_vector3_zero;

const PSMoveFloatVector3 g_psmove_float_vector3_one= {1.f, 1.f, 1.f};
const PSMoveFloatVector3 *k_psmove_float_vector3_one= &g_psmove_float_vector3_one;

const PSMoveFloatVector3 g_psmove_float_vector3_i = { 0.f, 0.f, 0.f };
const PSMoveFloatVector3 *k_psmove_float_vector3_i = &g_psmove_float_vector3_i;

const PSMoveFloatVector3 g_psmove_float_vector3_j = { 0.f, 0.f, 0.f };
const PSMoveFloatVector3 *k_psmove_float_vector3_j = &g_psmove_float_vector3_j;

const PSMoveFloatVector3 g_psmove_float_vector3_k = { 0.f, 0.f, 0.f };
const PSMoveFloatVector3 *k_psmove_float_vector3_k = &g_psmove_float_vector3_k;

const PSMoveIntVector3 g_psmove_int_vector3_zero= {0, 0, 0};
const PSMoveIntVector3 *k_psmove_int_vector3_zero= &g_psmove_int_vector3_zero;

const PSMoveIntVector3 g_psmove_int_vector3_one= {1, 1, 1};
const PSMoveIntVector3 *k_psmove_int_vector3_one= &g_psmove_int_vector3_one;

const PSMovePosition g_psmove_position_origin= {0.f, 0.f, 0.f};
const PSMovePosition *k_psmove_position_origin= &g_psmove_position_origin;

const PSMoveQuaternion g_psmove_quaternion_identity= {1.f, 0.f, 0.f, 0.f};
const PSMoveQuaternion *k_psmove_quaternion_identity= &g_psmove_quaternion_identity;

const PSMoveMatrix3x3 g_psmove_matrix_identity = { 1.f, 0.f, 0.f , 0.f, 1.f, 0.f, 0.f, 0.f, 1.f };
const PSMoveMatrix3x3 *k_psmove_matrix_identity = &g_psmove_matrix_identity;

const PSMovePose g_psmove_pose_identity = { g_psmove_quaternion_identity, g_psmove_position_origin };
const PSMovePose *k_psmove_pose_identity = &g_psmove_pose_identity;

//-- methods -----

// -- PSMoveFloatVector3 --
PSMoveFloatVector2 PSMoveFloatVector2::create(float i, float j)
{
    PSMoveFloatVector2 v;

    v.i = i;
    v.j = j;

    return v;
}

PSMoveFloatVector2 PSMoveFloatVector2::operator + (const PSMoveFloatVector2 &other) const
{
    return PSMoveFloatVector2::create(i + other.i, j + other.j);
}

PSMoveFloatVector2 PSMoveFloatVector2::operator - (const PSMoveFloatVector2 &other) const
{
    return PSMoveFloatVector2::create(i - other.i, j - other.j);
}

PSMoveFloatVector2 PSMoveFloatVector2::operator * (const float s) const
{
    return PSMoveFloatVector2::create(i*s, j*s);
}

PSMoveFloatVector2 PSMoveFloatVector2::unsafe_divide(const float s) const
{
    return PSMoveFloatVector2::create(i / s, j / s);
}

PSMoveFloatVector2 PSMoveFloatVector2::unsafe_divide(const PSMoveFloatVector2 &v) const
{
    return PSMoveFloatVector2::create(i / v.i, j / v.j);
}

PSMoveFloatVector2 PSMoveFloatVector2::safe_divide(const float s, const PSMoveFloatVector2 &default_result) const
{
    return !is_nearly_zero(s) ? unsafe_divide(s) : default_result;
}

PSMoveFloatVector2 PSMoveFloatVector2::safe_divide(const PSMoveFloatVector2 &v, const PSMoveFloatVector2 &default_result) const
{
    return
        PSMoveFloatVector2::create(
            !is_nearly_zero(v.i) ? i / v.i : default_result.i,
            !is_nearly_zero(v.j) ? j / v.j : default_result.j);
}

float PSMoveFloatVector2::length() const
{
    return sqrtf(i*i + j*j);
}

float PSMoveFloatVector2::normalize_with_default(const PSMoveFloatVector2 &default_result)
{
    const float divisor = length();

    *this = this->safe_divide(divisor, default_result);

    return divisor;
}

float PSMoveFloatVector2::minValue() const
{
    return std::min(i, j);
}

float PSMoveFloatVector2::maxValue() const
{
    return std::max(i, j);
}

float PSMoveFloatVector2::dot(const PSMoveFloatVector2 &a, const PSMoveFloatVector2 &b)
{
    return a.i*b.i + a.j*b.j;
}

PSMoveFloatVector2 PSMoveFloatVector2::min(const PSMoveFloatVector2 &a, const PSMoveFloatVector2 &b)
{
    return PSMoveFloatVector2::create(std::min(a.i, b.i), std::min(a.j, b.j));
}

PSMoveFloatVector2 PSMoveFloatVector2::max(const PSMoveFloatVector2 &a, const PSMoveFloatVector2 &b)
{
    return PSMoveFloatVector2::create(std::max(a.i, b.i), std::max(a.j, b.j));
}

// -- PSMoveFloatVector3 --
PSMoveFloatVector3 PSMoveFloatVector3::create(float i, float j, float k)
{
    PSMoveFloatVector3 v;

    v.i= i;
    v.j= j;
    v.k= k;

    return v;
}

PSMovePosition PSMoveFloatVector3::castToPSMovePosition() const
{
    return PSMovePosition::create(i, j, k);
}

PSMoveFloatVector3 PSMoveFloatVector3::operator + (const PSMoveFloatVector3 &other) const
{
    return PSMoveFloatVector3::create(i + other.i, j + other.j, k + other.k);
}

PSMoveFloatVector3 PSMoveFloatVector3::operator - (const PSMoveFloatVector3 &other) const
{
    return PSMoveFloatVector3::create(i - other.i, j - other.j, k - other.k);
}

PSMoveFloatVector3 PSMoveFloatVector3::operator * (const float s) const
{
    return PSMoveFloatVector3::create(i*s, j*s, k*s);
}

PSMoveFloatVector3 PSMoveFloatVector3::unsafe_divide(const float s) const
{
    return PSMoveFloatVector3::create(i/s, j/s, k/s);
}

PSMoveFloatVector3 PSMoveFloatVector3::unsafe_divide(const PSMoveFloatVector3 &v) const
{
    return PSMoveFloatVector3::create(i/v.i, j/v.j, k/v.k);
}

PSMoveFloatVector3 PSMoveFloatVector3::safe_divide(const float s, const PSMoveFloatVector3 &default_result) const
{
    return !is_nearly_zero(s) ? unsafe_divide(s) : default_result;
}

PSMoveFloatVector3 PSMoveFloatVector3::safe_divide(const PSMoveFloatVector3 &v, const PSMoveFloatVector3 &default_result) const
{
    return 
        PSMoveFloatVector3::create(
            !is_nearly_zero(v.i) ? i/v.i : default_result.i,
            !is_nearly_zero(v.j) ? j/v.j : default_result.j,
            !is_nearly_zero(v.k) ? k/v.k : default_result.k);
}

float PSMoveFloatVector3::length() const
{
    return sqrtf(i*i + j*j + k*k);
}

float PSMoveFloatVector3::normalize_with_default(const PSMoveFloatVector3 &default_result)
{
    const float divisor= length();
    
    *this= this->safe_divide(divisor, default_result);

    return divisor;
}

float PSMoveFloatVector3::minValue() const
{
    return std::min(std::min(i, j), k);
}

float PSMoveFloatVector3::maxValue() const
{
    return std::max(std::max(i, j), k);
}

float PSMoveFloatVector3::dot(const PSMoveFloatVector3 &a, const PSMoveFloatVector3 &b)
{
    return a.i*b.i + a.j*b.j + a.k*b.k;
}

PSMoveFloatVector3 PSMoveFloatVector3::min(const PSMoveFloatVector3 &a, const PSMoveFloatVector3 &b)
{
    return PSMoveFloatVector3::create(std::min(a.i, b.i), std::min(a.j, b.j), std::min(a.k, b.k));
}

PSMoveFloatVector3 PSMoveFloatVector3::max(const PSMoveFloatVector3 &a, const PSMoveFloatVector3 &b)
{
    return PSMoveFloatVector3::create(std::max(a.i, b.i), std::max(a.j, b.j), std::max(a.k, b.k));
}

// -- PSMoveIntVector3 -- 
PSMoveIntVector3 PSMoveIntVector3::create(int i, int j, int k)
{
    PSMoveIntVector3 v;

    v.i= i;
    v.j= j;
    v.k= k;

    return v;
}

PSMoveFloatVector3 PSMoveIntVector3::castToFloatVector3() const
{
    return PSMoveFloatVector3::create(static_cast<float>(i), static_cast<float>(j), static_cast<float>(k));
}

PSMoveIntVector3 PSMoveIntVector3::operator + (const PSMoveIntVector3 &other) const
{
    return PSMoveIntVector3::create(i + other.i, j + other.j, k + other.k);
}

PSMoveIntVector3 PSMoveIntVector3::operator - (const PSMoveIntVector3 &other) const
{
    return PSMoveIntVector3::create(i - other.i, j - other.j, k - other.k);
}

PSMoveIntVector3 PSMoveIntVector3::unsafe_divide(const int s) const
{
    return PSMoveIntVector3::create(i/s, j/s, k/s);
}

PSMoveIntVector3 PSMoveIntVector3::unsafe_divide(const PSMoveIntVector3 &v) const
{
    return PSMoveIntVector3::create(i/v.i, j/v.j, k/v.k);
}

PSMoveIntVector3 PSMoveIntVector3::safe_divide(const int s, const PSMoveIntVector3 &default_result) const
{
    return s != 0 ? unsafe_divide(s) : default_result;
}

PSMoveIntVector3 PSMoveIntVector3::safe_divide(const PSMoveIntVector3 &v, const PSMoveIntVector3 &default_result) const
{
    return 
        PSMoveIntVector3::create(
            v.i != 0 ? i/v.i : default_result.i,
            v.j != 0 ? j/v.j : default_result.j,
            v.k != 0 ? k/v.k : default_result.k);
}

int PSMoveIntVector3::lengthSquared() const
{
    return i*i + j*j + k*k;
}

int PSMoveIntVector3::minValue() const
{
    return std::min(std::min(i, j), k);
}

int PSMoveIntVector3::maxValue() const
{
    return std::max(std::max(i, j), k);
}

PSMoveIntVector3 PSMoveIntVector3::min(const PSMoveIntVector3 &a, const PSMoveIntVector3 &b)
{
    return PSMoveIntVector3::create(std::min(a.i, b.i), std::min(a.j, b.j), std::min(a.k, b.k));
}

PSMoveIntVector3 PSMoveIntVector3::max(const PSMoveIntVector3 &a, const PSMoveIntVector3 &b)
{
    return PSMoveIntVector3::create(std::max(a.i, b.i), std::max(a.j, b.j), std::max(a.k, b.k));
}

// -- PSMovePosition -- 
PSMovePosition PSMovePosition::create(float x, float y, float z)
{
    PSMovePosition p;

    p.x= x;
    p.y= y;
    p.z= z;

    return p;
}

PSMoveFloatVector3 PSMovePosition::toPSMoveFloatVector3() const
{
    return PSMoveFloatVector3::create(x, y, z);
}

PSMoveFloatVector3 PSMovePosition::operator - (const PSMovePosition &other) const
{
    return PSMoveFloatVector3::create(x - other.x, y - other.y, z - other.z);
}

PSMovePosition PSMovePosition::operator * (const float s) const
{
    return PSMovePosition::create(x*s, y*s, z*s);
}

// -- PSMovePosition -- 
PSMoveScreenLocation PSMoveScreenLocation::create(float x, float y)
{
    PSMoveScreenLocation p;

    p.x = x;
    p.y = y;

    return p;
}

PSMoveFloatVector2 PSMoveScreenLocation::toPSMoveFloatVector2() const
{
    return PSMoveFloatVector2::create(x, y);
}

PSMoveFloatVector2 PSMoveScreenLocation::operator - (const PSMoveScreenLocation &other) const
{
    return PSMoveFloatVector2::create(x - other.x, y - other.y);
}

// -- PSMoveQuaternion -- 
// psuedo-constructor to keep this a POD type
PSMoveQuaternion PSMoveQuaternion::create(float w, float x, float y, float z)
{
    PSMoveQuaternion q;

    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;

    return q;
}

PSMoveQuaternion PSMoveQuaternion::operator + (const PSMoveQuaternion &other) const
{
    return PSMoveQuaternion::create(w + other.w, x + other.x, y + other.y, z + other.z);
}

PSMoveQuaternion PSMoveQuaternion::unsafe_divide(const float s) const
{
    return PSMoveQuaternion::create(w / s, x / s, y / s, z / s);
}

PSMoveQuaternion PSMoveQuaternion::safe_divide(const float s, const PSMoveQuaternion &default_result) const
{
    return !is_nearly_zero(s) ? unsafe_divide(s) : default_result;
}

float PSMoveQuaternion::length() const
{
    return sqrtf(w*w + x*x + y*y + z*z);
}

PSMoveQuaternion & PSMoveQuaternion::normalize_with_default(const PSMoveQuaternion &default_result)
{
    const float divisor = length();

    *this = this->safe_divide(divisor, default_result);

    return *this;
}

// -- PSMoveMatrix3x3 --
PSMoveMatrix3x3 PSMoveMatrix3x3::create(
    const PSMoveFloatVector3 &basis_x,
    const PSMoveFloatVector3 &basis_y,
    const PSMoveFloatVector3 &basis_z)
{
    PSMoveMatrix3x3 mat;

    mat.m[0][0] = basis_x.i; mat.m[0][1] = basis_x.j; mat.m[0][2] = basis_x.k;
    mat.m[1][0] = basis_y.i; mat.m[1][1] = basis_y.j; mat.m[1][2] = basis_y.k;
    mat.m[2][0] = basis_z.i; mat.m[2][1] = basis_z.j; mat.m[2][2] = basis_z.k;

    return mat;
}

PSMoveFloatVector3 PSMoveMatrix3x3::basis_x() const
{
    return PSMoveFloatVector3::create(m[0][0], m[0][1], m[0][2]);
}

PSMoveFloatVector3 PSMoveMatrix3x3::basis_y() const
{
    return PSMoveFloatVector3::create(m[1][0], m[1][1], m[1][2]);
}

PSMoveFloatVector3 PSMoveMatrix3x3::basis_z() const
{
    return PSMoveFloatVector3::create(m[2][0], m[2][1], m[2][2]);
}

// -- PSMovePose -- 
void PSMovePose::Clear()
{
    Orientation= *k_psmove_quaternion_identity;
    Position= *k_psmove_position_origin;
}

// -- PSMoveFrustum -- 
void PSMoveFrustum::set_pose(const PSMovePose &pose)
{
    const PSMoveQuaternion &q = pose.Orientation;
    const glm::quat glm_quat(q.w, q.x, q.y, q.z);
    const glm::mat3 glm_mat3 = glm::mat3_cast(glm_quat);

    forward = PSMoveFloatVector3::create(glm_mat3[2].x, glm_mat3[2].y, glm_mat3[2].z); // z-axis
    left = PSMoveFloatVector3::create(glm_mat3[0].x, glm_mat3[0].y, glm_mat3[0].z); // x-axis
    up = PSMoveFloatVector3::create(glm_mat3[1].x, glm_mat3[1].y, glm_mat3[1].z); // y-axis

    origin = pose.Position;
}