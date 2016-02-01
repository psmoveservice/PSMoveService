//-- includes -----
#include "ClientGeometry.h"
#include "MathUtility.h"
#include <algorithm>

//-- pre-declarations -----

//-- constants -----
const PSMoveFloatVector3 g_psmove_float_vector3_zero= {0.f, 0.f, 0.f};
const PSMoveFloatVector3 *k_psmove_float_vector3_zero= &g_psmove_float_vector3_zero;

const PSMoveFloatVector3 g_psmove_float_vector3_one= {1.f, 1.f, 1.f};
const PSMoveFloatVector3 *k_psmove_float_vector3_one= &g_psmove_float_vector3_one;

const PSMoveIntVector3 g_psmove_int_vector3_zero= {0, 0, 0};
const PSMoveIntVector3 *k_psmove_int_vector3_zero= &g_psmove_int_vector3_zero;

const PSMoveIntVector3 g_psmove_int_vector3_one= {1, 1, 1};
const PSMoveIntVector3 *k_psmove_int_vector3_one= &g_psmove_int_vector3_one;

const PSMovePosition g_psmove_position_origin= {0.f, 0.f, 0.f};
const PSMovePosition *k_psmove_position_origin= &g_psmove_position_origin;

const PSMoveQuaternion g_psmove_quaternion_identity= {1.f, 0.f, 0.f, 0.f};
const PSMoveQuaternion *k_psmove_quaternion_identity= &g_psmove_quaternion_identity;

//-- methods -----

// -- PSMoveFloatVector3 --
PSMoveFloatVector3 PSMoveFloatVector3::create(float i, float j, float k)
{
    PSMoveFloatVector3 v;

    v.i= i;
    v.j= j;
    v.k= k;

    return v;
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

PSMoveFloatVector3 PSMoveFloatVector3::safe_divide(const float s, const PSMoveFloatVector3 &default) const
{
    return !is_nearly_zero(s) ? unsafe_divide(s) : default;
}

PSMoveFloatVector3 PSMoveFloatVector3::safe_divide(const PSMoveFloatVector3 &v, const PSMoveFloatVector3 &default) const
{
    return 
        PSMoveFloatVector3::create(
            !is_nearly_zero(v.i) ? i/v.i : default.i, 
            !is_nearly_zero(v.j) ? j/v.j : default.j,
            !is_nearly_zero(v.k) ? k/v.k : default.k);
}

float PSMoveFloatVector3::length() const
{
    return sqrtf(i*i + j*j + k*k);
}

float PSMoveFloatVector3::normalize_with_default(const PSMoveFloatVector3 &default)
{
    const float divisor= length();
    
    *this= this->safe_divide(divisor, default); 

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

PSMoveIntVector3 PSMoveIntVector3::safe_divide(const int s, const PSMoveIntVector3 &default) const
{
    return s != 0 ? unsafe_divide(s) : default;
}

PSMoveIntVector3 PSMoveIntVector3::safe_divide(const PSMoveIntVector3 &v, const PSMoveIntVector3 &default) const
{
    return 
        PSMoveIntVector3::create(
            v.i != 0 ? i/v.i : default.i, 
            v.j != 0 ? j/v.j : default.j,
            v.k != 0 ? k/v.k : default.k);
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

PSMoveFloatVector3 PSMovePosition::operator - (const PSMovePosition &other) const
{
    return PSMoveFloatVector3::create(x - other.x, y - other.y, z - other.z);
}

 // -- PSMovePose -- 
void PSMovePose::Clear()
{
    Orientation= *k_psmove_quaternion_identity;
    Position= *k_psmove_position_origin;
}