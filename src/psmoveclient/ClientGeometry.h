#ifndef CLIENT_GEOMETRY_H
#define CLIENT_GEOMETRY_H

//-- includes -----
#include "ClientConfig.h"

//-- declarations -----
struct CLIENTPSMOVEAPI PSMoveFloatVector2
{
    float i, j;

    // psuedo-constructor to keep this a POD type
    static PSMoveFloatVector2 create(float i, float j);

    PSMoveFloatVector2 operator + (const PSMoveFloatVector2 &other) const;
    PSMoveFloatVector2 operator - (const PSMoveFloatVector2 &other) const;
    PSMoveFloatVector2 operator * (const float s) const;

    PSMoveFloatVector2 unsafe_divide(const float s) const;
    PSMoveFloatVector2 unsafe_divide(const PSMoveFloatVector2 &v) const;
    PSMoveFloatVector2 safe_divide(const float s, const PSMoveFloatVector2 &default_result) const;
    PSMoveFloatVector2 safe_divide(const PSMoveFloatVector2 &v, const PSMoveFloatVector2 &default_result) const;

    float length() const;
    float normalize_with_default(const PSMoveFloatVector2 &default_result);

    float minValue() const;
    float maxValue() const;

    static float dot(const PSMoveFloatVector2 &a, const PSMoveFloatVector2 &b);
    static PSMoveFloatVector2 min(const PSMoveFloatVector2 &a, const PSMoveFloatVector2 &b);
    static PSMoveFloatVector2 max(const PSMoveFloatVector2 &a, const PSMoveFloatVector2 &b);
};

struct CLIENTPSMOVEAPI PSMoveFloatVector3
{
    float i, j, k;

    // psuedo-constructor to keep this a POD type
    static PSMoveFloatVector3 create(float i, float j, float k);

    PSMoveFloatVector3 operator + (const PSMoveFloatVector3 &other) const;
    PSMoveFloatVector3 operator - (const PSMoveFloatVector3 &other) const;
    PSMoveFloatVector3 operator * (const float s) const;
    
    PSMoveFloatVector3 unsafe_divide(const float s) const;
    PSMoveFloatVector3 unsafe_divide(const PSMoveFloatVector3 &v) const;
    PSMoveFloatVector3 safe_divide(const float s, const PSMoveFloatVector3 &default_result) const;
    PSMoveFloatVector3 safe_divide(const PSMoveFloatVector3 &v, const PSMoveFloatVector3 &default_result) const;
    
    float length() const;
    float normalize_with_default(const PSMoveFloatVector3 &default_result);

    float minValue() const;
    float maxValue() const;

    static float dot(const PSMoveFloatVector3 &a, const PSMoveFloatVector3 &b);
    static PSMoveFloatVector3 min(const PSMoveFloatVector3 &a, const PSMoveFloatVector3 &b);
    static PSMoveFloatVector3 max(const PSMoveFloatVector3 &a, const PSMoveFloatVector3 &b);
};

struct CLIENTPSMOVEAPI PSMoveIntVector3
{
    int i, j, k;

    // psuedo-constructor to keep this a POD type
    static PSMoveIntVector3 create(int i, int j, int k);

    PSMoveFloatVector3 castToFloatVector3() const;

    PSMoveIntVector3 operator + (const PSMoveIntVector3 &other) const;
    PSMoveIntVector3 operator - (const PSMoveIntVector3 &other) const;

    PSMoveIntVector3 unsafe_divide(const int s) const;
    PSMoveIntVector3 unsafe_divide(const PSMoveIntVector3 &v) const;
    PSMoveIntVector3 safe_divide(const int s, const PSMoveIntVector3 &default_result) const;
    PSMoveIntVector3 safe_divide(const PSMoveIntVector3 &v, const PSMoveIntVector3 &default_result) const;

    int lengthSquared() const;

    int minValue() const;
    int maxValue() const;

    static PSMoveIntVector3 min(const PSMoveIntVector3 &a, const PSMoveIntVector3 &b);
    static PSMoveIntVector3 max(const PSMoveIntVector3 &a, const PSMoveIntVector3 &b);
};

struct CLIENTPSMOVEAPI PSMovePosition
{
    float x, y, z;

    // psuedo-constructor to keep this a POD type
    static PSMovePosition create(float x, float y, float z);

    PSMoveFloatVector3 operator - (const PSMovePosition &other) const;
    PSMovePosition operator * (const float s) const;
};

struct CLIENTPSMOVEAPI PSMoveScreenLocation
{
    float x, y;

    // psuedo-constructor to keep this a POD type
    static PSMoveScreenLocation create(float x, float y);

    PSMoveFloatVector2 toPSMoveFloatVector2() const;
    PSMoveFloatVector2 operator - (const PSMoveScreenLocation &other) const;
};

struct CLIENTPSMOVEAPI PSMoveQuaternion
{
    float w, x, y, z;
};

struct CLIENTPSMOVEAPI PSMovePose
{
    PSMoveQuaternion Orientation;
    PSMovePosition Position;

    void Clear();
};

//-- constants -----
CLIENTPSMOVEAPI extern const PSMoveIntVector3 *k_psmove_int_vector3_zero;
CLIENTPSMOVEAPI extern const PSMoveFloatVector3 *k_psmove_float_vector3_zero;
CLIENTPSMOVEAPI extern const PSMoveIntVector3 *k_psmove_int_vector3_one;
CLIENTPSMOVEAPI extern const PSMoveFloatVector3 *k_psmove_float_vector3_one;
CLIENTPSMOVEAPI extern const PSMovePosition *k_psmove_position_origin;
CLIENTPSMOVEAPI extern const PSMoveQuaternion *k_psmove_quaternion_identity;

#endif // CLIENT_GEOMETRY_H