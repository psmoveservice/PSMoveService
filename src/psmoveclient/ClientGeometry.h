#ifndef CLIENT_GEOMETRY_H
#define CLIENT_GEOMETRY_H

//-- includes -----
#include "ClientConfig.h"

//-- declarations -----
struct CLIENTPSMOVEAPI PSMoveFloatVector3
{
    float i, j, k;

    // psuedo-constructor to keep this a POD type
    static PSMoveFloatVector3 create(float i, float j, float k);

    PSMoveFloatVector3 operator + (const PSMoveFloatVector3 &other) const;
    PSMoveFloatVector3 operator - (const PSMoveFloatVector3 &other) const;
    PSMoveFloatVector3 operator * (const float s) const;
    
    PSMoveFloatVector3 unsafe_divide(const float s) const;
    PSMoveFloatVector3 safe_divide(const float s, const PSMoveFloatVector3 &default) const;

    float minValue() const;
    float maxValue() const;

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
CLIENTPSMOVEAPI extern const PSMovePosition *k_psmove_position_origin;
CLIENTPSMOVEAPI extern const PSMoveQuaternion *k_psmove_quaternion_identity;

#endif // CLIENT_GEOMETRY_H