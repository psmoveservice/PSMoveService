#ifndef CLIENT_GEOMETRY_H
#define CLIENT_GEOMETRY_H

//-- includes -----
#include "ClientConfig.h"

//-- pre-declarations -----
struct PSMovePosition;

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

    PSMoveFloatVector2 abs() const;
    PSMoveFloatVector2 square() const;

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

    PSMovePosition castToPSMovePosition() const;

    PSMoveFloatVector3 operator + (const PSMoveFloatVector3 &other) const;
    PSMoveFloatVector3 operator - (const PSMoveFloatVector3 &other) const;
    PSMoveFloatVector3 operator * (const float s) const;
    
    PSMoveFloatVector3 unsafe_divide(const float s) const;
    PSMoveFloatVector3 unsafe_divide(const PSMoveFloatVector3 &v) const;
    PSMoveFloatVector3 safe_divide(const float s, const PSMoveFloatVector3 &default_result) const;
    PSMoveFloatVector3 safe_divide(const PSMoveFloatVector3 &v, const PSMoveFloatVector3 &default_result) const;
    
    PSMoveFloatVector3 abs() const;
    PSMoveFloatVector3 square() const;

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

    PSMoveIntVector3 abs() const;
    PSMoveIntVector3 square() const;

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

    PSMoveFloatVector3 toPSMoveFloatVector3() const;
    PSMoveFloatVector3 operator - (const PSMovePosition &other) const;
	PSMovePosition operator + (const PSMoveFloatVector3 &v) const;
	PSMovePosition operator - (const PSMoveFloatVector3 &v) const;
    PSMovePosition operator * (const float s) const;
};

/// A screen location in the space [-frameWidth/2, -frameHeight/2]x[frameWidth/2, frameHeight/2]    
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

    // psuedo-constructor to keep this a POD type
    static PSMoveQuaternion create(float w, float x, float y, float z);

    PSMoveQuaternion operator + (const PSMoveQuaternion &other) const;
	PSMoveQuaternion operator * (const PSMoveQuaternion &other) const;

    PSMoveQuaternion unsafe_divide(const float s) const;
    PSMoveQuaternion safe_divide(const float s, const PSMoveQuaternion &default_result) const;
	PSMoveQuaternion inverse() const;
	static PSMoveQuaternion concat(const PSMoveQuaternion &first, const PSMoveQuaternion &second);
	PSMoveFloatVector3 rotate_vector(const PSMoveFloatVector3 &v) const;
	PSMovePosition rotate_position(const PSMovePosition &v) const;

    float length() const;
    PSMoveQuaternion &normalize_with_default(const PSMoveQuaternion &default_result);
};

struct CLIENTPSMOVEAPI PSMoveMatrix3x3
{
    float m[3][3]; // storage is row major order: [x0,x1,x2,y0,y1,y1,z0,z1,z2]

    static PSMoveMatrix3x3 create(
        const PSMoveFloatVector3 &basis_x, 
        const PSMoveFloatVector3 &basis_y,
        const PSMoveFloatVector3 &basis_z);
	static PSMoveMatrix3x3 create(const PSMoveQuaternion &q);

    PSMoveFloatVector3 basis_x() const;
    PSMoveFloatVector3 basis_y() const;
    PSMoveFloatVector3 basis_z() const;
};

struct CLIENTPSMOVEAPI PSMovePose
{
    PSMovePosition Position;
    PSMoveQuaternion Orientation;

    void Clear();
	PSMovePose inverse() const;
	static PSMovePose concat(const PSMovePose &first, const PSMovePose &second);
	PSMovePosition apply_transform(const PSMovePosition &p) const;
	PSMovePosition apply_inverse_transform(const PSMovePosition &p) const;
};

struct CLIENTPSMOVEAPI PSMoveFrustum
{
    PSMovePosition origin; // cm
    PSMoveFloatVector3 forward, left, up;
    float HFOV, VFOV; // radians
    float zNear, zFar; // cm

    void set_pose(const PSMovePose &pose);
};

struct CLIENTPSMOVEAPI PSMoveTrackingProjection
{
    enum eShapeType {
        INVALID_PROJECTION = -1,

        Ellipse,
        LightBar,

        MAX_TRACKING_PROJECTION_TYPES
    };

    union{
        struct {
            PSMoveScreenLocation center;
            float half_x_extent;
            float half_y_extent;
            float angle;
        } ellipse;

        struct {
            PSMoveScreenLocation triangle[3];
            PSMoveScreenLocation quad[4];
        } lightbar;
    } shape;

    eShapeType shape_type;
};

struct CLIENTPSMOVEAPI PSMoveVolume
{
    PSMovePosition vertices[4];
    int vertex_count;
    float up_height;
};

//-- constants -----
CLIENTPSMOVEAPI extern const PSMoveIntVector3 *k_psmove_int_vector3_zero;
CLIENTPSMOVEAPI extern const PSMoveFloatVector3 *k_psmove_float_vector3_zero;
CLIENTPSMOVEAPI extern const PSMoveIntVector3 *k_psmove_int_vector3_one;
CLIENTPSMOVEAPI extern const PSMoveFloatVector3 *k_psmove_float_vector3_one;
CLIENTPSMOVEAPI extern const PSMoveFloatVector3 *k_psmove_float_vector3_i;
CLIENTPSMOVEAPI extern const PSMoveFloatVector3 *k_psmove_float_vector3_j;
CLIENTPSMOVEAPI extern const PSMoveFloatVector3 *k_psmove_float_vector3_k;
CLIENTPSMOVEAPI extern const PSMovePosition *k_psmove_position_origin;
CLIENTPSMOVEAPI extern const PSMoveQuaternion *k_psmove_quaternion_identity;
CLIENTPSMOVEAPI extern const PSMoveMatrix3x3 *k_psmove_matrix_identity;
CLIENTPSMOVEAPI extern const PSMovePose *k_psmove_pose_identity;

#endif // CLIENT_GEOMETRY_H