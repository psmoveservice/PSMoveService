#ifndef __CLIENTGEOMETRY_CAPI_H
#define __CLIENTGEOMETRY_CAPI_H
#include "PSMoveClient_export.h"
#include <stdbool.h>
//cut_before

//-- constants -----
#define PSM_METERS_TO_CENTIMETERS  100.f
#define PSM_CENTIMETERS_TO_METERS  0.01f

/// A 2D vector with float components.
typedef struct _PSMVector2f
{
    float x, y;
} PSMVector2f;

/// A 3D vector with float components.
typedef struct _PSMVector3f
{
    float x, y, z;
} PSMVector3f;

/// A 3D vector with int components.
typedef struct _PSMVector3i
{
    int x, y, z;
} PSMVector3i;

typedef struct _PSMMatrix3f
{
    float m[3][3]; // storage is row major order: [x0,x1,x2,y0,y1,y1,z0,z1,z2]
} PSMMatrix3f;

/// A quaternion rotation.
typedef struct _PSMQuatf
{
    float w, x, y, z;
} PSMQuatf;

/// Position and orientation together.
typedef struct _PSMPosef
{
    PSMVector3f  Position;
    PSMQuatf     Orientation;
} PSMPosef;

/// A camera frustum
typedef struct _PSMFrustum
{
    PSMVector3f origin; // cm
    PSMVector3f forward, left, up;
    float HFOV, VFOV; // radians
    float zNear, zFar; // cm
} PSMFrustum;

/// A 4 sided volume
typedef struct _PSMVolume
{
    PSMVector3f vertices[4];
    int vertex_count;
    float up_height;
} PSMVolume;

typedef struct _PSMTrackingProjection
{
    enum eShapeType
    {
        PSMShape_INVALID_PROJECTION = -1,
        PSMShape_Ellipse,
        PSMShape_LightBar,
		PSMShape_PointCloud
    }                               shape_type;
    union{
        struct {
            PSMVector2f center;
            float half_x_extent;
            float half_y_extent;
            float angle;
        } ellipse;
        struct {
            PSMVector2f triangle[3];
			PSMVector2f quad[4];
        } lightbar;
		struct {
			PSMVector2f points[7];
			int point_count;
		} pointcloud;
    }                               shape;
    
} PSMTrackingProjection;

// Interface
//----------

// PSMVector2f Methods
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fAdd(const PSMVector2f *a, const PSMVector2f *b);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fSubtract(const PSMVector2f *a, const PSMVector2f *b);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fScale(const PSMVector2f *v, const float s);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fScaleAndAdd(const PSMVector2f *v, const float s, const PSMVector2f *b);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fUnsafeScalarDivide(const PSMVector2f *numerator, const float divisor);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fUnsafeVectorDivide(const PSMVector2f *numerator, const PSMVector2f *divisor);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fSafeScalarDivide(const PSMVector2f *numerator, const float divisor, const PSMVector2f *default_result);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fSafeVectorDivide(const PSMVector2f *numerator, const PSMVector2f *divisor, const PSMVector2f *default_result);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fAbs(const PSMVector2f *v);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fSquare(const PSMVector2f *v);
PSM_PUBLIC_FUNCTION(float) PSM_Vector2fLength(const PSMVector2f *v);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fNormalizeWithDefault(const PSMVector2f *v, const PSMVector2f *default_result);
PSM_PUBLIC_FUNCTION(float) PSM_Vector2fMinValue(const PSMVector2f *v);
PSM_PUBLIC_FUNCTION(float) PSM_Vector2fMaxValue(const PSMVector2f *v);
PSM_PUBLIC_FUNCTION(float) PSM_Vector2fDot(const PSMVector2f *a, const PSMVector2f *b);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fMin(const PSMVector2f *a, const PSMVector2f *b);
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fMax(const PSMVector2f *a, const PSMVector2f *b);

// PSMVector3f Methods
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fAdd(const PSMVector3f *a, const PSMVector3f *b);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fSubtract(const PSMVector3f *a, const PSMVector3f *b);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fScale(const PSMVector3f *v, const float s);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fScaleAndAdd(const PSMVector3f *v, const float s, const PSMVector3f *b);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fUnsafeScalarDivide(const PSMVector3f *numerator, const float divisor);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fUnsafeVectorDivide(const PSMVector3f *numerator, const PSMVector3f *divisor);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fSafeScalarDivide(const PSMVector3f *numerator, const float divisor, const PSMVector3f *default_result);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fSafeVectorDivide(const PSMVector3f *numerator, const PSMVector3f *divisor, const PSMVector3f *default_result);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fAbs(const PSMVector3f *v);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fSquare(const PSMVector3f *v);
PSM_PUBLIC_FUNCTION(float) PSM_Vector3fLength(const PSMVector3f *v);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fNormalizeWithDefault(const PSMVector3f *v, const PSMVector3f *default_result);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fNormalizeWithDefaultGetLength(const PSMVector3f *v, const PSMVector3f *default_result, float *out_length);
PSM_PUBLIC_FUNCTION(float) PSM_Vector3fMinValue(const PSMVector3f *v);
PSM_PUBLIC_FUNCTION(float) PSM_Vector3fMaxValue(const PSMVector3f *v);
PSM_PUBLIC_FUNCTION(float) PSM_Vector3fDot(const PSMVector3f *a, const PSMVector3f *b);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fCross(const PSMVector3f *a, const PSMVector3f *b);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fMin(const PSMVector3f *a, const PSMVector3f *b);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fMax(const PSMVector3f *a, const PSMVector3f *b);

// PSMVector3i Methods
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iAdd(const PSMVector3i *a, const PSMVector3i *b);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iSubtract(const PSMVector3i *a, const PSMVector3i *b);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iUnsafeScalarDivide(const PSMVector3i *numerator, const int divisor);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iUnsafeVectorDivide(const PSMVector3i *numerator, const PSMVector3i *divisor);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iSafeScalarDivide(const PSMVector3i *numerator, const int divisor, const PSMVector3i *default_result);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iSafeVectorDivide(const PSMVector3i *numerator, const PSMVector3i *divisor, const PSMVector3i *default_result);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iAbs(const PSMVector3i *v);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iSquare(const PSMVector3i *v);
PSM_PUBLIC_FUNCTION(int) PSM_Vector3iLengthSquared(const PSMVector3i *v);
PSM_PUBLIC_FUNCTION(int) PSM_Vector3iMinValue(const PSMVector3i *v);
PSM_PUBLIC_FUNCTION(int) PSM_Vector3iMaxValue(const PSMVector3i *v);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iMin(const PSMVector3i *a, const PSMVector3i *b);
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iMax(const PSMVector3i *a, const PSMVector3i *b);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3iCastToFloat(const PSMVector3i *v);

// PSMQuatf Methods
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfCreate(float w, float x, float y, float z);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfCreateFromAngles(const PSMVector3f *eulerAngles);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfAdd(const PSMQuatf *a, const PSMQuatf *b);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfScale(const PSMQuatf *q, const float s);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfMultiply(const PSMQuatf *a, const PSMQuatf *b);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfUnsafeScalarDivide(const PSMQuatf *q, const float s);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfSafeScalarDivide(const PSMQuatf *q, const float s, const PSMQuatf *default_result);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfConjugate(const PSMQuatf *q);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfConcat(const PSMQuatf *first, const PSMQuatf *second);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_QuatfRotateVector(const PSMQuatf *q, const PSMVector3f *v);
PSM_PUBLIC_FUNCTION(float) PSM_QuatfLength(const PSMQuatf *q);
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfNormalizeWithDefault(const PSMQuatf *q, const PSMQuatf *default_result);

// PSMMatrix3f Methods
PSM_PUBLIC_FUNCTION(PSMMatrix3f) PSM_Matrix3fCreate(const PSMVector3f *basis_x, const PSMVector3f *basis_y, const PSMVector3f *basis_z);
PSM_PUBLIC_FUNCTION(PSMMatrix3f) PSM_Matrix3fCreateFromQuatf(const PSMQuatf *q);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Matrix3fBasisX(const PSMMatrix3f *m);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Matrix3fBasisY(const PSMMatrix3f *m);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Matrix3fBasisZ(const PSMMatrix3f *m);

// PSMPosef
PSM_PUBLIC_FUNCTION(PSMPosef) PSM_PosefCreate(const PSMVector3f *position, const PSMQuatf *orientation);
PSM_PUBLIC_FUNCTION(PSMPosef) PSM_PosefInverse(const PSMPosef *pose);
PSM_PUBLIC_FUNCTION(PSMPosef) PSM_PosefConcat(const PSMPosef *first, const PSMPosef *second);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_PosefTransformPoint(const PSMPosef *pose, const PSMVector3f *p);
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_PosefInverseTransformPoint(const PSMPosef *pose, const PSMVector3f *p);

// PSMFrustumf
PSM_PUBLIC_FUNCTION(void) PSM_FrustumSetPose(PSMFrustum *frustum, const PSMPosef *pose);

// PSMTrackingProjection
PSM_PUBLIC_FUNCTION(float) PSM_TrackingProjectionGetArea(const PSMTrackingProjection *proj);

//-- constants -----
PSM_PUBLIC_CLASS extern const PSMVector3i *k_psm_int_vector3_zero;
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_zero;
PSM_PUBLIC_CLASS extern const PSMVector3i *k_psm_int_vector3_one;
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_one;
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_i;
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_j;
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_k;
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_position_origin;
PSM_PUBLIC_CLASS extern const PSMQuatf *k_psm_quaternion_identity;
PSM_PUBLIC_CLASS extern const PSMMatrix3f *k_psm_matrix_identity;
PSM_PUBLIC_CLASS extern const PSMPosef *k_psm_pose_identity;

//cut_after
#endif
