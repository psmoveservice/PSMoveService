/**
\file
*/ 

#ifndef __CLIENTGEOMETRY_CAPI_H
#define __CLIENTGEOMETRY_CAPI_H
#include "PSMoveClient_export.h"
#include <stdbool.h>
//cut_before

/** 
\brief Geometrical data structures and functions used by the Client API  
\defgroup Geometry_CAPI Client Geometry
\addtogroup Geometry_CAPI 
@{ 
*/

//-- constants -----
/// Conversion factor to go from meters to centimeters
#define PSM_METERS_TO_CENTIMETERS  100.f

/// Conversion factor to go from centimeters to meters
#define PSM_CENTIMETERS_TO_METERS  0.01f

/// A 2D vector with float components.
typedef struct
{
    float x, y;
} PSMVector2f;

/// A 3D vector with float components.
typedef struct
{
    float x, y, z;
} PSMVector3f;

/// A 3D vector with int components.
typedef struct
{
    int x, y, z;
} PSMVector3i;

/** A 3x3 matrix with float components
	storage is row major order: [x0,x1,x2,y0,y1,y1,z0,z1,z2]
 */
typedef struct
{
    float m[3][3]; 
} PSMMatrix3f;

/// A quaternion rotation.
typedef struct
{
    float w, x, y, z;
} PSMQuatf;

/// Position and orientation together.
typedef struct
{
    PSMVector3f  Position;
    PSMQuatf     Orientation;
} PSMPosef;

/// A camera frustum
typedef struct
{
    PSMVector3f origin; 	///< frustum tip world location, in cm
    PSMVector3f forward; 	///< forward axis of the frustum
	PSMVector3f left; 		///< left axis of the frustum
	PSMVector3f up; 		///< up axis of the frustum
    float HFOV; 			///< horizontal field of view, in radians
	float VFOV; 			///< vertical field of fiew, in radians
    float zNear; 			///< near plane distance of frustum, in cm
	float zFar; 			///< far place distance of frustum, in cm
} PSMFrustum;

/// The projection of a tracking shape onto the image plane of a tracker video feed
typedef struct
{
    enum eShapeType
    {
        PSMShape_INVALID_PROJECTION = -1,
        PSMShape_Ellipse,					///< The 2D projection of a sphere (think conic sectioc)
        PSMShape_LightBar,					///< The 2D projection of a 3D quad (bounding shape of DS4 lightbar) 
		PSMShape_PointCloud					///< The 2D projection of a 3D point cloud (morpheus tracking lights)
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

/// Adds two 2D vectors together
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fAdd(const PSMVector2f *a, const PSMVector2f *b);
/// Subtracts one 2D vector from another 2D vector
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fSubtract(const PSMVector2f *a, const PSMVector2f *b);
/// Scales a 2D vector by a scalar
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fScale(const PSMVector2f *v, const float s);
/// Scales a 2D vector by a scalar and then adds a vector to the scaled result
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fScaleAndAdd(const PSMVector2f *v, const float s, const PSMVector2f *b);
/// Divides each component of a 2D vector by a scalar without checking for divide-by-zero
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fUnsafeScalarDivide(const PSMVector2f *numerator, const float divisor);
/// Divides each component of a 2D vector by the corresponding componenet of another vector without checking for a divide-by-zero
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fUnsafeVectorDivide(const PSMVector2f *numerator, const PSMVector2f *divisor);
/// Divides each component of a 2D vector by a scalar, returning a default vector in the case of divide by zero
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fSafeScalarDivide(const PSMVector2f *numerator, const float divisor, const PSMVector2f *default_result);
/// Divides each component of a 2D vector by another vector, returning a default value for each component in the case of divide by zero
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fSafeVectorDivide(const PSMVector2f *numerator, const PSMVector2f *divisor, const PSMVector2f *default_result);
/// Computes the absolute value of each component
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fAbs(const PSMVector2f *v);
/// Squares each component of a vector
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fSquare(const PSMVector2f *v);
/// Computes the length of a given vector
PSM_PUBLIC_FUNCTION(float) PSM_Vector2fLength(const PSMVector2f *v);
/// Computes the normalized version of a vector, returning a default vector in the event of a near zero length vector
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fNormalizeWithDefault(const PSMVector2f *v, const PSMVector2f *default_result);
/// Computes the minimum component of a given vector
PSM_PUBLIC_FUNCTION(float) PSM_Vector2fMinValue(const PSMVector2f *v);
/// Computes the maximum component of a given vector
PSM_PUBLIC_FUNCTION(float) PSM_Vector2fMaxValue(const PSMVector2f *v);
/// Computes the 2D dot product of two vectors
PSM_PUBLIC_FUNCTION(float) PSM_Vector2fDot(const PSMVector2f *a, const PSMVector2f *b);
/// Computes the min value of two vectors along each component
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fMin(const PSMVector2f *a, const PSMVector2f *b);
/// Computes the max value of two vectors along each component
PSM_PUBLIC_FUNCTION(PSMVector2f) PSM_Vector2fMax(const PSMVector2f *a, const PSMVector2f *b);

// PSMVector3f Methods

/// Adds two 3D vectors together
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fAdd(const PSMVector3f *a, const PSMVector3f *b);
/// Subtracts one 3D vector from another 3D vector
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fSubtract(const PSMVector3f *a, const PSMVector3f *b);
/// Scales a 3D vector by a scalar
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fScale(const PSMVector3f *v, const float s);
/// Scales a 3D vector by a scalar and then adds a vector to the scaled result
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fScaleAndAdd(const PSMVector3f *v, const float s, const PSMVector3f *b);
/// Divides each component of a 3D vector by a scalar without checking for divide-by-zero
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fUnsafeScalarDivide(const PSMVector3f *numerator, const float divisor);
/// Divides each component of a 3D vector by the corresponding componenet of another vector without checking for a divide-by-zero
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fUnsafeVectorDivide(const PSMVector3f *numerator, const PSMVector3f *divisor);
/// Divides each component of a 3D vector by a scalar, returning a default vector in the case of divide by zero
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fSafeScalarDivide(const PSMVector3f *numerator, const float divisor, const PSMVector3f *default_result);
/// Divides each component of a 2D vector by another vector, returning a default value for each component in the case of divide by zero
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fSafeVectorDivide(const PSMVector3f *numerator, const PSMVector3f *divisor, const PSMVector3f *default_result);
/// Computes the absolute value of each component
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fAbs(const PSMVector3f *v);
/// Squares each component of a vector
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fSquare(const PSMVector3f *v);
/// Computes the length of a given vector
PSM_PUBLIC_FUNCTION(float) PSM_Vector3fLength(const PSMVector3f *v);
/// Computes the normalized version of a vector, returning a default vector in the event of a near zero length vector
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fNormalizeWithDefault(const PSMVector3f *v, const PSMVector3f *default_result);
/// Computes the normalized version of a vector and its original length, returning a default vector in the event of a near zero length vector
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fNormalizeWithDefaultGetLength(const PSMVector3f *v, const PSMVector3f *default_result, float *out_length);
/// Computes the minimum component of a given vector
PSM_PUBLIC_FUNCTION(float) PSM_Vector3fMinValue(const PSMVector3f *v);
/// Computes the maximum component of a given vector
PSM_PUBLIC_FUNCTION(float) PSM_Vector3fMaxValue(const PSMVector3f *v);
/// Computes the 3D dot product of two vectors
PSM_PUBLIC_FUNCTION(float) PSM_Vector3fDot(const PSMVector3f *a, const PSMVector3f *b);
/// Compute the 3D cross product of two vectors
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fCross(const PSMVector3f *a, const PSMVector3f *b);
/// Computes the min value of two vectors along each component
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fMin(const PSMVector3f *a, const PSMVector3f *b);
/// Computes the max value of two vectors along each component
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3fMax(const PSMVector3f *a, const PSMVector3f *b);

// PSMVector3i Methods

/// Adds two 3D vectors together
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iAdd(const PSMVector3i *a, const PSMVector3i *b);
/// Subtracts one 3D vector from another 3D vector
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iSubtract(const PSMVector3i *a, const PSMVector3i *b);
/// Divides each component of a 3D vector by a scalar without checking for divide-by-zero
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iUnsafeScalarDivide(const PSMVector3i *numerator, const int divisor);
/// Divides each component of a 3D vector by the corresponding componenet of another vector without checking for a divide-by-zero
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iUnsafeVectorDivide(const PSMVector3i *numerator, const PSMVector3i *divisor);
/// Divides each component of a 3D vector by a scalar, returning a default vector in the case of divide by zero
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iSafeScalarDivide(const PSMVector3i *numerator, const int divisor, const PSMVector3i *default_result);
/// Divides each component of a 2D vector by another vector, returning a default value for each component in the case of divide by zero
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iSafeVectorDivide(const PSMVector3i *numerator, const PSMVector3i *divisor, const PSMVector3i *default_result);
/// Computes the absolute value of each component
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iAbs(const PSMVector3i *v);
/// Squares each component of a vector
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iSquare(const PSMVector3i *v);
/// Computes the squared-length of a given vector
PSM_PUBLIC_FUNCTION(int) PSM_Vector3iLengthSquared(const PSMVector3i *v);
/// Computes the minimum component of a given vector
PSM_PUBLIC_FUNCTION(int) PSM_Vector3iMinValue(const PSMVector3i *v);
/// Computes the maximum component of a given vector
PSM_PUBLIC_FUNCTION(int) PSM_Vector3iMaxValue(const PSMVector3i *v);
/// Computes the min value of two vectors along each component
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iMin(const PSMVector3i *a, const PSMVector3i *b);
/// Computes the max value of two vectors along each component
PSM_PUBLIC_FUNCTION(PSMVector3i) PSM_Vector3iMax(const PSMVector3i *a, const PSMVector3i *b);
/// Convertes a 3D int vector to a 3D float vector
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Vector3iCastToFloat(const PSMVector3i *v);

// PSMQuatf Methods

/// Construct a quaternion from raw w, x, y, and z components
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfCreate(float w, float x, float y, float z);
/// Construct a quaternion rotation from rotations about the X, Y, and Z axis
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfCreateFromAngles(const PSMVector3f *eulerAngles);
/// Component-wise add two quaternions together (used by numerical integration)
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfAdd(const PSMQuatf *a, const PSMQuatf *b);
/// Scale all components of a quaternion by a scalar (used by numerical integration)
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfScale(const PSMQuatf *q, const float s);
/// Compute the multiplication of two quaterions
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfMultiply(const PSMQuatf *a, const PSMQuatf *b);
/// Divide all components of a quaternion by a scalar without checking for divide by zero
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfUnsafeScalarDivide(const PSMQuatf *q, const float s);
/// Divide all components of a quaternion by a scalar, returning a default quaternion in case of a degenerate quaternion
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfSafeScalarDivide(const PSMQuatf *q, const float s, const PSMQuatf *default_result);
/// Compute the complex conjegate of a quaternion (negate imaginary components)
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfConjugate(const PSMQuatf *q);
/// Concatenate a second quaternion's rotation on to the end of a first quaternion's quaterion (just a quaternion multiplication)
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfConcat(const PSMQuatf *first, const PSMQuatf *second);
/// Rotate a vector by a given quaternion
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_QuatfRotateVector(const PSMQuatf *q, const PSMVector3f *v);
/// Compute the length of a quaternion (sum of squared components)
PSM_PUBLIC_FUNCTION(float) PSM_QuatfLength(const PSMQuatf *q);
/// Computes the normalized version of a quaternion, returning a default quaternion in the event of a near zero length quaternion
PSM_PUBLIC_FUNCTION(PSMQuatf) PSM_QuatfNormalizeWithDefault(const PSMQuatf *q, const PSMQuatf *default_result);

// PSMMatrix3f Methods
/// Create a 3x3 matrix from a set of 3 basis vectors (might not be ortho-normal)
PSM_PUBLIC_FUNCTION(PSMMatrix3f) PSM_Matrix3fCreate(const PSMVector3f *basis_x, const PSMVector3f *basis_y, const PSMVector3f *basis_z);
/// Create a 3x3 rotation matrix from a quaternion
PSM_PUBLIC_FUNCTION(PSMMatrix3f) PSM_Matrix3fCreateFromQuatf(const PSMQuatf *q);
/// Extract the x-axis basis vector from a 3x3 matrix
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Matrix3fBasisX(const PSMMatrix3f *m);
/// Extract the y-axis basis vector from a 3x3 matrix
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Matrix3fBasisY(const PSMMatrix3f *m);
/// Extract the z-axis basis vector from a 3x3 matrix
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_Matrix3fBasisZ(const PSMMatrix3f *m);

// PSMPosef
/// Create a pose from a given position and orientation
PSM_PUBLIC_FUNCTION(PSMPosef) PSM_PosefCreate(const PSMVector3f *position, const PSMQuatf *orientation);
/// Create a pose that inverts the transform (rotation and translation) of a given pose
PSM_PUBLIC_FUNCTION(PSMPosef) PSM_PosefInverse(const PSMPosef *pose);
/// Concatenate the transformation of one pose onto the transformation of another pose
PSM_PUBLIC_FUNCTION(PSMPosef) PSM_PosefConcat(const PSMPosef *first, const PSMPosef *second);
/// Transform point by a pose
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_PosefTransformPoint(const PSMPosef *pose, const PSMVector3f *p);
/// Transform a point by the inverse of a pose
PSM_PUBLIC_FUNCTION(PSMVector3f) PSM_PosefInverseTransformPoint(const PSMPosef *pose, const PSMVector3f *p);

// PSMFrustumf
/// Update the basis (position and orientation) of a fustum to match that of a given pose
PSM_PUBLIC_FUNCTION(void) PSM_FrustumSetPose(PSMFrustum *frustum, const PSMPosef *pose);

// PSMTrackingProjection
/// Compute the area in pixels^2 of a tracking projection
PSM_PUBLIC_FUNCTION(float) PSM_TrackingProjectionGetArea(const PSMTrackingProjection *proj);

//-- constants -----
/// A 3D integer vector whose components are all 0
PSM_PUBLIC_CLASS extern const PSMVector3i *k_psm_int_vector3_zero;
/// A 3D float vector whose components are all 0.0f
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_zero;
/// A 3D integer vector whose components are all 1
PSM_PUBLIC_CLASS extern const PSMVector3i *k_psm_int_vector3_one;
/// A 3D float vector whose components are all 1.0f
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_one;
/// The 3D float vector <1.0f, 0.0f, 0.0f>
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_i;
/// The 3D float vector <0.0f, 1.0f, 0.0f>
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_j;
/// The 3D float vector <0.0f, 0.0f, 1.0f>
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_float_vector3_k;
/// A 3D float vector that represents the world origin <0.f, 0.f, 0.f>
PSM_PUBLIC_CLASS extern const PSMVector3f *k_psm_position_origin;
/// The quaterion <1.f, 0.f, 0.f, 0.f> that represents no rotation
PSM_PUBLIC_CLASS extern const PSMQuatf *k_psm_quaternion_identity;
/// The 3x3 matrix that represent no transform (diagonal values 1.f, off diagonal values 0.f)
PSM_PUBLIC_CLASS extern const PSMMatrix3f *k_psm_matrix_identity;
/// The pose that represents no transformation (identity quaternion, zero vector)
PSM_PUBLIC_CLASS extern const PSMPosef *k_psm_pose_identity;

/** 
@} 
*/ 

//cut_after
#endif
