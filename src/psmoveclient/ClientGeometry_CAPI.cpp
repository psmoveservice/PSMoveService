//-- includes -----
#include "ClientGeometry_CAPI.h"
#include "MathUtility.h"
#include "MathGLM.h"
#include <algorithm>

//-- constants -----
const PSMVector3f g_psm_float_vector3_zero= {0.f, 0.f, 0.f};
const PSMVector3f *k_psm_float_vector3_zero= &g_psm_float_vector3_zero;

const PSMVector3f g_psm_float_vector3_one= {1.f, 1.f, 1.f};
const PSMVector3f *k_psm_float_vector3_one= &g_psm_float_vector3_one;

const PSMVector3f g_psm_float_vector3_i = { 1.f, 0.f, 0.f };
const PSMVector3f *k_psm_float_vector3_i = &g_psm_float_vector3_i;

const PSMVector3f g_psm_float_vector3_j = { 0.f, 1.f, 0.f };
const PSMVector3f *k_psm_float_vector3_j = &g_psm_float_vector3_j;

const PSMVector3f g_psm_float_vector3_k = { 0.f, 0.f, 1.f };
const PSMVector3f *k_psm_float_vector3_k = &g_psm_float_vector3_k;

const PSMVector3i g_psm_int_vector3_zero= {0, 0, 0};
const PSMVector3i *k_psm_int_vector3_zero= &g_psm_int_vector3_zero;

const PSMVector3i g_psm_int_vector3_one= {1, 1, 1};
const PSMVector3i *k_psm_int_vector3_one= &g_psm_int_vector3_one;

const PSMVector3f g_psm_position_origin= {0.f, 0.f, 0.f};
const PSMVector3f *k_psm_position_origin= &g_psm_position_origin;

const PSMQuatf g_psm_quaternion_identity= {1.f, 0.f, 0.f, 0.f};
const PSMQuatf *k_psm_quaternion_identity= &g_psm_quaternion_identity;

const PSMMatrix3f g_psm_matrix_identity = { {{1.f, 0.f, 0.f} , {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}} };
const PSMMatrix3f *k_psm_matrix_identity = &g_psm_matrix_identity;

const PSMPosef g_psm_pose_identity = { g_psm_position_origin, g_psm_quaternion_identity };
const PSMPosef *k_psm_pose_identity = &g_psm_pose_identity;

//-- methods -----
// PSMVector2f Methods
PSMVector2f PSM_Vector2fAdd(const PSMVector2f *a, const PSMVector2f *b)
{
	return {a->x + b->x, a->y + b->y};
}

PSMVector2f PSM_Vector2fSubtract(const PSMVector2f *a, const PSMVector2f *b)
{
	return {a->x - b->x, a->y - b->y};
}

PSMVector2f PSM_Vector2fScale(const PSMVector2f *v, const float s)
{
	return {v->x*s, v->y*s};
}

PSMVector2f PSM_Vector2fScaleAndAdd(const PSMVector2f *v, const float s, const PSMVector2f *b)
{
	return {v->x*s + b->x, v->y*s + b->y};
}

PSMVector2f PSM_Vector2fUnsafeScalarDivide(const PSMVector2f *numerator, const float divisor)
{
	return {numerator->x/divisor, numerator->y/divisor};
}

PSMVector2f PSM_Vector2fUnsafeVectorDivide(const PSMVector2f *numerator, const PSMVector2f *divisor)
{
	return {numerator->x/divisor->x, numerator->y/divisor->y};
}

PSMVector2f PSM_Vector2fSafeScalarDivide(const PSMVector2f *numerator, const float divisor, const PSMVector2f *default_result)
{
	return !is_nearly_zero(divisor) ? PSM_Vector2fUnsafeScalarDivide(numerator, divisor) : *default_result;
}

PSMVector2f PSM_Vector2fSafeVectorDivide(const PSMVector2f *numerator, const PSMVector2f *divisor, const PSMVector2f *default_result)
{
	return {!is_nearly_zero(divisor->x) ? (numerator->x / divisor->x) : default_result->x,
			!is_nearly_zero(divisor->y) ? (numerator->y / divisor->y) : default_result->y};
}

PSMVector2f PSM_Vector2fAbs(const PSMVector2f *v)
{
	return {fabsf(v->x), fabsf(v->y)};
}

PSMVector2f PSM_Vector2fSquare(const PSMVector2f *v)
{
	return {v->x*v->x, v->y*v->y};
}

float PSM_Vector2fLength(const PSMVector2f *v)
{
	return sqrtf(v->x*v->x + v->y*v->y);
}

PSMVector2f PSM_Vector2fNormalizeWithDefault(const PSMVector2f *v, const PSMVector2f *default_result)
{
    return PSM_Vector2fSafeScalarDivide(v, PSM_Vector2fLength(v), default_result);
}

float PSM_Vector2fMinValue(const PSMVector2f *v)
{
	return fminf(v->x, v->y);
}

float PSM_Vector2fMaxValue(const PSMVector2f *v)
{
	return fmaxf(v->x, v->y);
}

float PSM_Vector2fDot(const PSMVector2f *a, const PSMVector2f *b)
{
	return a->x*b->x + a->x*b->y;
}

PSMVector2f PSM_Vector2fMin(const PSMVector2f *a, const PSMVector2f *b)
{
	return { fminf(a->x, b->x), fminf(a->y, b->y) };
}

PSMVector2f PSM_Vector2fMax(const PSMVector2f *a, const PSMVector2f *b)
{
	return { fmaxf(a->x, b->x), fmaxf(a->y, b->y) };
}

// PSMVector3f Methods
PSMVector3f PSM_Vector3fAdd(const PSMVector3f *a, const PSMVector3f *b)
{
	return {a->x + b->x, a->y + b->y, a->z + b->z};
}

PSMVector3f PSM_Vector3fSubtract(const PSMVector3f *a, const PSMVector3f *b)
{
	return {a->x - b->x, a->y - b->y, a->z - b->z};
}

PSMVector3f PSM_Vector3fScale(const PSMVector3f *v, const float s)
{
	return {v->x*s, v->y*s, v->z*s};
}

PSMVector3f PSM_Vector3fScaleAndAdd(const PSMVector3f *v, const float s, const PSMVector3f *b)
{
	return {v->x*s + b->x, v->y*s + b->y, v->z*s + b->z};
}

PSMVector3f PSM_Vector3fUnsafeScalarDivide(const PSMVector3f *numerator, const float divisor)
{
	return {numerator->x/divisor, numerator->y/divisor, numerator->z/divisor};
}

PSMVector3f PSM_Vector3fUnsafeVectorDivide(const PSMVector3f *numerator, const PSMVector3f *divisor)
{
	return {numerator->x/divisor->x, numerator->y/divisor->y, numerator->z/divisor->z};
}

PSMVector3f PSM_Vector3fSafeScalarDivide(const PSMVector3f *numerator, const float divisor, const PSMVector3f *default_result)
{
	return !is_nearly_zero(divisor) ? PSM_Vector3fUnsafeScalarDivide(numerator, divisor) : *default_result;
}

PSMVector3f PSM_Vector3fSafeVectorDivide(const PSMVector3f *numerator, const PSMVector3f *divisor, const PSMVector3f *default_result)
{
	return {!is_nearly_zero(divisor->x) ? (numerator->x / divisor->x) : default_result->x,
			!is_nearly_zero(divisor->y) ? (numerator->y / divisor->y) : default_result->y,
			!is_nearly_zero(divisor->z) ? (numerator->z / divisor->z) : default_result->z};
}

PSMVector3f PSM_Vector3fAbs(const PSMVector3f *v)
{
	return {fabsf(v->x), fabsf(v->y), fabsf(v->z)};
}

PSMVector3f PSM_Vector3fSquare(const PSMVector3f *v)
{
	return {v->x*v->x, v->y*v->y, v->z*v->z};
}

float PSM_Vector3fLength(const PSMVector3f *v)
{
	return sqrtf(v->x*v->x + v->y*v->y + v->z*v->z);
}

PSMVector3f PSM_Vector3fNormalizeWithDefault(const PSMVector3f *v, const PSMVector3f *default_result)
{
	return PSM_Vector3fSafeScalarDivide(v, PSM_Vector3fLength(v), default_result);
}

PSMVector3f PSM_Vector3fNormalizeWithDefaultGetLength(const PSMVector3f *v, const PSMVector3f *default_result, float *out_length)
{
	const float length= PSM_Vector3fLength(v);
		
	if (out_length)
		*out_length= length;

	return PSM_Vector3fSafeScalarDivide(v, length, default_result);
}

float PSM_Vector3fMinValue(const PSMVector3f *v)
{
	return fminf(fminf(v->x, v->y), v->z);
}

float PSM_Vector3fMaxValue(const PSMVector3f *v)
{
	return fmaxf(fmaxf(v->x, v->y), v->z);
}

float PSM_Vector3fDot(const PSMVector3f *a, const PSMVector3f *b)
{
	return a->x*b->x + a->y*b->y + a->z*b->z;
}

PSMVector3f PSM_Vector3fCross(const PSMVector3f *a, const PSMVector3f *b)
{
	return {a->y*b->z - b->y*a->z, a->x*b->z - b->x*a->z, a->x*b->y - b->x*a->y};
}

PSMVector3f PSM_Vector3fMin(const PSMVector3f *a, const PSMVector3f *b)
{
	return {fminf(a->x, b->x), fminf(a->y, b->y), fminf(a->z, b->z)};
}

PSMVector3f PSM_Vector3fMax(const PSMVector3f *a, const PSMVector3f *b)
{
	return {fmaxf(a->x, b->x), fmaxf(a->y, b->y), fmaxf(a->z, b->z)};
}

// PSMVector3i Methods
PSMVector3i PSM_Vector3iAdd(const PSMVector3i *a, const PSMVector3i *b)
{
	return {a->x + b->x, a->y + b->y, a->z + b->z};
}

PSMVector3i PSM_Vector3iSubtract(const PSMVector3i *a, const PSMVector3i *b)
{
	return {a->x - b->x, a->y - b->y, a->z - b->z};
}

PSMVector3i PSM_Vector3iUnsafeScalarDivide(const PSMVector3i *numerator, const int divisor)
{
	return {numerator->x/divisor, numerator->y/divisor, numerator->z/divisor};
}

PSMVector3i PSM_Vector3iUnsafeVectorDivide(const PSMVector3i *numerator, const PSMVector3i *divisor)
{
	return {numerator->x/divisor->x, numerator->y/divisor->y, numerator->z/divisor->z};
}

PSMVector3i PSM_Vector3iSafeScalarDivide(const PSMVector3i *numerator, const int divisor, const PSMVector3i *default_result)
{
	return divisor != 0 ? PSM_Vector3iUnsafeScalarDivide(numerator, divisor) : *default_result;
}

PSMVector3i PSM_Vector3iSafeVectorDivide(const PSMVector3i *numerator, const PSMVector3i *divisor, const PSMVector3i *default_result)
{
	return {divisor->x != 0 ? (numerator->x / divisor->x) : default_result->x,
			divisor->y != 0 ? (numerator->y / divisor->y) : default_result->y,
			divisor->z != 0 ? (numerator->z / divisor->z) : default_result->z};
}

PSMVector3i PSM_Vector3iAbs(const PSMVector3i *v)
{
	return {std::abs(v->x), std::abs(v->y), std::abs(v->z)};
}

PSMVector3i PSM_Vector3iSquare(const PSMVector3i *v)
{
	return {v->x*v->x, v->y*v->y, v->z*v->z};
}

int PSM_Vector3iLengthSquared(const PSMVector3i *v)
{
	return v->x*v->x + v->y*v->y + v->z*v->z;
}

int PSM_Vector3iMinValue(const PSMVector3i *v)
{
	return std::min(std::min(v->x, v->y), v->z);
}

int PSM_Vector3iMaxValue(const PSMVector3i *v)
{
	return std::max(std::max(v->x, v->y), v->z);
}

PSMVector3i PSM_Vector3iMin(const PSMVector3i *a, const PSMVector3i *b)
{
	return {std::min(a->x, b->x), std::min(a->y, b->y), std::min(a->z, b->z)};
}

PSMVector3i PSM_Vector3iMax(const PSMVector3i *a, const PSMVector3i *b)
{
	return {std::max(a->x, b->x), std::max(a->y, b->y), std::max(a->z, b->z)};
}

PSMVector3f PSM_Vector3iCastToFloat(const PSMVector3i *v)
{
	return { static_cast<float>(v->x), static_cast<float>(v->y), static_cast<float>(v->z) };
}

// PSMQuatf Methods
PSMQuatf PSM_QuatfCreate(float w, float x, float y, float z)
{
	return {w, x, y, z};
}

PSMQuatf PSM_QuatfCreateFromAngles(const PSMVector3f *eulerAngles)
{
	PSMQuatf q;

	// Assuming the angles are in radians.
	float c1 = cosf(eulerAngles->y / 2.f);
	float s1 = sinf(eulerAngles->y / 2.f);
	float c2 = cosf(eulerAngles->z / 2.f);
	float s2 = sinf(eulerAngles->z / 2.f);
	float c3 = cosf(eulerAngles->x / 2.f);
	float s3 = sinf(eulerAngles->x / 2.f);
	float c1c2 = c1*c2;
	float s1s2 = s1*s2;
	q.w = c1c2*c3 - s1s2*s3;
	q.x = c1c2*s3 + s1s2*c3;
	q.y = s1*c2*c3 + c1*s2*s3;
	q.z = c1*s2*c3 - s1*c2*s3;

	return q;
}

PSMQuatf PSM_QuatfAdd(const PSMQuatf *a, const PSMQuatf *b)
{
	return {a->w + b->w, a->x + b->x, a->y + b->y, a->z + b->z};
}

PSMQuatf PSM_QuatfScale(const PSMQuatf *q, const float s)
{
	return {q->w*s, q->x*s, q->y*s, q->z*s};
}

PSMQuatf PSM_QuatfMultiply(const PSMQuatf *a, const PSMQuatf *b)
{
	return {a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z,
			a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y,
			a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x,
			a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w};
}

PSMQuatf PSM_QuatfUnsafeScalarDivide(const PSMQuatf *q, const float s)
{
	return {q->w / s, q->x / s, q->y / s, q->z / s};
}

PSMQuatf PSM_QuatfSafeScalarDivide(const PSMQuatf *q, const float s, const PSMQuatf *default_result)
{
	return !is_nearly_zero(s) ? PSM_QuatfUnsafeScalarDivide(q, s) : *default_result;
}

PSMQuatf PSM_QuatfConjugate(const PSMQuatf *q)
{
	return {q->w, -q->x, -q->y, -q->z};
}

PSMQuatf PSM_QuatfConcat(const PSMQuatf *first, const PSMQuatf *second)
{
	return PSM_QuatfMultiply(second, first);
}

PSMVector3f PSM_QuatfRotateVector(const PSMQuatf *q, const PSMVector3f *v)
{
	return {q->w*q->w*v->x + 2*q->y*q->w*v->z - 2*q->z*q->w*v->y + q->x*q->x*v->x + 2*q->y*q->x*v->y + 2*q->z*q->x*v->z - q->z*q->z*v->x - q->y*q->y*v->x,
			2*q->x*q->y*v->x + q->y*q->y*v->y + 2*q->z*q->y*v->z + 2*q->w*q->z*v->x - q->z*q->z*v->y + q->w*q->w*v->y - 2*q->x*q->w*v->z - q->x*q->x*v->y,
			2*q->x*q->z*v->x + 2*q->y*q->z*v->y + q->z*q->z*v->z - 2*q->w*q->y*v->x - q->y*q->y*v->z + 2*q->w*q->x*v->y - q->x*q->x*v->z + q->w*q->w*v->z};
}

float PSM_QuatfLength(const PSMQuatf *q)
{
    return sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

PSMQuatf PSM_QuatfNormalizeWithDefault(const PSMQuatf *q, const PSMQuatf *default_result)
{
	return PSM_QuatfSafeScalarDivide(q, PSM_QuatfLength(q), default_result);
}

// PSMMatrix3f Methods
PSMMatrix3f PSM_Matrix3fCreate(const PSMVector3f *basis_x, const PSMVector3f *basis_y, const PSMVector3f *basis_z)
{
    PSMMatrix3f mat;

    mat.m[0][0] = basis_x->x; mat.m[0][1] = basis_x->y; mat.m[0][2] = basis_x->z;
    mat.m[1][0] = basis_y->x; mat.m[1][1] = basis_y->y; mat.m[1][2] = basis_y->z;
    mat.m[2][0] = basis_z->x; mat.m[2][1] = basis_z->y; mat.m[2][2] = basis_z->z;

    return mat;
}

PSMMatrix3f PSM_Matrix3fCreateFromQuatf(const PSMQuatf *q)
{
	PSMMatrix3f mat;

	const float qw = q->w;
	const float qx = q->x;
	const float qy = q->y;
	const float qz = q->z;

	const float qx2 = qx*qx;
	const float qy2 = qy*qy;
	const float qz2 = qz*qz;

	mat.m[0][0] = 1.f - 2.f*qy2 - 2.f*qz2; mat.m[0][1] = 2.f*qx*qy - 2.f*qz*qw;   mat.m[0][2] = 2.f*qx*qz + 2.f*qy*qw;
	mat.m[1][0] = 2.f*qx*qy + 2.f*qz*qw;   mat.m[1][1] = 1.f - 2.f*qx2 - 2.f*qz2; mat.m[1][2] = 2.f*qy*qz - 2.f*qx*qw;
	mat.m[2][0] = 2.f*qx*qz - 2.f*qy*qw;   mat.m[2][1] = 2.f * qy*qz + 2.f*qx*qw; mat.m[2][2] = 1.f - 2.f*qx2 - 2.f*qy2;

	return mat;
}

PSMVector3f PSM_Matrix3fBasisX(const PSMMatrix3f *mat)
{
	return {mat->m[0][0], mat->m[0][1], mat->m[0][2]};
}

PSMVector3f PSM_Matrix3fBasisY(const PSMMatrix3f *mat)
{
	return {mat->m[1][0], mat->m[1][1], mat->m[1][2]};
}

PSMVector3f PSM_Matrix3fBasisZ(const PSMMatrix3f *mat)
{
	return {mat->m[2][0], mat->m[2][1], mat->m[2][2]};
}

// PSMPosef
PSMPosef PSM_PosefCreate(const PSMVector3f *position, const PSMQuatf *orientation)
{
	return {*position, *orientation};
}

PSMPosef PSM_PosefInverse(const PSMPosef *pose)
{
	PSMQuatf q_inv = PSM_QuatfConjugate(&pose->Orientation);
	PSMPosef result;

	result.Orientation = q_inv;
	result.Position = PSM_QuatfRotateVector(&q_inv, &pose->Position);
	result.Position = PSM_Vector3fScale(&result.Position, -1.f);

	return result;
}

PSMPosef PSM_PosefConcat(const PSMPosef *first, const PSMPosef *second)
{
	PSMPosef result;

	result.Orientation = PSM_QuatfConcat(&first->Orientation, &second->Orientation);
	result.Position = PSM_QuatfRotateVector(&second->Orientation, &first->Position);
    result.Position = PSM_Vector3fAdd(&result.Position, &second->Position);

	return result;
}

PSMVector3f PSM_PosefTransformPoint(const PSMPosef *pose, const PSMVector3f *p)
{
	PSMVector3f result= PSM_QuatfRotateVector(&pose->Orientation, p);
	result= PSM_Vector3fAdd(&result, &pose->Position);

	return result;
}

PSMVector3f PSM_PosefInverseTransformPoint(const PSMPosef *pose, const PSMVector3f *p)
{
	PSMQuatf q_inv = PSM_QuatfConjugate(&pose->Orientation);
	PSMVector3f unrotate_p= PSM_QuatfRotateVector(&q_inv, p);
	PSMVector3f unrotate_pose_position= PSM_QuatfRotateVector(&q_inv, &pose->Position);
	PSMVector3f result = PSM_Vector3fSubtract(&unrotate_p, &unrotate_pose_position);

	return result;
}

// PSMFrustumf
void PSM_FrustumSetPose(PSMFrustum *frustum, const PSMPosef *pose)
{
    const glm::quat orientation(pose->Orientation.w, pose->Orientation.x, pose->Orientation.y, pose->Orientation.z);
    const glm::vec3 position(pose->Position.x, pose->Position.y, pose->Position.z);
    const glm::mat4 rot = glm::mat4_cast(orientation);
    const glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
    const glm::mat4 glm_mat4 = trans * rot;

    frustum->forward = {glm_mat4[2].x, glm_mat4[2].y, glm_mat4[2].z}; // z-axis
    frustum->left = {glm_mat4[0].x, glm_mat4[0].y, glm_mat4[0].z}; // x-axis
    frustum->up = {glm_mat4[1].x, glm_mat4[1].y, glm_mat4[1].z}; // y-axis

    frustum->origin = {glm_mat4[3].x, glm_mat4[3].y, glm_mat4[3].z};
}

// -- PSMoveTrackingProjection -- 
float PSM_TrackingProjectionGetArea(const PSMTrackingProjection *proj)
{
	float area = 0.f;

	switch (proj->shape_type)
	{
	case PSMTrackingProjection::PSMShape_Ellipse:
		{
			area = k_real_pi*proj->shape.ellipse.half_x_extent*proj->shape.ellipse.half_y_extent;
		} break;
	case PSMTrackingProjection::PSMShape_LightBar:
		{
			PSMVector2f edge1 = PSM_Vector2fSubtract(&proj->shape.lightbar.quad[0], &proj->shape.lightbar.quad[1]);
			PSMVector2f edge2 = PSM_Vector2fSubtract(&proj->shape.lightbar.quad[0], &proj->shape.lightbar.quad[3]);

			area = PSM_Vector2fLength(&edge1)*PSM_Vector2fLength(&edge2);
		} break;
	}

	return area;
}
