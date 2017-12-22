#ifndef MATH_UTILITY_H
#define MATH_UTILITY_H

//-- includes -----
#include <assert.h>
#include <float.h>
#include <math.h>

//-- constants ----
#define k_real_max FLT_MAX
#define k_real_min FLT_MIN

#define k_positional_epsilon 0.001f
#define k_normal_epsilon 0.0001f
#define k_real_epsilon FLT_EPSILON

#define k_real64_positional_epsilon 0.001
#define k_real64_normal_epsilon 0.0001
#define k_real64_epsilon DBL_EPSILON

#define k_real_pi 3.14159265f
#define k_real_two_pi 2.f*k_real_pi // 360 degrees
#define k_real_half_pi 0.5f*k_real_pi // 90 degrees
#define k_real_quarter_pi 0.25f*k_real_pi // 45 degrees

#define k_degrees_to_radians (k_real_pi / 180.f)
#define k_radians_to_degreees (180.f / k_real_pi)

#define k_real64_pi 3.14159265358979323846
#define k_real64_two_pi 2.0*k_real64_pi // 360 degrees
#define k_real64_half_pi 0.5*k_real64_pi // 90 degrees
#define k_real64_quarter_pi 0.25*k_real64_pi // 45 degrees

#define k_real64_degrees_to_radians (k_real64_pi / 180.0)
#define k_real64_radians_to_degreees (180.0 / k_real64_pi)

//-- macros ----
#ifdef isfinite
#define is_valid_float(x) (!std::isnan(x) && isfinite(x))
#else
#define is_valid_float(x) (!std::isnan(x))
#endif

#define is_nearly_equal(a, b, epsilon) (fabsf((a)-(b)) <= (epsilon))
#define is_nearly_zero(x) is_nearly_equal(x, 0.0f, k_real_epsilon)

#define is_double_nearly_equal(a, b, epsilon) (fabs((a)-(b)) <= (epsilon))
#define is_double_nearly_zero(x) is_double_nearly_equal(x, 0.0, k_real64_epsilon)

#ifndef sgn
#define sgn(x) (((x) >= 0) ? 1 : -1)
#endif

#ifdef NDEBUG
#define assert_valid_float(x) assert(is_valid_float(x))
#else
#define assert_valid_float(x)     ((void)0)
#endif

//-- float methods -----
float safe_divide_with_default(float numerator, float denomenator, float default_result);
double safe_divide_with_default(double numerator, double denomenator, double default_result);
float safe_sqrt_with_default(float square, float default_result);
double safe_sqrt_with_default(double square, double default_result);
float clampf(float x, float lo, float hi);
float clampf01(float x);
float lerpf(float a, float b, float u);
float lerp_clampf(float a, float b, float u);
float degrees_to_radians(float x);
float radians_to_degrees(float x);
float wrap_radians(float angle);
float wrap_degrees(float angle);
float wrap_range(float value, float range_min, float range_max);
double wrap_ranged(double value, double range_min, double range_max);
float wrap_lerpf(float a, float b, float u, float range_min, float range_max);

#endif // MATH_UTILITY_h
