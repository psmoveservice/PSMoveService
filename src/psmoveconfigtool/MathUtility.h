#ifndef MATH_UTILITY_H
#define MATH_UTILITY_H

//-- includes -----
#include <float.h>
#include <math.h>

//-- constants ----
#define k_real_max FLT_MAX
#define k_real_min FLT_MIN

#define k_positional_epsilon 0.001f
#define k_normal_epsilon 0.0001f
#define k_real_epsilon FLT_EPSILON

#define k_real_pi 3.14159265f
#define k_real_two_pi 2.f*k_real_pi
#define k_real_half_pi 0.5f*k_real_pi

//-- macros ----
#ifdef isfinite
#define is_valid_float(x) (!isnan(x) && isfinite(x))
#else
#define is_valid_float(x) (!isnan(x))
#endif

#define is_nearly_equal(a, b, epsilon) (fabsf(a-b) <= epsilon)
#define is_nearly_zero(x) is_nearly_equal(x, 0.0f, k_real_epsilon)

#define safe_divide_with_default(numerator, denomenator, default_result) (is_nearly_zero(denomenator) ? (default_result) : (numerator / denomenator));

#ifndef sgn
#define sgn(x) (((x) >= 0) ? 1 : -1)
#endif

#ifndef sqr
#define sqr(x) (x*x)
#endif

#ifdef NDEBUG
#define assert_valid_float(x) assert(is_valid_float(x))
#else
#define assert_valid_float(x)     ((void)0)
#endif

//-- methods -----
float clampf(float x, float lo, float hi);
float clampf01(float x);
float lerpf(float a, float b, float u);
float lerp_clampf(float a, float b, float u);
float degrees_to_radians(float x);
float radians_to_degrees(float x);
float wrap_radians(float angle);
float wrap_degrees(float angle);

#endif // MATH_UTILITY_h