//-- includes -----
#include "MathUtility.h"

//-- float methods -----
float safe_divide_with_default(float numerator, float denomenator, float default_result) 
{
    return is_nearly_zero(denomenator) ? default_result : (numerator / denomenator);
}

float safe_sqrt_with_default(float square, float default_result)
{
    return is_nearly_zero(square) ? default_result : sqrtf(square);
}

float clampf(float x, float lo, float hi)
{
	return fminf(fmaxf(x, lo), hi);
}

float clampf01(float x)
{
	return clampf(x, 0.f, 1.f);
}

float lerpf(float a, float b, float u)
{
	return a*(1.f - u) + b*u;
}

float lerp_clampf(float a, float b, float u)
{
	return clampf(lerpf(a, b, u), a, b);
}

float degrees_to_radians(float x)
{
	return ((x * k_real_pi) / 180.f);
}

float radians_to_degrees(float x)
{
	return ((x * 180.f) / k_real_pi);
}

float wrap_radians(float angle)
{
    return fmodf(angle + k_real_two_pi, k_real_two_pi);
}

float wrap_degrees(float angle)
{
    return fmodf(angle + 360.f, 360.f);
}

float wrap_range(float value, float range_min, float range_max)
{
    assert(range_max > range_min);
    const float range = range_max - range_min;
    
    return range_min + fmodf((value - range_min) + range, range);
}

double wrap_ranged(double value, double range_min, double range_max)
{
	assert(range_max > range_min);
	const double range = range_max - range_min;

	return range_min + fmod((value - range_min) + range, range);
}

float wrap_lerpf(float a, float b, float u, float range_min, float range_max)
{
	assert(range_max > range_min);
	const float range = range_max - range_min;
	float wrapped_a = a;
	float wrapped_b = b;

	if (fabsf(a - b) >= (range / 2.f))
	{
		if (a > b)
			wrapped_a = wrap_range(a, range_min, range_max) - range;
		else
			wrapped_b = wrap_range(b, range_min, range_max) - range;
	}

	return wrap_range(lerpf(wrapped_a, wrapped_b, u), range_min, range_max);
}