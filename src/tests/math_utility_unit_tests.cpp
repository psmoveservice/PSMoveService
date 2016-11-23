//-- includes -----
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "MathUtility.h"
#include "unit_test.h"

//-- public interface -----
bool run_math_utility_unit_tests()
{
	UNIT_TEST_MODULE_BEGIN("math_utility")
		UNIT_TEST_MODULE_CALL_TEST(math_utility_test_wrap_lerpf);
	UNIT_TEST_MODULE_END()
}

//-- private functions -----
bool
math_utility_test_wrap_lerpf()
{
	UNIT_TEST_BEGIN("wrap lerpf")

	for (float u = 0.f; success && u <= 1.f; u += 0.1f)
	{
		float wrap_lerp_result = wrap_lerpf(0, 60.f, u, -180.f, 180.f);
		float lerp_result = lerpf(0.f, 60.f, u);

		success = is_nearly_equal(wrap_lerp_result, lerp_result, k_normal_epsilon);
		assert(success);
	}

	for (float u = 0.f; success && u <= 1.f; u += 0.1f)
	{
		float wrap_lerp_result = wrap_lerpf(-60, 60.f, u, -180.f, 180.f);
		float lerp_result = lerpf(-60.f, 60.f, u);

		success = is_nearly_equal(wrap_lerp_result, lerp_result, k_normal_epsilon);
		assert(success);
	}

	for (float u = 0.f; success && u <= 0.5f; u += 0.1f)
	{
		float wrap_lerp_result = wrap_lerpf(-170.f, 170.f, u, -180.f, 180.f);
		float lerp_result = wrap_range(lerpf(-170.f, -190.f, u), -180.f, 180.f);

		success = is_nearly_equal(wrap_lerp_result, lerp_result, k_normal_epsilon);
		assert(success);
	}
	for (float u = 0.5f; success && u <= 1.f; u += 0.1f)
	{
		float wrap_lerp_result = wrap_lerpf(-170.f, 170.f, u, -180.f, 180.f);
		float lerp_result = wrap_range(lerpf(190.f, 170.f, u), -180.f, 180.f);

		success = is_nearly_equal(wrap_lerp_result, lerp_result, k_normal_epsilon);
		assert(success);
	}

	UNIT_TEST_COMPLETE()
}