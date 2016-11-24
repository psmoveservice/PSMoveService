//-- includes -----
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "MathAlignment.h"
#include "MathUtility.h"
#include "unit_test.h"

//-- public interface -----
bool run_math_alignment_unit_tests()
{
	UNIT_TEST_MODULE_BEGIN("math_alignment")
		UNIT_TEST_MODULE_CALL_TEST(math_alignment_test_best_fit_exponential);
	UNIT_TEST_MODULE_END()
}

//-- private functions -----
bool
math_alignment_test_best_fit_exponential()
{
	UNIT_TEST_BEGIN("best_fit_exponential")

	const int k_sample_count = 10;
	Eigen::Vector2f samples[k_sample_count] = {
		Eigen::Vector2f(2161.6f, 0.00055f),
		Eigen::Vector2f(1726.9f, 0.0011f),
		Eigen::Vector2f(1124.6f, 0.00028f),
		Eigen::Vector2f(704.2f, 0.00051f),
		Eigen::Vector2f(367.1f, 0.0163f),
		Eigen::Vector2f(262.2f, 0.2343f),
		Eigen::Vector2f(172.9f, 0.529f),
		Eigen::Vector2f(139.1f, 1.8f),
		Eigen::Vector2f(108.5f, 1.316f),
		Eigen::Vector2f(68.85f, 0.9238f)
	};
		
	Eigen::Vector2f curve;
	success= eigen_alignment_fit_least_squares_exponential(samples, k_sample_count, &curve);
	assert(success);
	
	// Should be close to y(x)= 0.4488802f*exp(-0.00402157f*x)
	success = is_nearly_equal(curve.x(), -0.00402157f, k_normal_epsilon);
	assert(success);
	success = is_nearly_equal(curve.y(), 0.4488802f, k_normal_epsilon);
	assert(success);	
	
	UNIT_TEST_COMPLETE()
}