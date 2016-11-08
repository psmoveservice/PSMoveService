//-- includes -----
#include <stdio.h>
#include <stdlib.h>
#include "unit_test.h"

//-- prototypes -----

//-- entry point -----
int
main(int argc, char* argv[])
{
	UNIT_TEST_SUITE_BEGIN()
		UNIT_TEST_SUITE_CALL_CPP_MODULE(run_math_utility_unit_tests);
	UNIT_TEST_SUITE_END()

	return success ? EXIT_SUCCESS : EXIT_FAILURE;
}