/* Performance measurement structures and functions */
#ifndef __UNIT_TEST_H
#define __UNIT_TEST_H

//-- macros ----
#define UNIT_TEST_SUITE_BEGIN() \
	bool success = true; \
	fprintf(stdout, "Running Unit Tests.\n"); \

#define UNIT_TEST_SUITE_CALL_CPP_MODULE(method) \
	bool method(); \
	success&= method(); \

#define UNIT_TEST_SUITE_DECLARE_C_MODULE(method) \
	extern "C" { bool method(); }; 

#define UNIT_TEST_SUITE_CALL_C_MODULE(method) \
	success&= method(); \

#define UNIT_TEST_SUITE_END() \
if (success) \
{ \
	fprintf(stdout, "All Unit Tests Passed.\n"); \
} \
else \
{ \
	fprintf(stdout, "Some Unit Tests Failed!.\n"); \
} \

#define UNIT_TEST_MODULE_BEGIN(name) \
	bool success = true; \
	const char *__module_name= name; \
	fprintf(stdout, "[%s]\n", __module_name); \

#define UNIT_TEST_MODULE_CALL_TEST(method) \
	bool method(); \
	success&= method(); \

#define UNIT_TEST_MODULE_END() \
 	fprintf(stdout, "  %s module - %s\n", __module_name, success ? "PASSED" : "FAILED"); \
	return success; \

#define UNIT_TEST_BEGIN(name) \
	const char *__test_name= name; \
	bool success= true; \

#define UNIT_TEST_COMPLETE() \
 	fprintf(stdout, "    %s - %s\n", __test_name, success ? "PASSED" : "FAILED"); \
	return success; \

#endif // __UNIT_TEST_H