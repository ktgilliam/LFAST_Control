#include "../00_Utils/math_util.h"
#include <gtest/gtest.h>

# define TEST_M_E		2.7182818284590452354	/* e */
# define TEST_M_LOG2E	1.4426950408889634074	/* log_2 e */
# define TEST_M_LOG10E	0.43429448190325182765	/* log_10 e */
# define TEST_M_LN2		0.69314718055994530942	/* log_e 2 */
# define TEST_M_LN10		2.30258509299404568402	/* log_e 10 */
# define TEST_M_PI		3.14159265358979323846	/* pi */
# define TEST_M_PI_2		1.57079632679489661923	/* pi/2 */
# define TEST_M_PI_4		0.78539816339744830962	/* pi/4 */
# define TEST_M_1_PI		0.31830988618379067154	/* 1/pi */
# define TEST_M_2_PI		0.63661977236758134308	/* 2/pi */
# define TEST_M_2_SQRTPI	1.12837916709551257390	/* 2/sqrt(pi) */
# define TEST_M_SQRT2	1.41421356237309504880	/* sqrt(2) */
# define TEST_M_SQRT1_2	0.70710678118654752440	/* 1/sqrt(2) */

#define THRESH 0.00000000000000001
TEST(math_util_tests, testdeg2rad)
{
    double test1_90 = 90.0;
    double test1_pi_2 = deg2rad(test1_90);
    double test1_correct_answer = TEST_M_PI*0.5;
    EXPECT_NEAR(test1_pi_2, test1_correct_answer, THRESH);

    double test2_pi_2 = TEST_M_PI*0.5;
    double test2_90 = rad2deg(test2_pi_2);
    double test2_correct_answer = 90.0;
    EXPECT_NEAR(test2_90, test2_correct_answer, THRESH);
}