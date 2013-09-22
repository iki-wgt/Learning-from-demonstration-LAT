// Bring in my package's API, which is what I'm testing
//#include "ros_lfd_lat/lat_reproducer.h"
// Bring in gtest
#include <gtest/gtest.h>

// hack from https://code.google.com/p/googletest/wiki/FAQ#How_do_I_test_a_file_that_defines_main()?
// Renames main() in foo.cc to avoid redefinition by the unit test main()
#define main FooMain
#include "../src/lat_reproducer.cpp"
#undef main

TEST(TestSuite, testObjectUnderConstraint)
{
	int constraintArray[] =
		{-1, -1, -1, 1, 1, 1, -1, -1, 0, 0, 0, 0, -1, -1};

	std::deque<int> constraints = std::deque<int>();

	for (int i = 0; i < 14; ++i)
	{
		constraints.push_back(constraintArray[i]);
	}

	int objectId = 1;
	unsigned int step = 3;

	EXPECT_EQ(true, objectUnderConstraint(objectId, step, constraints)) << "erster Test ungleich true";

	EXPECT_NE(true, objectUnderConstraint(1, 2, constraints));

	objectId = 1;
	step = 5;
	EXPECT_EQ(true, objectUnderConstraint(objectId, step, constraints));

	EXPECT_NE(true, objectUnderConstraint(1, 0, constraints));

	EXPECT_NE(true, objectUnderConstraint(1, 6, constraints));

	EXPECT_NE(true, objectUnderConstraint(1, 8, constraints));

	EXPECT_NE(true, objectUnderConstraint(1, 13, constraints));

	EXPECT_NE(true, objectUnderConstraint(0, 0, constraints));

	EXPECT_NE(true, objectUnderConstraint(0, 6, constraints));

	objectId = 0;
	step = 5;
	EXPECT_NE(true, objectUnderConstraint(objectId, step, constraints));

	objectId = 0;
	step = 11;
	EXPECT_EQ(true, objectUnderConstraint(objectId, step, constraints));
}

TEST(TestSuite, testObjectAfterConstraint)
{
	int constraintArray[] =
		{-1, -1, -1, 1, 1, 1, -1, -1, 0, 0, 0, 0, -1, -1};

	std::deque<int> constraints = std::deque<int>();

	for (int i = 0; i < 14; ++i)
	{
		constraints.push_back(constraintArray[i]);
	}

	int objectId = 1;
	unsigned int step = 3;

	EXPECT_NE(true, objectAfterConstraint(objectId, step, constraints)) << "erster Test ungleich false";

	EXPECT_NE(true, objectAfterConstraint(1, 2, constraints));

	objectId = 1;
	step = 5;
	EXPECT_NE(true, objectAfterConstraint(objectId, step, constraints));

	EXPECT_NE(true, objectAfterConstraint(1, 0, constraints));

	EXPECT_EQ(true, objectAfterConstraint(1, 6, constraints));

	EXPECT_EQ(true, objectAfterConstraint(1, 8, constraints));

	EXPECT_EQ(true, objectAfterConstraint(1, 13, constraints));

	EXPECT_NE(true, objectAfterConstraint(0, 0, constraints));

	EXPECT_NE(true, objectAfterConstraint(0, 6, constraints));

	objectId = 0;
	step = 5;
	EXPECT_NE(true, objectAfterConstraint(objectId, step, constraints));

	objectId = 0;
	step = 11;
	EXPECT_NE(true, objectAfterConstraint(objectId, step, constraints));

	EXPECT_EQ(true, objectAfterConstraint(0, 12, constraints));

	EXPECT_EQ(true, objectAfterConstraint(0, 13, constraints));
}

TEST(TestSuite, testIsObjectReachable)
{
	geometry_msgs::PointStamped objectLocation = geometry_msgs::PointStamped();

	objectLocation.header.frame_id = "/katana_base_link";

	objectLocation.point.x = 1;		// to the front
	objectLocation.point.y = 1;		// to the side
	objectLocation.point.z = 1;		// up

	EXPECT_NE(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0.4;
	objectLocation.point.y = 0;
	objectLocation.point.z = 0.2;

	EXPECT_EQ(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0.4;
	objectLocation.point.y = 0;
	objectLocation.point.z = 0.3;

	EXPECT_EQ(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0;
	objectLocation.point.y = 0;
	objectLocation.point.z = 0.83;

	EXPECT_NE(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0;
	objectLocation.point.y = 0;
	objectLocation.point.z = 0.82;

	EXPECT_EQ(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0.61;
	objectLocation.point.y = 0;
	objectLocation.point.z = 0.22;

	EXPECT_NE(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0.6;
	objectLocation.point.y = 0;
	objectLocation.point.z = 0.22;

	EXPECT_EQ(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0;
	objectLocation.point.y = 0.61;
	objectLocation.point.z = 0.22;

	EXPECT_NE(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0;
	objectLocation.point.y = 0.6;
	objectLocation.point.z = 0.22;

	EXPECT_EQ(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0;
	objectLocation.point.y = -0.61;
	objectLocation.point.z = 0.22;

	EXPECT_NE(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0;
	objectLocation.point.y = -0.6;
	objectLocation.point.z = 0.22;

	EXPECT_EQ(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0.15;
	objectLocation.point.y = 0.0;
	objectLocation.point.z = -0.31;

	EXPECT_NE(true, isObjectReachable(objectLocation));

	objectLocation.point.x = 0.15;
	objectLocation.point.y = 0.0;
	objectLocation.point.z = -0.3;

	EXPECT_EQ(true, isObjectReachable(objectLocation));
}

TEST(TestSuite, testStepsTillConstraint)
{
	int constraintArray[] =
		{-1, -1, -1, 1, 1, 1, -1, -1, 0, 0, 0, 0, -1, -1};

	std::deque<int> constraints = std::deque<int>();

	for (int i = 0; i < 14; ++i)
	{
		constraints.push_back(constraintArray[i]);
	}

	// object id 1
	EXPECT_EQ(3, stepsTillConstraint(1, 0, constraints));
	EXPECT_EQ(2, stepsTillConstraint(1, 1, constraints));
	EXPECT_EQ(1, stepsTillConstraint(1, 2, constraints));

	EXPECT_EQ(0, stepsTillConstraint(1, 3, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 4, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 5, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 6, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 7, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 8, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 9, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 10, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 11, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 12, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 13, constraints));

	EXPECT_EQ(0, stepsTillConstraint(1, 14, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 99, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 4000, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 763356, constraints));

	// object id 0
	EXPECT_EQ(8, stepsTillConstraint(0, 0, constraints));
	EXPECT_EQ(7, stepsTillConstraint(0, 1, constraints));
	EXPECT_EQ(6, stepsTillConstraint(0, 2, constraints));
	EXPECT_EQ(5, stepsTillConstraint(0, 3, constraints));
	EXPECT_EQ(4, stepsTillConstraint(0, 4, constraints));
	EXPECT_EQ(3, stepsTillConstraint(0, 5, constraints));
	EXPECT_EQ(2, stepsTillConstraint(0, 6, constraints));
	EXPECT_EQ(1, stepsTillConstraint(0, 7, constraints));
	EXPECT_EQ(0, stepsTillConstraint(0, 8, constraints));
	EXPECT_EQ(0, stepsTillConstraint(0, 9, constraints));
	EXPECT_EQ(0, stepsTillConstraint(0, 10, constraints));
	EXPECT_EQ(0, stepsTillConstraint(0, 11, constraints));
	EXPECT_EQ(0, stepsTillConstraint(0, 12, constraints));
	EXPECT_EQ(0, stepsTillConstraint(0, 13, constraints));

	EXPECT_EQ(0, stepsTillConstraint(1, 14, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 98, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 4001, constraints));
	EXPECT_EQ(0, stepsTillConstraint(1, 763756, constraints));
}

TEST(TestSuite, testBlaBla)
{

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
