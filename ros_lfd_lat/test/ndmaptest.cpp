// Bring in my package's API, which is what I'm testing
#include "leatra/leatra.hh"
// Bring in gtest
#include <gtest/gtest.h>

ndmap createNdmap()
{
	// values from the coke

	double dim1[] = {0.03, 0.05, 0.025, 0.01, 0.01, 0.009, 0.025, 0.025, 0.1, 0.11, 0.11, 0.07, 0.05, 0.05};	// values from the cup
	double dim2[] = {0.22, 0.4, 0.5, 0.6, 0.6, 0.6, 0.5, 0.11, 0.015, 0.015, 0.015, 0.018, 0.38, 0.35};
	double dim3[] = {0.03, 0.02, 0.07, 0.03, 0.04, 0.04, 0.02, 0.05, 0.01, 0.01, 0.01, 0.06, 0.03, 0.07};

	ndmap ndm;

	ndm.add_deque(std::deque<double>());
	ndm.add_deque(std::deque<double>());
	ndm.add_deque(std::deque<double>());

	for (int i = 0; i < 14; ++i)
	{
		std::deque<double> point;
		point.push_back(dim1[i]);
		point.push_back(dim2[i]);
		point.push_back(dim3[i]);
		ndm.push_back(point);
	}

	return ndm;
}

ndmapSet createNdmapSet(std::string name)
{
	ndmapSet ndmSet;
	ndmSet.set_name(name);
	ndmap ndm = createNdmap();
	ndm.set_name(ndmSet.get_name() + "_mean+stdev");
	ndmSet.add_ndmap(ndm);
	ndm.set_name(ndmSet.get_name() + "_mean");
	ndmSet.add_ndmap(ndm);
	ndm.set_name(ndmSet.get_name() + "_mean-stdev");
	ndmSet.add_ndmap(ndm);
	ndm.set_name(ndmSet.get_name() + "_stdev");
	ndmSet.add_ndmap(ndm);

	return ndmSet;
}

TEST(TestSuite, testGetMinMax)
{
	ndmap ndm = createNdmap();

	int dim = ndm.get_dim();
	ASSERT_EQ(3, dim) << "Dimension of ndmap not equal 3";

	double min;
	double max;

	try
	{
		ndm.getMinMax(0, min, max);
		EXPECT_DOUBLE_EQ(0.009, min) << "Min value of row 0 not  equal 0.009";
		EXPECT_DOUBLE_EQ(0.11, max) << "Max value of row 0 not  equal 0.11";

		ndm.getMinMax(1, min, max);
		EXPECT_DOUBLE_EQ(0.015, min) << "Min value of row 1 not  equal 0.015";
		EXPECT_DOUBLE_EQ(0.6, max) << "Max value of row 1 not  equal 0.6";

		ndm.getMinMax(2, min, max);
		EXPECT_DOUBLE_EQ(0.01, min) << "Min value of row 2 not  equal 0.01";
		EXPECT_DOUBLE_EQ(0.07, max) << "Max value of row 2 not  equal 0.07";
	}
	catch(data_error& e)
	{
		e.print();
		FAIL() << "data_error exception!";
	}
}

TEST(TestSuite, testGetDimWithMaxDev)
{
	ndmap ndm = createNdmap();

	unsigned int dimWithMaxDev = ndm.getDimWithMaxDev();
	EXPECT_EQ(1, dimWithMaxDev) << "Returned dim not the one with max dev";
}

TEST(TestSuite, testGetConstraints)
{
	ndmapSet ndmSet = createNdmapSet("object1");
	double threshold1 = 0.015;
	double threshold2 = 0.11;
	bool resultArray1[] =
		{false, false, false, false, false, false, false, false, true, true, true, false, false, false};
	bool resultArray2[] =
		{false, false, false, false, false, false, false, true, true, true, true, true, false, false};

	std::deque<bool> expectedResult1 = std::deque<bool>();
	std::deque<bool> expectedResult2 = std::deque<bool>();

	for (int i = 0; i < 14; ++i)
	{
		expectedResult1.push_back(resultArray1[i]);
		expectedResult2.push_back(resultArray2[i]);
	}

	std::deque<bool> result1 = ndmSet.getConstraints(threshold1);

	ASSERT_EQ(expectedResult1.size(), result1.size()) << "Size of result 1 incorrect";

	std::deque<bool> result2 = ndmSet.getConstraints(threshold2);

	ASSERT_EQ(expectedResult2.size(), result2.size()) << "Size of result 2 incorrect";

	for (unsigned int i = 0; i < expectedResult1.size(); ++i) {
		ASSERT_EQ(expectedResult1[i], result1[i]) << "Result 1 incorrect";
		ASSERT_EQ(expectedResult2[i], result2[i]) << "Result 2 incorrect";
	}
}

/*TEST(TestSuite, testGetConstraintsGroup)
{
	ndmapSet ndmSet = createNdmapSet("object1");
	double threshold1 = 1.2;
	double threshold2 = 1.3;
	bool resultArray1[] =
		{true, true, true, false, false, false, false, false, false, false, false, false, false, false, false};
	bool resultArray2[] =
		{true, true, true, false, false, false, false, false, false, false, false, true, true, false, false};

	std::deque<bool> expectedResult1 = std::deque<bool>();
	std::deque<bool> expectedResult2 = std::deque<bool>();

	for (int i = 0; i < 15; ++i)
	{
		expectedResult1.push_back(resultArray1[i]);
		expectedResult2.push_back(resultArray2[i]);
	}

	std::deque<bool> result1 = ndmSet.getConstraints(threshold1);

	ASSERT_EQ(expectedResult1.size(), result1.size()) << "Size of result 1 incorrect";

	std::deque<bool> result2 = ndmSet.getConstraints(threshold2);

	ASSERT_EQ(expectedResult2.size(), result2.size()) << "Size of result 2 incorrect";

	for (unsigned int i = 0; i < expectedResult1.size(); ++i) {
		ASSERT_EQ(expectedResult1[i], result1[i]) << "Result 1 incorrect";
		ASSERT_EQ(expectedResult2[i], result2[i]) << "Result 2 incorrect";
	}
}*/

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
