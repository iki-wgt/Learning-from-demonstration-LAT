// Bring in my package's API, which is what I'm testing
#include "leatra/leatra.hh"
// Bring in gtest
#include <gtest/gtest.h>

ndmap createNdmap()
{
	double dim1[] = {1.0, 1.0, 1.2, 2.1, 2.1, 3.0, 2.5, 2.5, 3.2, 2.7, 2.7, 1.3, 1.3, 3.0, 3.0};
	double dim2[] = {0.0, 1.0, 1.2, 5.1, 5.1, 7.0, 5.5, 4.5, 8.2, 2.7, 2.7, 1.3, 1.3, 3.0, 3.0};
	double dim3[] = {1.0, 1.0, 1.2, 22.1, 22.1, 33.0, 22.5, 2.5, 3.2, 2.7, 2.7, 1.3, 1.3, 3.0, 3.0};

	ndmap ndm;

	ndm.add_deque(std::deque<double>());
	ndm.add_deque(std::deque<double>());
	ndm.add_deque(std::deque<double>());

	for (int i = 0; i < 15; ++i)
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
		EXPECT_DOUBLE_EQ(1.0, min) << "Min value of row 0 not  equal 1";
		EXPECT_DOUBLE_EQ(3.2, max) << "Max value of row 0 not  equal 3.2";

		ndm.getMinMax(1, min, max);
		EXPECT_DOUBLE_EQ(0.0, min) << "Min value of row 1 not  equal 0";
		EXPECT_DOUBLE_EQ(8.2, max) << "Max value of row 1 not  equal 8.2";

		ndm.getMinMax(2, min, max);
		EXPECT_DOUBLE_EQ(1.0, min) << "Min value of row 2 not  equal 1";
		EXPECT_DOUBLE_EQ(33.0, max) << "Max value of row 2 not  equal 33.0";
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
	EXPECT_EQ(2, dimWithMaxDev) << "Returned dim not the one with max dev";
}

TEST(TestSuite, testGetConstraints)
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
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
