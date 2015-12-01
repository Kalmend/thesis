#include <gtest/gtest.h>

#include <ros/ros.h>

TEST(TestSuite1, testCase1)
{
   ASSERT_TRUE(true);
}

int main(int argc, char** argv)
{
   testing::InitGoogleTest(&argc, argv);
   return RUN_ALL_TESTS();
}
