#include <gtest/gtest.h>

#include <ros/ros.h>

#include "demo_class.h"

class DemoTestSuite : public ::testing::Test
{
 protected:
  demo_namespace::DemoClass* demo;

  virtual void SetUp()
  {
    char* args = new char[10];
    char** argv = &args;
    int argc = 0;
    ros::init(argc, argv, "demo_class_test");
    demo = new demo_namespace::DemoClass();
  }

  virtual void TearDown()
  {
    // Teardown here     
  }

};

TEST_F(DemoTestSuite, testCase1)
{
  ASSERT_TRUE(demo->demo_function());
}

/*
   TEST(TestSuite1, testCase1)
   {
   ASSERT_TRUE(true);
   }*/

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
