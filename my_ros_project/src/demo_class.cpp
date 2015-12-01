#include "demo_class.h"

namespace demo_namespace
{

DemoClass::DemoClass()
{
   ROS_INFO("Constructor!");
}

bool DemoClass::demo_function()
{
   ROS_INFO("demo_function()");
   return true;
}

}
