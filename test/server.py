include "ros/ros.h"
include "beginner_tutorials/AddTwoInts.h"
 
bool add(beginner_tutorials::AddTwoInts::Request  &req,
        beginner_tutorials::AddTwoInts::Response &res)
{
    if (req.a > req.b)
    {
        return req.a;
    }
    else
    {
        return req.b;
    }
}
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "greater_value_server");
   ros::NodeHandle n;
 
   ros::ServiceServer service = n.advertiseService("greater_value", add);
   ros::spin();
 
   return 0;
}