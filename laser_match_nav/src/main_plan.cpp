#include "ChassisPlanner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_plan");

    // 获取小车参数
    double separation=0,radius=0;
    double max_speed, accel_dist, accel_time, ctrl_period;
    ros::param::param<double>("~wheel_separation", separation, 0.37);
    ros::param::param<double>("~wheel_radius", radius, 0.0625);
    ros::param::param<double>("~max_speed", max_speed, 0.3);
    ros::param::param<double>("~accel_dist", accel_dist, 0.1);    // 加减速距离
    ros::param::param<double>("~accel_time", accel_time, 1);    // 加减速时间
    ros::param::param<double>("~ctrl_period", ctrl_period, 40);   // 控制周期，单位ms

    plan::ChassisPlanner chassis_planner(separation, radius, max_speed, accel_dist, accel_time, ctrl_period);

    int rate_hz = 1000/ctrl_period;
    ros::Rate loop_rate(rate_hz);

    while(ros::ok()) {
        chassis_planner.AutoPlanner();
        // lidar_plan.ChassisStatus();

        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
