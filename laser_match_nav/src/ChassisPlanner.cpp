#include "ChassisPlanner.h"



namespace plan {

ChassisPlanner::ChassisPlanner(){
    file.open("data.txt");

    // Head Direction
    m_head_flag = 1;
    // PID parameter
    last_ang_dist_up = 0;
    ang_integral_up = 0;
    last_ang_dist_slow = 0;
    ang_integral_slow = 0;
    last_ang_dist_ave = 0;
    ang_integral_ave = 0;

    //
    m_b_pose_init = false;
    m_b_end_adjust = false;

    // 到达8mm范围内，位置调整结束
    dist_thre = 8;
    // 1 degree 范围内，姿态调整结束
    theta_thre = 0.0174;

    head_dist = 1500;

    send_cmd.linear.x = 0;
    send_cmd.angular.z = 0;
    last_linear_cmd = 0;
    last_angular_cmd = 0;

    //
    T_Ref_in_Ori.setIdentity(3,3);
    T_Laser_in_Ref.setIdentity(3,3);
    T_Laser_in_Ori.setIdentity(3,3);

    //
    m_chassis_status = STOP;

    m_pose_suber = m_nh.subscribe("pose2D", 1000,
                                  &ChassisPlanner::PoseUpdateCallback, this);
    m_vel_puber = m_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    m_client = m_nh.serviceClient<laser_detect::laser_detect_srv>("global_calib");
}

ChassisPlanner::ChassisPlanner(double separation, double radius, double max_speed,
          double accel_dist, double accel_time, double ctrl_period){
    new (this)ChassisPlanner();

    m_ctrl_period = ctrl_period;
    m_separation = separation*1000;
    m_radius = radius*1000;
    m_max_speed = max_speed;
    m_accel_dist = accel_dist*1000;
    m_accel_time = accel_time;
    m_accel = 2*accel_dist/pow(accel_time, 2);
}


ChassisPlanner::~ChassisPlanner(){
    double linear_vel = 0.0;
    double angular_vel = 0.0;
    send_cmd.linear.x = linear_vel;
    send_cmd.angular.z = angular_vel;
    m_vel_puber.publish(send_cmd);

    file.close();
}


/**
* 更新机器人位姿
* @param chassis_pose [description]
*/
void ChassisPlanner::PoseUpdateCallback(const geometry_msgs::Pose2D& chassis_pose){
    double chassis_pose_x = chassis_pose.x*1000;
    double chassis_pose_y = chassis_pose.y*1000;
    double chassis_pose_theta = chassis_pose.theta;

    //
    boost::mutex::scoped_lock lock(m_mutex);
    // 成功启动的标志
    if (true == m_b_pose_init){
        double pose_Laser_in_Ref[3], pose_Laser_in_Ori[3];
        pose_Laser_in_Ref[0] = chassis_pose_x;
        pose_Laser_in_Ref[1] = chassis_pose_y;
        pose_Laser_in_Ref[2] = chassis_pose_theta;

        PoseToMat(pose_Laser_in_Ref, T_Laser_in_Ref);
        T_Laser_in_Ori = T_Ref_in_Ori*T_Laser_in_Ref;
        MatToPose(T_Laser_in_Ori, pose_Laser_in_Ori);

        pose_x = pose_Laser_in_Ori[0];
        pose_y =pose_Laser_in_Ori[1];
        pose_theta = pose_Laser_in_Ori[2];

        // Original -> Destination
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(dest_pose_x, dest_pose_y, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, dest_pose_theta);
        transform.setRotation(q);
        m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "original", "dest"));

        // Heading Direction
        if (dest_pose_x < pose_x){
            m_head_flag = -1;
        }
        else{
            m_head_flag = 1;
        }

        ROS_INFO("pose x is %f, pose y is %f, pose theta is %f\r\n",
                 pose_x, pose_y, pose_theta);
    }
    else{
        laser_detect::laser_detect_srv srv;
        srv.request.flag = true;

        ROS_INFO("IN service\r\n");

        if (m_client.call(srv)){

          ROS_INFO("IN Call\r\n");
            //
            mat3x3 T_Ori_in_Laser;
            double pose_Ori_in_Laser[3];
            pose_Ori_in_Laser[0] = srv.response.pose.x;
            pose_Ori_in_Laser[1] = srv.response.pose.y;
            pose_Ori_in_Laser[2] = srv.response.pose.theta;

            PoseToMat(pose_Ori_in_Laser, T_Ori_in_Laser);
            T_Laser_in_Ori = T_Ori_in_Laser.inverse();

            //
            double pose_Laser_in_Ref[3];
            pose_Laser_in_Ref[0] = chassis_pose_x;
            pose_Laser_in_Ref[1] = chassis_pose_y;
            pose_Laser_in_Ref[2] = chassis_pose_theta;

            PoseToMat(pose_Laser_in_Ref, T_Laser_in_Ref);

            //
            T_Ref_in_Ori = T_Laser_in_Ori*(T_Laser_in_Ref.inverse());

            //
            double pose_now[3];
            MatToPose(T_Laser_in_Ori, pose_now);

            start_pose_x = pose_now[0];
            start_pose_y = pose_now[1];
            start_pose_theta = pose_now[2];
            pose_x = pose_now[0];
            pose_y =pose_now[1];
            pose_theta = pose_now[2];
            dest_pose_x = -1.3*1000;
            // dest_pose_x = -1.0;
            dest_pose_y = -1.15*1000;
            dest_pose_theta = 0.0;

            if (dest_pose_x < pose_x){
                m_head_flag = -1;
            }
            else{
                m_head_flag = 1;
            }

            m_b_pose_init = true;

        }
        else{
            m_b_pose_init = false;

            ROS_INFO("Initialization Occurs Error!!!\r\n");
        }
                ROS_INFO("Out CALL\r\n");

    }
}


/**
* 状态判断
*/
void ChassisPlanner::StatusDef(){
    // double start_dist = sqrt(pow((pose_x-start_pose_x), 2) +
    //                           pow((pose_y-start_pose_y), 2));
    // double end_dist = sqrt(pow((pose_x-dest_pose_x), 2) +
    //                         pow((pose_y-dest_pose_y), 2));
    double start_dist = abs(pose_x-start_pose_x);
    double end_dist = abs(pose_x-dest_pose_x);
    double theta_dist = abs(pose_theta-dest_pose_theta);
    ROS_INFO("Start_dist is: %f, End_dist is: %f, Theta_dist is: %f\r\n",
             start_dist, end_dist, theta_dist);

    // 初始化未完成
    if (false == m_b_pose_init){
        m_chassis_status = STOP;
    }
    // 加速阶段
    else if (true == m_b_pose_init && start_dist <= m_accel_dist &&
             end_dist > m_accel_dist && SPEEDSLOW != m_chassis_status &&
             ADJUST != m_chassis_status && HOLDON!= m_chassis_status){
        m_chassis_status = SPEEDUP;
    }
    // 匀速阶段
    else if (true == m_b_pose_init && start_dist > m_accel_dist && end_dist > m_accel_dist &&
             (SPEEDUP == m_chassis_status || SPEEDCONST == m_chassis_status) && HOLDON != m_chassis_status){
        m_chassis_status = SPEEDCONST;
    }
    // 减速阶段
    else if (true == m_b_pose_init && start_dist > m_accel_dist &&
             end_dist <= m_accel_dist && end_dist >= dist_thre &&
             HOLDON != m_chassis_status){
        m_chassis_status = SPEEDSLOW;
    }
    // 位置到位，调整姿态
    else if (true == m_b_pose_init && end_dist < dist_thre && SPEEDSLOW == m_chassis_status &&
             theta_dist >= theta_thre && HOLDON != m_chassis_status){
        m_chassis_status = ADJUST;
    }
    // 到达终点
    else if (true == m_b_pose_init && end_dist < dist_thre && theta_dist < theta_thre &&
             (SPEEDSLOW == m_chassis_status || ADJUST == m_chassis_status) || (HOLDON == m_chassis_status)){
        m_chassis_status = HOLDON;
    }
    // 状态出现错误
    else{
        m_chassis_status = ERROR;
    }

    ROS_INFO("End, Chassis Status is %d\r\n", m_chassis_status);

    file << "Chassis Status is: " << m_chassis_status << std::endl;
    file << "Start Dist is: " << start_dist << std::endl;
    file << "End Dist is: " << end_dist << std::endl;
    file << "Theta Dist is: " << theta_dist << std::endl;
    file << "Pose x is: " << pose_x << std::endl;
    file << "Pose y is: " << pose_y << std::endl;
}

/**
* 速度规划
*/
void ChassisPlanner::AutoPlanner(){
    //
    boost::mutex::scoped_lock lock(m_mutex);

    double linear_vel, angular_vel;

    // 判断底盘状态
    StatusDef();

    // 停止状态
    if (STOP == m_chassis_status){
        linear_vel = 0;
        angular_vel = 0;

        last_linear_cmd = 0;
        last_angular_cmd = 0;
    }
    // 加速状态
    else if (SPEEDUP == m_chassis_status){
        double dist_kp = 1.5;
        double dist_ki = 0;
        double dist_kd = 0;
        double theta_kp = 0.6;
        double theta_ki = 0;
        double theta_kd = 0;

        static double ang_dist_integ_up = 0.0;
        static double ang_theta_integ_up = 0.0;
        static double last_ang_dist_up = 0.0;
        static double last_ang_theta_up = 0.0;

        // 直线速度
        linear_vel = last_linear_cmd + m_head_flag*m_accel*(m_ctrl_period/1000.0);

        // 角速度
        double ang_dist_up = asin((dest_pose_y-pose_y)/head_dist);
        double ang_theta_up = dest_pose_theta - pose_theta;
        // 积分误差
        ang_dist_integ_up += ang_dist_up;
        ang_theta_integ_up += ang_theta_up;
        // 角度PID
        double ang_dist_vel = AngVelPID(ang_dist_up, ang_dist_integ_up, last_ang_dist_up,
                                        dist_kp, dist_ki, dist_kd);
        double ang_theta_vel = AngVelPID(ang_theta_up, ang_theta_integ_up, last_ang_theta_up,
                                         theta_kp, theta_ki, theta_kd);
        angular_vel = ang_dist_vel + ang_theta_vel;
        // 更新上一次角度误差
        last_ang_dist_up = ang_dist_up;
        last_ang_theta_up = ang_theta_up;
        // 更新上一次角度命令
        last_linear_cmd = linear_vel;
        last_angular_cmd = angular_vel;

        file << "Angular Distant UP is: " << ang_dist_up << std::endl;
        file << "Angular Theta UP is: " << ang_theta_up << std::endl;
        file << "Angular Dist Vel is: " << ang_dist_vel << std::endl;
        file << "Angular Theta Vel is: " << ang_theta_vel << std::endl;
        file << "Linear Velocity is: " << linear_vel << std::endl;
        file << "Angular Velocity is: " << angular_vel << std::endl;
    }
    // 匀速阶段
    else if (SPEEDCONST == m_chassis_status){
        double dist_kp = 1.5;
        double dist_ki = 0;
        double dist_kd = 0;
        double theta_kp = 0.6;
        double theta_ki = 0;
        double theta_kd = 0;

        static double ang_dist_integ_ave = 0.0;
        static double ang_theta_integ_ave = 0.0;
        static double last_ang_dist_ave = 0.0;
        static double last_ang_theta_ave = 0.0;


        // 直线速度
        linear_vel = last_linear_cmd;

        // 角速度
        double ang_dist_ave = asin((dest_pose_y-pose_y)/head_dist);
        double ang_theta_ave = dest_pose_theta - pose_theta;
        // 积分误差
        ang_dist_integ_ave += ang_dist_ave;
        ang_theta_integ_ave += ang_theta_ave;
        // 角度PID
        double ang_dist_vel = AngVelPID(ang_dist_ave, ang_dist_integ_ave, last_ang_dist_ave,
                                        dist_kp, dist_ki, dist_kd);
        double ang_theta_vel = AngVelPID(ang_theta_ave, ang_theta_integ_ave, last_ang_theta_ave,
                                         theta_kp, theta_ki, theta_kd);
        angular_vel = ang_dist_vel + ang_theta_vel;
        // 更新上一次角度误差
        last_ang_dist_ave = ang_dist_ave;
        last_ang_theta_ave = ang_theta_ave;
        // 更新上一次角度命令
        last_linear_cmd = linear_vel;
        last_angular_cmd = angular_vel;


        file << "Angular Distant AVE is: " << ang_dist_ave << std::endl;
        file << "Angular Theta AVE is: " << ang_theta_ave << std::endl;
        file << "Angular Dist Vel is: " << ang_dist_vel << std::endl;
        file << "Angular Theta Vel is: " << ang_theta_vel << std::endl;
        file << "Linear Velocity is: " << linear_vel << std::endl;
        file << "Angular Velocity is: " << angular_vel << std::endl;
    }
    // 减速阶段
    else if (SPEEDSLOW == m_chassis_status){
        double dist_kp = 1.5;
        double dist_ki = 0;
        double dist_kd = 0;
        double theta_kp = 0.6;
        double theta_ki = 0;
        double theta_kd = 0;

        static double ang_dist_integ_slow = 0.0;
        static double ang_theta_integ_slow = 0.0;
        static double last_ang_dist_slow = 0.0;
        static double last_ang_theta_slow = 0.0;


        // 直线速度
        // double end_dist = sqrt(pow((pose_x-dest_pose_x), 2) +
        //                       pow((pose_y-dest_pose_y), 2));
        double end_dist = abs(pose_x - dest_pose_x);
        double slow_kp = 3.2;
        // linear_vel = last_linear_cmd*(end_dist/m_accel_dist);
        linear_vel = slow_kp*m_head_flag*end_dist;

        // 角速度
        double ang_dist_slow = asin((dest_pose_y-pose_y)/head_dist);
        double ang_theta_slow = dest_pose_theta - pose_theta;
        // 积分误差
        ang_dist_integ_slow += ang_dist_slow;
        ang_theta_integ_slow += ang_theta_slow;
        // 角度PID
        double ang_dist_vel = AngVelPID(ang_dist_slow, ang_dist_integ_slow, last_ang_dist_slow,
                                        dist_kp, dist_ki, dist_kd);
        double ang_theta_vel = AngVelPID(ang_theta_slow, ang_theta_integ_slow, last_ang_theta_slow,
                                         theta_kp, theta_ki, theta_kd);
        angular_vel = ang_dist_vel + ang_theta_vel;
        // 更新上一次角度误差
        last_ang_dist_slow = ang_dist_slow;
        last_ang_theta_slow = ang_theta_slow;
        // 更新上一次角度命令
        last_linear_cmd = linear_vel;
        last_angular_cmd = angular_vel;


        file << "Angular Distant SLOW is: " << ang_dist_slow << std::endl;
        file << "Angular Theta SLOW is: " << ang_theta_slow << std::endl;
        file << "Angular Dist Vel is: " << ang_dist_vel << std::endl;
        file << "Angular Theta Vel is: " << ang_theta_vel << std::endl;
        file << "Linear Velocity is: " << linear_vel << std::endl;
        file << "Angular Velocity is: " << angular_vel << std::endl;
    }
    // 位置到位，调整姿态
    else if(ADJUST == m_chassis_status){
        double theta_kp = 0.6;
        double theta_ki = 0;
        double theta_kd = 0;

        static double ang_theta_integ_adj = 0.0;
        static double last_ang_theta_adj = 0.0;

        // 直线速度
        linear_vel = 0.0;

        // 角速度
        double ang_theta_adj = dest_pose_theta - pose_theta;
        // 积分误差
        ang_theta_integ_adj += ang_theta_adj;
        // 角度PID
        double ang_theta_vel = AngVelPID(ang_theta_adj, ang_theta_integ_adj, last_ang_theta_adj,
                                         theta_kp, theta_ki, theta_kd);
        angular_vel = ang_theta_vel;
        // 更新上一次角度误差
        last_ang_theta_adj = ang_theta_adj;

        // 调整姿态后位置发生改变，故不能这么干
        linear_vel = 0.0;
        angular_vel = 0.0;
        // 更新上一次角度命令
        last_linear_cmd = linear_vel;
        last_angular_cmd = angular_vel;


        file << "Angular Theta adj is: " << ang_theta_adj << std::endl;
        file << "Angular Theta Vel is: " << ang_theta_vel << std::endl;
        file << "Linear Velocity is: " << linear_vel << std::endl;
        file << "Angular Velocity is: " << angular_vel << std::endl;
    }
    // 完全到位
    else if (HOLDON == m_chassis_status){
        linear_vel = 0;
        angular_vel = 0;

        last_linear_cmd = 0;
        last_angular_cmd = 0;
    }
    // 错误状态
    else {
        linear_vel = 0;
        angular_vel = 0;

        last_linear_cmd = 0;
        last_angular_cmd = 0;
    }

    ROS_INFO("linear vel is: %f, angular vel is: %f\r\n", linear_vel, angular_vel);

//    linear_vel = 0;
//    angular_vel = 0;
    send_cmd.linear.x = linear_vel;
    send_cmd.angular.z = angular_vel;
    m_vel_puber.publish(send_cmd);

    file << std::endl;
}



double ChassisPlanner::AngVelPID(double err, double err_integral, double last_err,
                 double kp, double ki, double kd)
{
    double vel_plan = kp*err + ki*err_integral + kd*(err - last_err);

    return vel_plan;
}

void ChassisPlanner::PoseToMat(double pose[], mat3x3 &pose_mat){
    double pose_x = pose[0];
    double pose_y = pose[1];
    double pose_theta = pose[2];
    double sin_theta = sin(pose_theta);
    double cos_theta = cos(pose_theta);

    pose_mat(0,2) = pose_x;
    pose_mat(1,2) = pose_y;
    pose_mat(0,0) = cos_theta;
    pose_mat(0,1) = -sin_theta;
    pose_mat(1,0) = sin_theta;
    pose_mat(1,1) = cos_theta;
    pose_mat(2,0) = 0;
    pose_mat(2,1) = 0;
    pose_mat(2,2) = 1;
}

void ChassisPlanner::MatToPose(mat3x3 &pose_mat, double pose[]){
    double pose_x, pose_y, pose_theta;
    pose_x = pose_mat(0,2);
    pose_y = pose_mat(1,2);
    pose_theta = atan(pose_mat(1,0)/pose_mat(0,0));

    pose[0] = pose_x;
    pose[1] = pose_y;
    pose[2] = pose_theta;
}


}
