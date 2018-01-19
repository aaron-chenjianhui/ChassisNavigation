#include "laser_detect.h"


namespace lms{
LaserDetect::LaserDetect(){
    //
    ParamInit();
    //
    m_suber = m_nh.subscribe("scan", 1000, &LaserDetect::LaserDetectCallback, this);
    m_puber = m_nh.advertise<sensor_msgs::PointCloud>("point", 1000);
    //
    m_srver = m_nh.advertiseService("global_calib", &LaserDetect::LaserSrvCallback, this);

    // For checkout error
    m_pose_puber = m_nh.advertise<geometry_msgs::Pose2D>("ori_pose", 1000);
    m_filter_lidar_puber = m_nh.advertise<sensor_msgs::LaserScan>("custom_scan", 1000);
}

LaserDetect::~LaserDetect(){

}

void LaserDetect::ParamInit(){
    // Boolean
    detect_flag = 0;

    // Unit: m
    m_lidar_max = 10;
    m_lidar_min = 0.8;
    m_lidar_narr_max = 2;
    m_lidar_narr_min = 0.8;

    // Unit: mm
    m_front_dist = 1000;
    m_front_thre = m_front_dist*1.2;

    //
    lidar_filter_in.erase(lidar_filter_in.begin(), lidar_filter_in.end());
    lidar_filter_out.erase(lidar_filter_out.begin(), lidar_filter_out.end());

    double coeff_Az[] = {1, -2.999548768734757, 2.999097639268598, -0.999548870522360};
    double coeff_Bz[] = {8.673617379884036e-20, 5.741326415393087e-12, 5.740463101634766e-12, 0};
    if (sizeof(coeff_Az) == sizeof(coeff_Bz)){
        int num = sizeof(coeff_Az)/sizeof(double);
        m_filter_order = num - 1;

        Az.insert(Az.begin(), coeff_Az, coeff_Az+num);
        Bz.insert(Bz.begin(), coeff_Bz, coeff_Bz+num);
    }
    else{
        ROS_INFO("Initialization Error!\r\n");
    }

    m_ave_num = 250;
}

bool LaserDetect::LaserSrvCallback(laser_detect::laser_detect_srv::Request &req,
                                   laser_detect::laser_detect_srv::Response &res){
    // Ask for Laser Detect
    bool ask_for_calib = false;
    ask_for_calib = req.flag;

    if (true == ask_for_calib){
       while(1){}
        // sleep(2);
        //
        boost::mutex::scoped_lock lock(m_mutex);

        vec_data_type path_param;
        vec_data_type ori_pose;
        vec_data_type l_line_param, r_line_param, f_line_param;

        //
        if (WallDetect(lidar_data_filter, l_line_param, r_line_param, f_line_param)){
            DataProcess(l_line_param, r_line_param, f_line_param, ori_pose);

            res.pose.x = ori_pose[0];
            res.pose.y = ori_pose[1];
            res.pose.theta = ori_pose[2];

            tf::Transform transform;
            transform.setOrigin(tf::Vector3(ori_pose[0]/1000.0, ori_pose[1]/1000.0, 0.0));
            tf::Quaternion q;
            q.setRPY(0, 0, ori_pose[2]);
            transform.setRotation(q);
            m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "laser", "original"));

            return true;
        }
        else{
            return false;
        }

    }
    else{
        return false;
    }

}

void LaserDetect::LaserDetectCallback(const sensor_msgs::LaserScan &laser_data){
    //
    boost::mutex::scoped_lock lock(m_mutex);

    lidar_data_raw.clear();
    lidar_data_filter.clear();

    unsigned int seq = laser_data.header.seq;
    double angle_min = laser_data.angle_min;
    double angle_max = laser_data.angle_max;
    double angle_increment = laser_data.angle_increment;
    double time_increment = laser_data.time_increment;
    double scan_time = laser_data.scan_time;
    double range_min = laser_data.range_min;
    double range_max = laser_data.range_max;
    std::vector<float> ranges = laser_data.ranges;
    std::vector<float> intensities = laser_data.intensities;

    //
    // lidar_iter_type iter;
    lidar_ang.erase(lidar_ang.begin(), lidar_ang.end());
    std::vector<float>::iterator iter;
    double angle_now = angle_min;
    double lidar_data_now;
    for (iter = ranges.begin(); iter != ranges.end(); ++iter){
        lidar_data_now = (double)(*iter);
        lidar_data_raw.insert(std::pair<double, double>(angle_now, lidar_data_now));
        lidar_ang.push_back(angle_now);

        angle_now += angle_increment;
    }

//    // Put lidar_data_raw into filter deque
//    lidar_filter_in.push_back(lidar_data_raw);
//    if ((m_filter_order+1) < lidar_filter_in.size()){
//        lidar_filter_in.pop_front();
//    }

//    BWFilter(lidar_filter_in, lidar_filter_out, Az, Bz);
//    lidar_data_filter = lidar_filter_out.back();


    // Put lidar_data_raw into filter deque
    lidar_filter_in.push_back(lidar_data_raw);
    if (m_ave_num < lidar_filter_in.size()){
        lidar_filter_in.pop_front();
    }
    MAFilter(lidar_filter_in, lidar_data_filter, m_ave_num);

    // For Display
    lidar_data_type lidar_data_disp = lidar_data_filter;
    vec_data_type path_param;
    vec_data_type ori_pose;
    vec_data_type l_line_param, r_line_param, f_line_param;

    //
    sensor_msgs::LaserScan custom_msgs;
    std::vector<float> custom_data;
    lidar_iter_type custom_iter;
    for (custom_iter = lidar_data_disp.begin(); custom_iter != lidar_data_disp.end(); ++custom_iter){
        custom_data.push_back((float)(custom_iter->second));
    }
    custom_msgs.ranges = custom_data;
    m_filter_lidar_puber.publish(custom_msgs);


    if (WallDetect(lidar_data_disp, l_line_param, r_line_param, f_line_param)){
        DataProcess(l_line_param, r_line_param, f_line_param, ori_pose);
        ResultDisp(lidar_data_disp, ori_pose, l_line_param, r_line_param, f_line_param);

        //
        geometry_msgs::Pose2D ori_pose_msg;
        ori_pose_msg.x = ori_pose[0];
        ori_pose_msg.y = ori_pose[1];
        ori_pose_msg.theta = ori_pose[2];
        m_pose_puber.publish(ori_pose_msg);

        // Broadcast tf tree
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(ori_pose[0]/1000.0, ori_pose[1]/1000.0, 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, ori_pose[2]);
        transform.setRotation(q);
        m_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "laser", "test"));
    }
    else{
        ROS_INFO("Detect Error\r\n");
    }

}

void LaserDetect::BWFilter(lidar_filter_type &filter_data_in, lidar_filter_type& filter_data_out,
                                                        vec_data_type &Az, vec_data_type &Bz){
    if (sizeof(Az) == sizeof(Bz) && (m_filter_order+1) == filter_data_in.size()){
        lidar_data_type lidar_data_now;
        lidar_iter_type lidar_iter;

        //
        filter_riter_type filter_in_riter;
        filter_riter_type filter_out_riter;
        vec_iter_type az_iter;
        vec_iter_type bz_iter;

        double angle_now;
        double data_now;
        // Loop for every angle
        lidar_iter_type iter;
        int angle_index = 0;
        for (iter = filter_data_in.back().begin(); iter != filter_data_in.back().end(); ++iter, ++angle_index){
            double in_sum = 0;
            double out_sum = 0;

            // Sum of filter_data_in
            filter_in_riter = filter_data_in.rbegin();
            bz_iter = Bz.begin();
            for (; bz_iter != Bz.end(); ++bz_iter, ++filter_in_riter){
                lidar_iter = (*filter_in_riter).begin();
                for (int i = 0; i < angle_index; ++i, ++lidar_iter)

                in_sum += (*bz_iter)*(lidar_iter->second);
            }

            // Sum of filter_data_out
            filter_out_riter = filter_data_out.rbegin();
            az_iter = Az.begin() + 1;
            for (; az_iter != Az.end(); ++az_iter, ++filter_out_riter){
                lidar_iter = (*filter_out_riter).begin();
                for (int i = 0; i < angle_index; ++i, ++lidar_iter)

                out_sum += (*az_iter)*(lidar_iter->second);
            }

            //
            angle_now = iter->first;
            data_now = (in_sum - out_sum)/Az[0];

            lidar_data_now.insert(std::pair<double, double>(angle_now, data_now));
        }

        // Push lidar_data_now into filter_data_out deque
        filter_data_out.push_back(lidar_data_now);
        if ((m_filter_order+1) < filter_data_out.size()){
            filter_data_out.pop_front();
        }
    }
    // Mean Value filter for first m_filter_order+1 data
    else if (sizeof(Az) == sizeof(Bz) && m_filter_order >= filter_data_in.size()){
        lidar_data_type lidar_data_now;
        lidar_iter_type iter;

        int filter_num = filter_data_in.size();

        //
        vec_data_type filter_data_sum(filter_data_in.back().size());
        vec_iter_type sum_iter;

        filter_iter_type filter_iter;
        for (filter_iter = filter_data_in.begin(); filter_iter != filter_data_in.end(); ++filter_iter){
            vec_data_type filter_data_select;
            vec_iter_type select_iter;

            SelectLidarData(*filter_iter, filter_data_select);

            sum_iter = filter_data_sum.begin();
            for (select_iter = filter_data_select.begin(); select_iter != filter_data_select.end(); ++select_iter){
                *sum_iter += *select_iter;
                ++sum_iter;
            }
        }

        sum_iter = filter_data_sum.begin();
        double angle_now;
        double data_now;
        for (iter = filter_data_in.back().begin(); iter != filter_data_in.back().end(); ++iter){
            angle_now = iter->first;
            data_now = (*sum_iter)/filter_num;

            lidar_data_now.insert(std::pair<double, double>(angle_now, data_now));

            ++sum_iter;
        }

        // Push lidar_data_now into filter_data_out deque
        filter_data_out.push_back(lidar_data_now);
        if ((m_filter_order+1) < filter_data_out.size()){
            filter_data_out.pop_front();
        }
    }
    else{

    }
}

void LaserDetect::MAFilter(lidar_filter_type &filter_data_in, lidar_data_type &filter_data_out, int ave_num){
    filter_data_out.erase(filter_data_out.begin(), filter_data_out.end());
    if (ave_num >= filter_data_in.size()){
        //
        vec_data_type lidar_sum;
        lidar_sum.resize(lidar_ang.size());
        vec_iter_type sum_iter;

        // Loop for every angle
        filter_iter_type num_iter;
        lidar_iter_type ang_iter;
        for (num_iter = filter_data_in.begin(); num_iter != filter_data_in.end(); ++num_iter){
            sum_iter = lidar_sum.begin();
            for (ang_iter = (*num_iter).begin(); ang_iter != (*num_iter).end(); ++ang_iter, ++sum_iter){
                (*sum_iter) += ang_iter->second;
            }
        }

        //
        double angle_now;
        double data_now;
        vec_iter_type iter = lidar_ang.begin();
        for (sum_iter = lidar_sum.begin(); sum_iter != lidar_sum.end(); ++sum_iter, ++iter){
            angle_now = *iter;
            data_now = (*sum_iter)/(filter_data_in.size());

            filter_data_out.insert(std::pair<double, double>(angle_now, data_now));
        }
    }
    else{

    }
}

void LaserDetect::DataSelect(double min_angle, double max_angle, lidar_data_type& lidar_data_in,
                             lidar_data_type& lidar_data_out){
    if (min_angle < max_angle){
        lidar_iter_type iter_low;
        lidar_iter_type iter_up;

        lidar_data_out.clear();

        iter_low = lidar_data_in.lower_bound(min_angle);
        iter_up = lidar_data_in.upper_bound(max_angle);

        // if (iter_low < iter_up){
        lidar_data_out.insert(iter_low, iter_up);
        // }
    }
    else{
        ROS_ERROR("Angle Range Wrong!\r\n");
    }
}

// Ignore Unreasonable Data
void LaserDetect::DataFilter(lidar_data_type& lidar_data_in, lidar_data_type& lidar_data_out){
    lidar_iter_type iter;
    lidar_data_out.clear();

    for (iter = lidar_data_in.begin(); iter != lidar_data_in.end(); ++iter){
        if ((iter->first > DEG2RAD(90)) || (iter->first < DEG2RAD(-90))){
            if ((iter->second > m_lidar_narr_min) && (iter->second < m_lidar_narr_max)){
                lidar_data_out.insert(*iter);
            }
        }
        else{
            if ((iter->second > m_lidar_min) || (iter->second < m_lidar_max)){
                lidar_data_out.insert(*iter);
            }
        }
    }
}

//
void LaserDetect::DataIgn(vec_data_type& ign_angle, lidar_data_type& lidar_data_in,
                          lidar_data_type& lidar_data_out){
    vec_iter_type max_iter;
    vec_iter_type min_iter;
    vec_iter_type vec_iter_now;
    lidar_iter_type iter;

    bool ign_flag = 0;

    lidar_data_out.clear();
    vec_iter_now = ign_angle.begin();

    min_iter = std::min_element(ign_angle.begin(), ign_angle.end());
    max_iter = std::max_element(ign_angle.begin(), ign_angle.end());

    for (iter = lidar_data_in.begin(); iter != lidar_data_in.end(); ++iter){
        if (iter->first > (*max_iter) || iter->first < (*min_iter)){
            lidar_data_out.insert(*iter);
        }
        else{
            vec_iter_type vec_iter;
            for (vec_iter = vec_iter_now;vec_iter != ign_angle.end(); ++iter){
                if (abs(*vec_iter-iter->first) < 0.001){
                    ign_flag = 1;
                    break;
                }
            }

            if (0 == ign_flag){
                lidar_data_out.insert(*iter);
            }
            else{
                ++vec_iter_now;
            }
            ign_flag = 0;
        }
    }
}

//
bool LaserDetect::ResultCheck(vec_data_type& left_line_param, vec_data_type& right_line_param, vec_data_type& front_line_param){
    if (left_line_param.empty() || right_line_param.empty() || front_line_param.empty()){
        return false;
    }

    // Final Check
    double r_n_x = right_line_param[0];
    double r_n_y = right_line_param[1];
    double r_a_x = right_line_param[2];
    double r_a_y = right_line_param[3];
    double l_n_x = left_line_param[0];
    double l_n_y = left_line_param[1];
    double l_a_x = left_line_param[2];
    double l_a_y = left_line_param[3];
    double f_n_x = front_line_param[0];
    double f_n_y = front_line_param[1];
    double f_a_x = front_line_param[2];
    double f_a_y = front_line_param[3];

    // Left & Right Check
    bool l_r_flag = (fabs(r_n_x*l_n_y-r_n_y*l_n_x) < 0.05) & (l_a_y > r_a_y);

    // Left
    bool l_f_flag = fabs(f_n_x*l_n_x+f_n_y*l_n_y) < 0.1;

    // Right
    bool r_f_flag = fabs(r_n_x*f_n_x+r_n_y*f_n_y) < 0.1;

    return (l_r_flag & l_f_flag &r_f_flag);
}

void LaserDetect::DataProcess(vec_data_type& l_line_param, vec_data_type& r_line_param, vec_data_type& f_line_param, vec_data_type& ori_pose){
    double r_n_x, r_n_y, r_a_x, r_a_y;
    double l_n_x, l_n_y, l_a_x, l_a_y;
    double f_n_x, f_n_y, f_a_x, f_a_y;
    double r_theta, l_theta, ori_theta;

    //
    mat2x2 ori_A;
    mat2x1 ori_B;
    mat2x1 ori_pos;             // original: (X, Y)

    r_n_x = r_line_param[0];
    r_n_y = r_line_param[1];
    r_a_x = r_line_param[2];
    r_a_y = r_line_param[3];
    l_n_x = l_line_param[0];
    l_n_y = l_line_param[1];
    l_a_x = l_line_param[2];
    l_a_y = l_line_param[3];
    f_n_x = f_line_param[0];
    f_n_y = f_line_param[1];
    f_a_x = f_line_param[2];
    f_a_y = f_line_param[3];

    // Original Position
    ori_A(0, 0) = f_n_x;
    ori_A(0, 1) = f_n_y;
    ori_A(1, 0) = l_n_x;
    ori_A(1, 1) = l_n_y;
    ori_B(0, 0) = f_n_x*f_a_x + f_n_y*f_a_y;
    ori_B(1, 0) = l_n_x*l_a_x + l_n_y*l_a_y;
    ori_pos = ori_A.inverse()*ori_B;
    ori_pose.push_back(ori_pos(0, 0));
    ori_pose.push_back(ori_pos(1, 0));

    // Original Orientation
    r_theta = -atan(r_n_x/r_n_y);
    l_theta = -atan(l_n_x/l_n_y);
    ori_theta = (r_theta + l_theta)/2;
    ori_pose. push_back(ori_theta);
}

// path_param: (nx, ny, ax, ay);        dest_pose: (dest_x, dest_y, dest_theta)
bool LaserDetect::WallDetect(lidar_data_type& lidar_data, vec_data_type& l_param,
                             vec_data_type& r_param, vec_data_type& f_param){
    // Lidar Data
    lidar_data_type lidar_filter;
    lidar_data_type lidar_left;
    lidar_data_type lidar_right;
    lidar_data_type lidar_front;
    lidar_data_type lidar_judge;


    vec_data_type judge_data;

    // Lidar Data for final Ransac
    lidar_data_type front_select_lidar;
    lidar_data_type left_select_lidar;
    lidar_data_type right_select_lidar;

    // Left, Right, Front Range
    double left_min = DEG2RAD(0);
    double left_max = DEG2RAD(134);
    double right_min = DEG2RAD(-134);
    double right_max = DEG2RAD( 0);
    double front_min = DEG2RAD(-90);
    double front_max = DEG2RAD(90);
    double judge_min = DEG2RAD(-45);
    double judge_max = DEG2RAD(45);
    // First Ransac Angle
    double r_angle_min, r_angle_max;
    double l_angle_min, l_angle_max;
    double f_angle_min, f_angle_max;
    // Final Ransac  Angle
    double r_fin_angle_min, r_fin_angle_max;
    double l_fin_angle_min, l_fin_angle_max;
    double f_fin_angle_min, f_fin_angle_max;

    // RANSAC Data
    std::vector<double> right_line_param;
    std::vector<double> left_line_param;
    std::vector<double> front_line_param;
    std::vector<Point2D> right_points;
    std::vector<Point2D> left_points;
    std::vector<Point2D> front_points;

    //
    std::vector<bool> r_vote_index;
    std::vector<bool> l_vote_index;
    std::vector<bool> f_vote_index;
    std::vector<bool>::iterator r_vote_iter;
    std::vector<bool>::iterator l_vote_iter;
    std::vector<bool>::iterator f_vote_iter;

    vec_data_type r_vote_angle;
    vec_data_type l_vote_angle;
    vec_data_type f_vote_angle;

    // Final Parameter
    double r_n_x, r_n_y, r_a_x, r_a_y;
    double l_n_x, l_n_y, l_a_x, l_a_y;
    double f_n_x, f_n_y, f_a_x, f_a_y;

    // 0:left, right    1:left(right), front    2:front    3:error
    unsigned int chassis_pos_mode = 0;

    // RANSAC Parameters
    LineParamEstimator lpEstimator(40); //for a point to be on the line it has to be closer than 0.5 units from the line
    double desiredProbabilityForNoOutliers = 0.999;

    // Data Pre-Processing
    DataFilter(lidar_data, lidar_filter);

    DataSelect(left_min, left_max, lidar_filter, lidar_left);
    DataSelect(right_min, right_max, lidar_filter, lidar_right);
    DataSelect(front_min, front_max, lidar_filter, lidar_front);
    DataSelect(judge_min, judge_max, lidar_filter, lidar_judge);

    ScanToPoint(lidar_right, right_points);
    ScanToPoint(lidar_left, left_points);
    ScanToPoint(lidar_front, front_points);


    // Position Judgemen
    SelectLidarData(lidar_judge, judge_data);
    double judge_sum = std::accumulate(judge_data.begin(), judge_data.end(), 0);
    double judge_mean = judge_sum/(judge_data.end() - judge_data.begin())*1000;
    if (judge_mean < m_front_thre){
        chassis_pos_mode = 2;
    }


    // Find Lines
    if (2 == chassis_pos_mode){
        double f_usedData = RANSAC<Point2D,double>::compute(front_line_param,
                                                            &lpEstimator ,
                                                            front_points,
                                                            desiredProbabilityForNoOutliers,
                                                            f_vote_index);
        // Voted Angle
        lidar_iter_type iter;
        f_vote_iter = f_vote_index.begin();
        for (iter = lidar_front.begin(); iter != lidar_front.end(); ++iter){
            if (true == *f_vote_iter){
                f_vote_angle.push_back(iter->first);
            }
            ++f_vote_iter;
        }
        f_angle_max = *(std::max_element(f_vote_angle.begin(), f_vote_angle.end()));
        f_angle_min = *(std::min_element(f_vote_angle.begin(), f_vote_angle.end()));

        // Final left  & right Angle
        l_fin_angle_min = f_angle_max;
        l_fin_angle_max = left_max;
        r_fin_angle_min = right_min;
        r_fin_angle_max = f_angle_min;

        DataSelect(l_fin_angle_min, l_fin_angle_max, lidar_filter, left_select_lidar);
        DataSelect(r_fin_angle_min, r_fin_angle_max, lidar_filter, right_select_lidar);

        ScanToPoint(left_select_lidar, left_points);
        ScanToPoint(right_select_lidar, right_points);

        double r_usedData = RANSAC<Point2D,double>::compute(right_line_param,
                                                            &lpEstimator ,
                                                            right_points,
                                                            desiredProbabilityForNoOutliers,
                                                            r_vote_index);
        double l_usedData = RANSAC<Point2D,double>::compute(left_line_param,
                                                            &lpEstimator ,
                                                            left_points,
                                                            desiredProbabilityForNoOutliers,
                                                            l_vote_index);


    }
    else{
        double r_usedData = RANSAC<Point2D,double>::compute(right_line_param,
                                                            &lpEstimator ,
                                                            right_points,
                                                            desiredProbabilityForNoOutliers,
                                                            r_vote_index);
        double l_usedData = RANSAC<Point2D,double>::compute(left_line_param,
                                                            &lpEstimator ,
                                                            left_points,
                                                            desiredProbabilityForNoOutliers,
                                                            l_vote_index);
        // Voted Angle
        lidar_iter_type iter;
        r_vote_iter = r_vote_index.begin();
        for (iter = lidar_right.begin(); iter != lidar_right.end(); ++iter){
            if (true == *r_vote_iter){
                r_vote_angle.push_back(iter->first);
            }
            ++r_vote_iter;
        }
        r_angle_max = *(std::max_element(r_vote_angle.begin(), r_vote_angle.end()));
        r_angle_min = *(std::min_element(r_vote_angle.begin(), r_vote_angle.end()));

        l_vote_iter = l_vote_index.begin();
        for (iter = lidar_left.begin(); iter != lidar_left.end(); ++iter){
            if (true == *l_vote_iter){
                l_vote_angle.push_back(iter->first);
            }
            ++l_vote_iter;
        }
        l_angle_max = *(std::max_element(l_vote_angle.begin(), l_vote_angle.end()));
        l_angle_min = *(std::min_element(l_vote_angle.begin(), l_vote_angle.end()));
        // Line Parameter
        r_n_x = right_line_param[0];
        r_n_y = right_line_param[1];
        r_a_x = right_line_param[2];
        r_a_y = right_line_param[3];
        l_n_x = left_line_param[0];
        l_n_y = left_line_param[1];
        l_a_x = left_line_param[2];
        l_a_y = left_line_param[3];

        // Mode judgement
        if (fabs(r_n_x*l_n_y-r_n_y*l_n_x) < 0.05 && l_a_y > r_a_y){
            chassis_pos_mode = 0;
        }
        else if (fabs(r_n_x*l_n_x+r_n_y*l_n_y) < 0.1){
            double l_angle_pos = fabs((l_angle_min + l_angle_max)/2.0 - left_min)/(left_max - left_min);
            double r_angle_pos = fabs((r_angle_min + r_angle_max)/2.0 - right_min)/(right_max - right_min);

            if (l_angle_pos < r_angle_pos){
                chassis_pos_mode = 11;         // front, right
            }
            else{
                chassis_pos_mode = 12;         // left, front
            }
        }
        else{
            chassis_pos_mode = 3;
        }

        // Run Ransac
        if (0 == chassis_pos_mode){
            f_fin_angle_min = r_angle_max;
            f_fin_angle_max = l_angle_min;
            DataSelect(f_fin_angle_min, f_fin_angle_max, lidar_filter, front_select_lidar);

            //
            ScanToPoint(front_select_lidar, front_points);

            double f_usedData = RANSAC<Point2D,double>::compute(front_line_param,
                                                                &lpEstimator ,
                                                                front_points,
                                                                desiredProbabilityForNoOutliers,
                                                                f_vote_index);
        }
        else if (11 == chassis_pos_mode){
            left_line_param.clear();
            l_vote_index.clear();

            f_fin_angle_min = r_angle_max;
            f_fin_angle_max = l_angle_max;
            l_fin_angle_min = l_angle_max;
            l_fin_angle_max = left_max;

            DataSelect(f_fin_angle_min, f_fin_angle_max, lidar_filter, front_select_lidar);
            DataSelect(l_fin_angle_min, l_fin_angle_max, lidar_filter, left_select_lidar);

            ScanToPoint(front_select_lidar, front_points);
            ScanToPoint(left_select_lidar, left_points);

            double f_usedData = RANSAC<Point2D,double>::compute(front_line_param,
                                                                &lpEstimator ,
                                                                front_points,
                                                                desiredProbabilityForNoOutliers,
                                                                f_vote_index);
            double l_usedData = RANSAC<Point2D,double>::compute(left_line_param,
                                                                &lpEstimator ,
                                                                left_points,
                                                                desiredProbabilityForNoOutliers,
                                                                l_vote_index);
        }
        else if (12 == chassis_pos_mode){
            right_line_param.clear();
            r_vote_index.clear();

            f_fin_angle_min = r_angle_min;
            f_fin_angle_max = l_angle_min;
            r_fin_angle_min = r_angle_min;
            r_fin_angle_max = right_min;

            DataSelect(f_fin_angle_min, f_fin_angle_max, lidar_filter, front_select_lidar);
            DataSelect(r_fin_angle_min, r_fin_angle_max, lidar_filter, right_select_lidar);

            ScanToPoint(front_select_lidar, front_points);
            ScanToPoint(right_select_lidar, right_points);

            double f_usedData = RANSAC<Point2D,double>::compute(front_line_param,
                                                                &lpEstimator ,
                                                                front_points,
                                                                desiredProbabilityForNoOutliers,
                                                                f_vote_index);
            double r_usedData = RANSAC<Point2D,double>::compute(right_line_param,
                                                                &lpEstimator ,
                                                                right_points,
                                                                desiredProbabilityForNoOutliers,
                                                                r_vote_index);
        }
        else{
            f_fin_angle_min = r_angle_max;
            f_fin_angle_max = l_angle_min;
            DataSelect(f_fin_angle_min, f_fin_angle_max, lidar_filter, front_select_lidar);

            //
            ScanToPoint(front_select_lidar, front_points);

            double f_usedData = RANSAC<Point2D,double>::compute(front_line_param,
                                                                &lpEstimator ,
                                                                front_points,
                                                                desiredProbabilityForNoOutliers,
                                                                f_vote_index);
        }


    }


    // Result Checking
    bool re = ResultCheck(left_line_param, right_line_param, front_line_param);
    if (true == re){
        l_param = left_line_param;
        r_param = right_line_param;
        f_param = front_line_param;

        return true;
    }
    else{
        ROS_INFO("ERROR\r\n");
        return false;
    }
}

void LaserDetect::ResultDisp(lidar_data_type& lidar_data, vec_data_type& ori_pose, vec_data_type &left_line_param,
                             vec_data_type &right_line_param, vec_data_type &front_line_param){
    // Display Results
    sensor_msgs::PointCloud cloud;
    geometry_msgs::Point32 single_point;
    std::vector<float> intensity_value;
    std::vector<geometry_msgs::Point32> points_value;

    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "laser";
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";


    //    lidar_data_type data_disp = front_select_lidar;
    lidar_data_type data_disp = lidar_data;
    lidar_iter_type iter;
    for (iter = data_disp.begin(); iter != data_disp.end(); ++iter){
        single_point.x = (iter->second)*cos(iter->first);
        single_point.y = (iter->second)*sin(iter->first);
        single_point.z = 0;
        points_value.push_back(single_point);

        intensity_value.push_back(500);
    }


    //
    double n_x, n_y, a_x, a_y;
    double start_x, start_y, end_x, end_y, incre_x, incre_y;

    int num = 500;

    // Front
    n_x = front_line_param[0];
    n_y = front_line_param[1];
    a_x = front_line_param[2]/1000.0;
    a_y = front_line_param[3]/1000.0;

    start_x = a_x + 4*n_y;
    start_y = a_y - 4*n_x;
    end_x = a_x - 4*n_y;
    end_y = a_y + 4*n_x;
    incre_x = (end_x - start_x)/(double)num;
    incre_y = (end_y - start_y)/(double)num;

    for (int i = 0; i < num; ++i){
        single_point.x = start_x + incre_x*i;
        single_point.y = start_y + incre_y*i;
        single_point.z = 0;

        points_value.push_back(single_point);
        intensity_value.push_back(100);
    }

    // Right
    n_x = right_line_param[0];
    n_y = right_line_param[1];
    a_x = right_line_param[2]/1000.0;
    a_y = right_line_param[3]/1000.0;

    start_x = a_x + 4*n_y;
    start_y = a_y - 4*n_x;
    end_x = a_x - 4*n_y;
    end_y = a_y + 4*n_x;
    incre_x = (end_x - start_x)/(double)num;
    incre_y = (end_y - start_y)/(double)num;

    for (int i = 0; i < num; ++i){
        single_point.x = start_x + incre_x*i;
        single_point.y = start_y + incre_y*i;
        single_point.z = 0;

        points_value.push_back(single_point);
        intensity_value.push_back(200);
    }

    // Left
    n_x = left_line_param[0];
    n_y = left_line_param[1];
    a_x = left_line_param[2]/1000.0;
    a_y = left_line_param[3]/1000.0;

    start_x = a_x + 4*n_y;
    start_y = a_y - 4*n_x;
    end_x = a_x - 4*n_y;
    end_y = a_y + 4*n_x;
    incre_x = (end_x - start_x)/(double)num;
    incre_y = (end_y - start_y)/(double)num;

    for (int i = 0; i < num; ++i){
        single_point.x = start_x + incre_x*i;
        single_point.y = start_y + incre_y*i;
        single_point.z = 0;

        points_value.push_back(single_point);
        intensity_value.push_back(300);
    }

    cloud.points = points_value;
    cloud.channels[0].values = intensity_value;

    m_puber.publish(cloud);
}

}
