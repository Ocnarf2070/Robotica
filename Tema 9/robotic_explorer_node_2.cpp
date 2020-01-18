#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>


#include <stdlib.h>
#include <numeric>
#include <math.h>
#include <tf/tf.h>
#include <list>

float TurnAngle = 0;
bool rotating = 0;
geometry_msgs::Pose2D current_pose;

template < typename T>
int betterindex(const std::vector<T>  & vecOfElements, const T  & mean)
{
    std::vector <std::pair<T,int>> list;
    int d=0;
    std::pair<T,int> p;
    for(auto vec:vecOfElements){
        if(vec>=mean){
            p=std::make_pair(vec,d);
            //std::cout<<p.first<<" "<<p.second<<std::endl;
          list.push_back(p);
        }
        d++;
    }
    std::list <std::pair<T,int>> list2;
    std::pair<T,int> ant = list.at(0);
    T m=ant.first,idx=ant.second;

    for (int it=1; it < list.size(); it++){
        std::pair<T,int> ant = list.at(it-1);
        std::pair<T,int> act = list.at(it);
        if(ant.second==act.second-1){
            m=(m+act.first)/2;
            idx=(idx+act.second)/2;
        }else{
            list2.push_back(std::make_pair(m,(int)idx));
           m=act.first;idx=act.second;
        }
    }

    list2.push_back(std::make_pair(m,(int)idx));
    T min=std::numeric_limits<T>::max();
    idx=0;
    for (auto l:list2) {
        std::pair<T,int> elem = l;
        if(elem.first<min){
            min=elem.first;
            idx=elem.second;
        }
    }
    return idx;
   }

void turning(const float & angle, const float & AngleStep, const int & index, const int & size, const bool & wall){
    int centre = size/2;
    std::cout<<centre<<std::endl;
    if(!wall)
        ::TurnAngle = angle + AngleStep * index;
    else {
        if(index>=centre){
            ::TurnAngle = angle - AngleStep * index;
        }else {

            ::TurnAngle = - angle + AngleStep * index;
        }
    }


}
void rotate(){
    ::rotating=1;
}

void odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    // linear position
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    // quaternion to RPY conversion
    tf::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // angular position
    current_pose.theta = yaw;
}

template < typename T>
int index(const std::vector<T>  & vecOfElements, const T  & element)
{
    int idx=0;

    while (vecOfElements.at(idx)!=NULL && vecOfElements.at(idx)!=element) {
        idx++;
    }

    return idx;
}

void processScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    int n_ranges = msg->ranges.size();
    float nearest = *(std::min_element(msg->ranges.begin(),msg->ranges.end()));
    ROS_INFO("[robot_explorer] I've a total of %i measurements to process! Are you ready? Nearest=%.2f",n_ranges, nearest);
    std::vector<std::float_t> vec = msg->ranges;
    float max = *(std::max_element(msg->ranges.begin(),msg->ranges.end()));
    float media = std::accumulate(vec.begin(),vec.end(),0.0)/n_ranges;
    std::cout<<"Media: "<<media<<std::endl;
    int idx = betterindex(vec,10.f);
    int idx2 = index(vec,nearest);
    if(max>media&&!rotating)
        if (nearest>0.5f)
            turning(msg->angle_min,msg->angle_increment,idx,n_ranges,0);
        else
            turning(msg->angle_max,msg->angle_increment,idx2,n_ranges,1);
    else{
        rotate();
    }


}



int main(int argc, char **argv)
{

    // Initialize the ROS system and become a node.

    ros::init( argc , argv , "exploring") ;

    ROS_INFO("[robot_explorer] Robotic explorer node running and initialized! Let's have some fun!");

    ros::NodeHandle nh;

    // Create a subscriber object
    ros::Subscriber sub = nh.subscribe("/laser_scan", 1000, processScanCallback);
    ros::Subscriber sub_odom = nh.subscribe("odom", 1, odomCallback);
    // Create a publisher object
    ros::Publisher pub = nh.advertise <geometry_msgs::Twist>("/cmd_vel" , 1000) ;

    // Seed the random number generator.
    srand ( time (0) ) ;

    // Loop at 2Hz until the node is shut down.
    ros::Rate rate (2) ;
    ros::Time begin = ros::Time::now();

    while ( begin.toSec() == 0 )
        begin = ros::Time::now();

    double ellapsed_time = 0;

    while ( ( ros::ok () ) && ( ellapsed_time < 60*5 ) )
    {
        // Create and fill in the message. The other four
        // fields , which are ignored by stage, default to 0.
        double original = current_pose.theta;
        if (rotating){
            while(ros::ok() && current_pose.theta > (original-M_PI_2))
            {
                geometry_msgs::Twist move;
                //velocity controls
                move.linear.x = 0; //speed value m/s
                move.angular.z = -0.3;
                pub.publish(move);

                ros::spinOnce();
                rate.sleep();
            }
        } else {
            geometry_msgs::Twist msg2;
            msg2.linear.x = 1000;
            msg2.angular.z = ::TurnAngle;

            // Publish the message.
            pub.publish(msg2) ;
        }

        ::rotating=0;


        // Wait until it 's time for another iteration .
        rate.sleep () ;
        ros::spinOnce(); // give time to receive /base_scan messages

        ros::Time current = ros::Time::now();
        ellapsed_time = (current - begin).toSec();
        ROS_INFO("[robot_explorer] Ellpased time: %.2f", ellapsed_time );
    }

}
