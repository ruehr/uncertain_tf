#include <ros/ros.h>
#include <uncertain_tf/UncertainTransformBroadcaster.h>

using namespace uncertain_tf;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_variance_publisher", ros::init_options::AnonymousName);

    std::cout << "USAGE: static_variance_publisher frame_id var_x var_y var_z var_rot_z var_rot_y var_rot_x period(milliseconds)" << std::endl;

    if (argc < 2)
        exit(0);

    ros::NodeHandle node;

    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    UncertainTransformBroadcaster utfb;

    StampedCovariance stc;
    stc.resize(6,6);
    stc.setZero();

    //set values
    for (int i = 0; i < std::min(argc - 2, 6); ++i)
    {
        stc(i,i) = atof(argv[i+2]);
    }

    stc.frame_id_ = argv[1];

    double period = 100;

    if (argc>8)
        period = atof(argv[8]);

    ros::Rate rate(1000 / period);

    std::cout << "frame_id: " << stc.frame_id_ << std::endl;

    std::cout << "Cov: " << std::endl << stc << std::endl;

    std::cout << "Period: " << period << std::endl;

    ros::Time start_time = ros::Time::now();

    while (node.ok())
    {
        stc.stamp_ = ros::Time::now();

        utfb.sendCovariance(stc);

        rate.sleep();
    }
    return 0;
};

