#include <ros/ros.h>
#include <uncertain_tf/UncertainTransformBroadcaster.h>

using namespace uncertain_tf;
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "static_covariance_publisher", ros::init_options::AnonymousName);

    std::cout << "USAGE: static_covariance_publisher frame_id var_x  cov_xy cov_xz cov_xZ cov_xY cov_xX" << std::endl
              << "                                            cov_yx var_y  cov_yz cov_yZ cov_yY cov_yX" << std::endl
              << "                                            cov_zx cov_zy var_z  cov_zZ cov_zY cov_zX" << std::endl
              << "                                            cov_Zx cov_Zy cov_Zz var_Z  cov_ZY cov_ZX" << std::endl
              << "                                            cov_Yx cov_Yy cov_Yz cov_YZ var_Y  cov_YX" << std::endl
              << "                                            cov_Xx cov_Xy cov_Xz cov_XZ cov_XY var_X   period(milliseconds)" << std::endl
              << " (36 var/cov values in total + period)" << std::endl;

    if (argc < 6*6+1)
        exit(0);

    ros::NodeHandle node;

    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    UncertainTransformBroadcaster utfb;

    StampedCovariance stc;
    stc.resize(6,6);
    stc.setZero();

    //set values
    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 6; ++j)
        {
            stc(i,j) = atof(argv[2+i+j * 6]);
        }

    stc.frame_id_ = argv[1];

    double period = 100;

    if (argc>6*6+2)
        period = atof(argv[6*6+2]);

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

