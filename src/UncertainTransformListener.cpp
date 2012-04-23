
#include "uncertain_tf/UncertainTransformListener.h"


using namespace Eigen;
using namespace tf;
using namespace uncertain_tf;
using namespace std;

namespace uncertain_tf {




/*
void UncertainTransformListener::calculateSampleCovariance(const MatrixXd& x, const MatrixXd& y, MatrixXd & C_)
{
    //typedef typename Derived::Scalar Scalar;
    //typedef typename internal::plain_row_type<Derived>::type RowVectorType;

    double num_observations = x.rows();

    VectorXd x_mean = x.colwise().sum() / num_observations;
    VectorXd y_mean = y.colwise().sum() / num_observations;


    MatrixXd& C = const_cast< MatrixXd& >(C_);

    C.derived().resize(x.cols(),x.cols()); // resize the derived object
    C = (x.rowwise() - x_mean).transpose() * (y.rowwise() - y_mean) / num_observations;

    //C.derived().resize(x.cols(),x.cols()); // resize the derived object
    //C = (x.rowwise() - x_mean).transpose() * (y.rowwise() - y_mean) / num_observations;
}
*/

VectorXd UncertainTransformListener::calculateSampleMean(const MatrixXd &x)
{
    return x.colwise().sum() / x.rows();
}

bool UncertainTransformListener::isZero(const MatrixXd mat)
{
    for (int i = 0; i < mat.cols(); ++i)
        for (int j = 0; j < mat.cols(); ++j)
            if (fabs(mat(i,j)) > 0) //.0000000000000001)
                return false;
    return true;
}

MatrixXd UncertainTransformListener::sampleSetTFtoMatrixXd(std::vector<tf::StampedTransform> sampleset)
{
    MatrixXd ret(sampleset.size(),6);
    for (size_t n=0; n < sampleset.size(); ++n)
    {
        ret.row(n) = transformTFToVectorXd(sampleset[n]);
    }
    return ret;
}

// sample n points from the given distribution
MatrixXd UncertainTransformListener::sampleFromMeanCov(const VectorXd &mean_, const MatrixXd &cov_, size_t n)
{
    //cout << "SAMPLING" << endl << endl << "Mean" << endl << mean_ << endl << "Cov" << endl << cov_ << endl;

    //EigenMultivariateNormal<double, 6> emn(mean_.transpose(),cov_);
    emn_->setMean(mean_.transpose());
    emn_->setCovar(cov_);

    Matrix<double,6,1>  nextSample;
    MatrixXd ret(mean_.rows(),n);

    for (size_t i = 0; i < n; ++i)
    {
        emn_->nextSample(nextSample);
        ret.col(i) = nextSample;
    }

    return ret;
}

void UncertainTransformListener::sampleFromMeanCov(const tf::Transform &mean, const MatrixXd &cov, std::vector<tf::Transform> &output, size_t n)
{
    VectorXd mean_ = transformTFToVectorXd(mean);
    MatrixXd samples = sampleFromMeanCov(mean_, cov, n);
    for (int i = 0; i < samples.cols(); i++)
        output.push_back(transformVectorXdToTF(samples.col(i).transpose()));
}

tf::Transform UncertainTransformListener::sampleFromMeanCov(const tf::Transform &mean, const MatrixXd &cov)
{
    std::vector<tf::Transform> samples;
    sampleFromMeanCov(mean, cov, samples);
    //std::cout << "Variances " << cov(0,0) << " " << cov(1,1) << " " << cov(2,2) << " " << cov(3,3) << " " << cov(4,4) << " " << cov(5,5) << endl;
    //printFrame("smpl", "mean", mean);
    //printFrame("smpl", "smpl", samples[0]);
    return samples[0];
}

VectorXd UncertainTransformListener::transformTFToVectorXd(const tf::Transform &transform)
{
    VectorXd ret;
    ret.resize(6);
    ret(0) = transform.getOrigin().x();
    ret(1) = transform.getOrigin().y();
    ret(2) = transform.getOrigin().z();
    transform.getBasis().getEulerYPR(ret(3), ret(4), ret(5));
    return ret;
}

tf::Transform UncertainTransformListener::transformVectorXdToTF(const VectorXd &vec)
{
    tf::Transform ret;
    ret.setOrigin(tf::Vector3(vec(0), vec(1), vec(2)));
    ret.getBasis().setEulerYPR(vec(3), vec(4), vec(5));
    return ret;

}


UncertainTransformListener::UncertainTransformListener(ros::Duration max_cache_time, bool spin_thread)
{
    message_subscriber_utf_ = node_.subscribe<uncertain_tf::utfMessage>("/tf_uncertainty", 100, boost::bind(&UncertainTransformListener::subscription_callback, this, _1));
    ROS_INFO("SUBSCRIBER SET UP");
    VectorXd m(6);
    MatrixXd c(6,6);
    emn_ = new EigenMultivariateNormal<double, 6>(m,c);
}

void UncertainTransformListener::subscription_callback(const uncertain_tf::utfMessageConstPtr& msg)
{
    //std::cout << "got a message :" << endl;
    for (std::vector<uncertain_tf::CovarianceStamped>::const_iterator it = msg->covariances.begin(); it != msg->covariances.end(); ++it)
    {
        uncertain_tf::CovarianceStamped sc_msg = *it;
        //std::cout << sc_msg << endl;
        StampedCovariance sc;
        covarianceStampedMsgToTF(sc_msg, sc);
        setCovariance(sc);
    }
}

void UncertainTransformListener::printFrame(std::string last_frame, std::string current_frame, tf::Transform rel)
{
    std::cout << last_frame << " to \t" << current_frame << ": \t\t(" << rel.getOrigin().x() << " , " << rel.getOrigin().y() << " , " << rel.getOrigin().z() << ") \t"
              << " (" << rel.getRotation().x() << " , " << rel.getRotation().y() << " , " << rel.getRotation().z() << " , " << rel.getRotation().w() << ")" << endl;
}

void UncertainTransformListener::sampleTransform(const std::string& target_frame, const std::string& source_frame,
        const ros::Time& time, std::vector<StampedTransform>& output, size_t n)
{
    ROS_INFO("START SAMPLING");

    ros::Time start = ros::Time::now();

    StampedTransform mean;
    ((const tf::TransformListener*)this)->lookupTransform(target_frame, source_frame, time, mean);

    std::list<std::string> source_parents;
    std::set<std::string> source_parents_set;
    std::list<std::string> target_parents;
    std::string parent = tf::resolve(((const tf::TransformListener*)this)->getTFPrefix(), source_frame);
    bool have_parent = true;
    while (have_parent)
    {
        //std::cout << "source_chain: \'" << parent << "\'" << endl;
        source_parents.push_back(parent);
        source_parents_set.insert(parent);
        //std::string next_parent;
        have_parent = ((const tf::TransformListener*)this)->getParent(parent,time,parent);
        //parent = next_parent;
    }

    parent = tf::resolve(((const tf::TransformListener*)this)->getTFPrefix(), target_frame);
    have_parent = true;
    bool connected = false;
    while (have_parent)
    {
        //std::cout << "target_chain: \'" << parent << "\'" << endl;
        target_parents.push_back(parent);

        if (source_parents_set.find(parent) != source_parents_set.end())
        {
            have_parent = false;
            connected = true;
        }
        else
            have_parent = ((const tf::TransformListener*)this)->getParent(parent,time,parent);
    }

    //todo: if not connected throw
    while ((source_parents.size() > 0) && (source_parents.back()) != target_parents.back())
    {
        //std::cout << "pop  " << source_parents.back() << " spf " << source_parents.back() << "  tpf " << target_parents.back() << endl;
        source_parents.pop_back();
    }

    // source -> common parent

    std::string last_frame_init = source_parents.front();

    source_parents.pop_front();
    target_parents.pop_back();

    std::vector<std::vector<tf::Transform> > chain_sampled_transforms;

    std::string last_frame = last_frame_init;

    //std::cout << "Sample" << k << last_frame << endl;

    //tf::Transform acc;
    //acc.setOrigin(tf::Vector3(0,0,0));
    //acc.setRotation(tf::Quaternion(0,0,0,1));

    tf::Transform zeroMean;
    zeroMean.setOrigin(tf::Vector3(0,0,0));
    zeroMean.setRotation(tf::Quaternion(0,0,0,1));


    for (std::list<std::string>::iterator it = source_parents.begin(); it!= source_parents.end(); ++it)
    {
        const std::string &current_frame = *it;

        StampedTransform rel;
        ((const tf::TransformListener*)this)->lookupTransform(current_frame, last_frame, time, rel);

        CovarianceStorage cs;
        getCovariance(lookupOrInsertFrameNumber(last_frame))->getData(time,cs);

        last_frame = current_frame;

        std::vector<tf::Transform> samples;
        chain_sampled_transforms.push_back(samples);

        if (isZero(cs.covariance_))
            chain_sampled_transforms.back().push_back(zeroMean);
        else
            sampleFromMeanCov(zeroMean,cs.covariance_,chain_sampled_transforms.back(),n);

        for (std::vector<tf::Transform>::iterator jt=chain_sampled_transforms.back().begin(); jt!= chain_sampled_transforms.back().end(); ++jt)
            *jt = rel * (*jt);

    }

    // common parent -> target
    for (std::list<std::string>::reverse_iterator it = target_parents.rbegin(); it!= target_parents.rend(); ++it)
    {
        const std::string &current_frame = *it;
        StampedTransform rel;
        ((const tf::TransformListener*)this)->lookupTransform(last_frame, current_frame, time, rel); // lookup inverse frame, where we can sample and then invert again

        CovarianceStorage cs;
        getCovariance(lookupOrInsertFrameNumber(current_frame))->getData(time,cs);

        last_frame = current_frame;
        std::vector<tf::Transform> samples;
        chain_sampled_transforms.push_back(samples);

        if (isZero(cs.covariance_))
            chain_sampled_transforms.back().push_back(zeroMean);
        else
            sampleFromMeanCov(zeroMean,cs.covariance_,chain_sampled_transforms.back(),n);

        //sampleFromMeanCov(zeroMean,cs.covariance_,chain_sampled_transforms.back(),n);
        for (std::vector<tf::Transform>::iterator jt=chain_sampled_transforms.back().begin(); jt!= chain_sampled_transforms.back().end(); ++jt)
            *jt = (rel * (*jt)).inverse(); // when walking down, we invert the transforms

        last_frame = current_frame;
    }

    for (unsigned int smp = 0; smp < n; ++smp)
    {

        tf::Transform acc;
        acc.setOrigin(tf::Vector3(0,0,0));
        acc.setRotation(tf::Quaternion(0,0,0,1));

        for (unsigned int chain = 0; chain < chain_sampled_transforms.size(); ++chain)
        {
            if (chain_sampled_transforms[chain].size() == 1)
                acc = chain_sampled_transforms[chain][0] * acc;
            else
                acc = chain_sampled_transforms[chain][smp] * acc;
        }
        mean.setData(acc);
        output.push_back(mean);
    }

    ROS_INFO("DONE SAMPLING %f", (ros::Time::now() - start).toSec());

}

} // namespace uncertain_tf
