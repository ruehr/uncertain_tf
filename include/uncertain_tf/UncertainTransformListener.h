
#ifndef _UNCERTAIN_TRANSFORM_LISTENER_
#define _UNCERTAIN_TRANSFORM_LISTENER_

#include "uncertain_tf/UncertainTransformer.h"
#include "uncertain_tf/utfMessage.h"
#include "uncertain_tf/EigenMultiVariateNormal.hpp"

using namespace tf;
using namespace Eigen;

namespace uncertain_tf {

class UncertainTransformListener : public UncertainTransformer
{

public:

    UncertainTransformListener(ros::Duration max_cache_time = ros::Duration(tf::Transformer::DEFAULT_CACHE_TIME), bool spin_thread = true);

    //! sample from frame names and time into a vector of transforms
    void sampleTransform(const std::string& target_frame, const std::string& source_frame,
                         const ros::Time& time, std::vector<StampedTransform>& transform, size_t n);

    void printFrame(std::string last_frame, std::string current_frame, tf::Transform rel);


    //!sample n transforms given mean and cov in eigen types
    MatrixXd sampleFromMeanCov(const VectorXd &mean_, const MatrixXd &cov_, size_t n = 1);

    //!sample n transforms given mean in tf and cov in eigen type
    void sampleFromMeanCov(const tf::Transform &mean, const MatrixXd &cov, std::vector<tf::Transform> &output, size_t n = 1);

    //!sample a single transform given mean in tf and cov in eigen type
    tf::Transform sampleFromMeanCov(const tf::Transform &mean, const MatrixXd &cov);

    //! convert from tf to eigen
    VectorXd transformTFToVectorXd(const tf::Transform &transform);

    //! convert from eigen to tf
    tf::Transform transformVectorXdToTF(const VectorXd &vec);

    //! check if a covariance matrix is zero - as initialized when no cov is known
    bool isZero(const MatrixXd mat);

    //! calculate the covariance of a given sample set in eigen type
    template <typename Derived, typename OtherDerived> void calculateSampleCovariance(const MatrixBase<Derived>& x, const MatrixBase<Derived>& y, MatrixBase<OtherDerived> & C_);

    //! calculate the covariance of a given sample set in eigen type
    VectorXd calculateSampleMean(const MatrixXd &x);

    //! convert a vector of transforms to a eigen sampleset matrix for covariance calculation
    MatrixXd sampleSetTFtoMatrixXd(std::vector<tf::StampedTransform> sampleset);

private:

    void subscription_callback(const uncertain_tf::utfMessageConstPtr& msg);

    ros::NodeHandle node_;
    ros::Subscriber message_subscriber_utf_;
    EigenMultivariateNormal<double, 6> *emn_;
};

template <typename Derived, typename OtherDerived>
void UncertainTransformListener::calculateSampleCovariance(const MatrixBase<Derived>& x, const MatrixBase<Derived>& y, MatrixBase<OtherDerived> & C_)
{
    typedef typename Derived::Scalar Scalar;
    typedef typename internal::plain_row_type<Derived>::type RowVectorType;

    const Scalar num_observations = static_cast<Scalar>(x.rows());

    const RowVectorType x_mean = x.colwise().sum() / num_observations;
    const RowVectorType y_mean = y.colwise().sum() / num_observations;

    MatrixBase<OtherDerived>& C = const_cast< MatrixBase<OtherDerived>& >(C_);

    C.derived().resize(x.cols(),x.cols()); // resize the derived object
    C = (x.rowwise() - x_mean).transpose() * (y.rowwise() - y_mean) / num_observations;
}


}

#endif
