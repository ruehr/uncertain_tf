#ifndef __EIGENMULTIVARIATENORMAL_HPP
#define __EIGENMULTIVARIATENORMAL_HPP

//found at http://lost-found-wandering.blogspot.de/2011/05/sampling-from-multivariate-normal-in-c.html

#include <Eigen/Dense>

#include <math.h>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

/**
    We find the eigen-decomposition of the covariance matrix.
    We create a vector of normal samples scaled by the eigenvalues.
    We rotate the vector by the eigenvectors.
    We add the mean.
*/
template<typename _Scalar, int _size>
class EigenMultivariateNormal
{
    boost::mt19937 rng;    // The uniform pseudo-random algorithm
    boost::normal_distribution<_Scalar> norm;  // The gaussian combinator
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<_Scalar> >
       randN; // The 0-mean unit-variance normal generator

    Eigen::Matrix<_Scalar,_size,_size> rot;
    Eigen::Matrix<_Scalar,_size,1> scl;

    Eigen::Matrix<_Scalar,_size,1> mean;

public:
    EigenMultivariateNormal(const Eigen::Matrix<_Scalar,_size,1>& meanVec,
        const Eigen::Matrix<_Scalar,_size,_size>& covarMat)
        : randN(rng,norm)
    {
        setCovar(covarMat);
        setMean(meanVec);
    }

    void setCovar(const Eigen::Matrix<_Scalar,_size,_size>& covarMat)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix<_Scalar,_size,_size> >
           eigenSolver(covarMat);
        rot = eigenSolver.eigenvectors();
        scl = eigenSolver.eigenvalues();
        for (int ii=0;ii<_size;++ii) {
            scl(ii,0) = sqrt(scl(ii,0));
        }
    }

    void setMean(const Eigen::Matrix<_Scalar,_size,1>& meanVec)
    {
        mean = meanVec;
    }

    void nextSample(Eigen::Matrix<_Scalar,_size,1>& sampleVec)
    {
        for (int ii=0;ii<_size;++ii) {
            sampleVec(ii,0) = randN()*scl(ii,0);
        }
        sampleVec = rot*sampleVec + mean;
    }

};



/**
    Get a a sample from a univariant gaussian distribution
*/
template<typename _Scalar>
class UnivariateNormal
{
    boost::mt19937 rng;    // The uniform pseudo-random algorithm
    boost::normal_distribution<_Scalar> norm;  // The gaussian combinator
    boost::variate_generator<boost::mt19937&,boost::normal_distribution<_Scalar> >
       randN; // The 0-mean unit-variance normal generator

    _Scalar mean_;
    _Scalar stdev_;

public:

    UnivariateNormal(const _Scalar & mean, const _Scalar &var)
        : randN(rng,norm)
    {
        setVar(var);
        setMean(mean);
    }

    void setVar(const _Scalar & var)
    {
        stdev_ = sqrt(fabs(var));
    }

    void setMean(const _Scalar & mean)
    {
        mean_ = mean;
    }

    void nextSample(_Scalar &sample)
    {
        sample = randN() * stdev_ + mean_;
    }

};



#endif
