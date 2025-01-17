#include <fstream>
#include <iostream>
#include "eigenmvn.h"
#ifndef M_PI
#define M_PI REAL(3.1415926535897932384626433832795029)
#endif

/**
  Take a pair of un-correlated variances.
  Create a covariance matrix by correlating 
  them, sandwiching them in a rotation matrix.
*/
Eigen::Matrix2d genCovar(double v0,double v1,double theta)
{
  Eigen::Matrix2d rot = Eigen::Rotation2Dd(theta).matrix();
  return rot*Eigen::DiagonalMatrix<double,2,2>(v0,v1)*rot.transpose();
}

int main()
{
  Eigen::Vector2d mean;
  Eigen::Matrix2d covar;
  mean << -1,0.5; // Set the mean
  // Create a covariance matrix
  // Much wider than it is tall
  // and rotated clockwise by a bit
  covar = genCovar(3,0.1,M_PI/5.0);
  covar(0,0) = 0.1;
  covar(0,1) = 0;
  covar(1,0) = 0;
  covar(1,1) = 0.1;


  // Create a bivariate gaussian distribution of doubles.
  // with our chosen mean and covariance
  const int dim = 2;
  Eigen::EigenMultivariateNormal<double> normX_solver(mean,covar);
  const uint64_t seed=std::mt19937::default_seed;
  normX_solver.setCovar(covar);

  auto a = normX_solver.samples(5).transpose();
  std::cout<<a<<std::endl;
  std::cout<<a.rows()<<std::endl;
  std::cout<<a.cols()<<std::endl;
  std::cout<<a.row(1)(0)<<std::endl;
  auto b = normX_solver.samples(5).transpose();
  std::cout<<b<<std::endl;
  std::cout<<b.rows()<<std::endl;
  std::cout<<b.cols()<<std::endl;
  // std::cout<<a.

  // std::cout<<typeid(a)<<std::endl;
  // std::ofstream file_solver("samples_solver.txt");

  // Generate some samples and write them out to file
  // for plotting
  // file_solver << normX_solver.samples(5000).transpose() << std::endl;

  // // same for Cholesky decomposition.
  // covar = genCovar(3,0.1,M_PI/5.0);
  // Eigen::EigenMultivariateNormal<double> normX_cholesk(mean,covar,true);
  // std::ofstream file_cholesky("samples_cholesky.txt");
  // file_cholesky << normX_cholesk.samples(5000).transpose() << std::endl;
}
