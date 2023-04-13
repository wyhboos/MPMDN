#include <iostream>
#include <boost/random.hpp>
#include <Eigen/Dense>

int main()
{
    // 创建随机数生成器
    boost::random::mt19937 rng;
    rng.seed(std::time(0));

    // 定义高斯分布的均值和协方差矩阵
    Eigen::Vector3d mean(1.0, 2.0, 3.0);
    Eigen::Matrix3d cov;
    cov << 2.0, 0.5, 0.1,
           0.5, 3.0, 0.2,
           0.1, 0.2, 1.0;

    // 创建多维高斯分布对象
    boost::random::mvn_distribution<double, 3> mvn(mean, cov);
    Eigen::EigenMultivariateNormal<double,2> normX(mean,covar);

    // 生成多维高斯分布的随机抽样
    Eigen::Vector3d sample = mvn(rng);

    // 输出随机抽样结果
    std::cout << "Random Sample: " << sample.transpose() << std::endl;

    return 0;
}
