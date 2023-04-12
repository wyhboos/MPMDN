
#include <iostream>
#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>

int main() {
    // 创建随机数生成器和三维高斯分布的协方差矩阵
    boost::random::mt19937 rng; // 随机数生成器
    double mean_x = 0.0; // 高斯分布的均值 x
    double mean_y = 0.0; // 高斯分布的均值 y
    double mean_z = 0.0; // 高斯分布的均值 z
    double cov_xx = 1.0; // 协方差矩阵的元素 (0, 0)
    double cov_yy = 2.0; // 协方差矩阵的元素 (1, 1)
    double cov_zz = 3.0; // 协方差矩阵的元素 (2, 2)
    double cov_xy = 0.5; // 协方差矩阵的元素 (0, 1) = (1, 0)
    double cov_xz = 0.3; // 协方差矩阵的元素 (0, 2) = (2, 0)
    double cov_yz = 0.4; // 协方差矩阵的元素 (1, 2) = (2, 1)
    // boost::math::normal_distribution<> normalDist_x(mean_x, cov_xx); // 高斯分布 x
    // boost::math::normal_distribution<> normalDist_y(mean_y, cov_yy); // 高斯分布 y
    // boost::math::normal_distribution<> normalDist_z(mean_z, cov_zz); // 高斯分布 z
    boost::random::multivariate_normal_distribution<> multivariateNormalDist(
        {mean_x, mean_y, mean_z}, // 均值向量
        {{cov_xx, cov_xy, cov_xz}, // 协方差矩阵
         {cov_xy, cov_yy, cov_yz},
         {cov_xz, cov_yz, cov_zz}});

    // 生成三维高斯分布随机数
    int num_samples = 10;
    for (int i = 0; i < num_samples; ++i) {
        boost::random::variate_generator<boost::random::mt19937&, boost::random::multivariate_normal_distribution<>> generator(rng, multivariateNormalDist);
        boost::math::tuple<double, double, double> sample = generator();
        std::cout << "Sample " << i + 1 << ": x = " << sample.get<0>() << ", y = " << sample.get<1>() << ", z = " << sample.get<2>() << std::endl;
    }

    return 0;
}
