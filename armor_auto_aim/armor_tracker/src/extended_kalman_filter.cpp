#include "armor_tracker/extended_kalman_filter.hpp"

namespace armor {
ExtendedKalmanFilter::ExtendedKalmanFilter(
    const VecVecFunc& f,
    const VecVecFunc& h,
    const VecMatFunc& j_f,
    const VecMatFunc& j_h,
    const VoidMatFunc& u_q,
    const VecMatFunc& u_r,
    const Eigen::MatrixXd& P0
):
    f(f),
    h(h),
    jacobian_f(j_f),
    jacobian_h(j_h),
    update_Q(u_q),
    update_R(u_r),
    P_post(P0),
    n(P0.rows()),
    I(Eigen::MatrixXd::Identity(n, n)),
    x_pri(n),
    x_post(n) {}

void ExtendedKalmanFilter::SetState(const Eigen::VectorXd& x0) {
    x_post = x0;
}

Eigen::MatrixXd ExtendedKalmanFilter::Predict() {
    F = jacobian_f(x_post), Q = update_Q();

    x_pri = f(x_post);
    P_pri = F * P_post * F.transpose() + Q;

    // 处理在下一次预测之前没有测量的情况
    x_post = x_pri;
    P_post = P_pri;

    return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::Update(const Eigen::VectorXd& z) {
    H = jacobian_h(x_pri), R = update_R(z);

    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
    x_post = x_pri + K * (z - h(x_pri));
    P_post = (I - K * H) * P_pri;

    return x_post;
}

} // namespace armor
