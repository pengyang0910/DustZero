#include "navtask/Tf/tf.h"
#include "navtask/NavUtils/NavUtils.h"
#include <cmath>
#include <iostream>

int main() {
    // 历史 30 度坐标变换样例：
    // 坐标系 1 → 坐标系 2：平移 (√3, 1)，旋转 30°
    Frame_t frame12;
    frame12.SetRotationFromRPY(0, 0, dtor(30));
    frame12.SetTransVector(sqrt(3.0), 1, 0);

    // p1 在坐标系 1 中的位姿
    double p1_x = 1.0, p1_y = 0.0, p1_a = dtor(30);

    // 正变换：坐标系 1 → 坐标系 2
    double p12_x = 0.0, p12_y = 0.0, p12_a = 0.0;
    frame12.GetTFPose(p1_x, p1_y, p1_a, p12_x, p12_y, p12_a);

    // 逆变换：坐标系 2 → 坐标系 1
    double p21_x = 0.0, p21_y = 0.0, p21_a = 0.0;
    frame12.GetInvTFPose(p12_x, p12_y, p12_a, p21_x, p21_y, p21_a);

    std::cout << p12_x << "\t" << p12_y << "\t" << rtod(p12_a) << std::endl;
    std::cout << p21_x << "\t" << p21_y << "\t" << rtod(p21_a) << std::endl;

    // 验证正逆变换可恢复原始位姿
    bool ok = (std::fabs(p21_x - p1_x) < 1e-3) &&
              (std::fabs(p21_y - p1_y) < 1e-3) &&
              (std::fabs(p21_a - p1_a) < 1e-3);
    return ok ? 0 : 1;
}
