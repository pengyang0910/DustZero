#pragma once
#include "navtask/nav_types.h"
#include <vector>

class ZPlanner_t {
public:
    ZPlanner_t();

    // 生成弓字形路径（简化版）
    // 输入：
    //   contour    — 区块多边形轮廓（mm 坐标）
    //   step       — 线间距（mm，默认 250 mm）
    //   erosion    — 轮廓内缩安全距离（mm，默认 100 mm）
    //   mainAngle  — 主方向（rad，默认 -1000 表示自动计算）
    // 输出：
    //   ZPlanLine 列表，按清扫顺序排列
    std::vector<ZPlanLine> generateZPath(
        const std::vector<NavPose>& contour,
        float step = 250.0f,
        float erosion = 100.0f,
        float mainAngle = -1000.0f);

private:
    // 计算多边形包围盒及主方向
    struct BoundingBox {
        float minX, minY, maxX, maxY;
        float centerX, centerY;
        float mainAngle;  // 长轴方向
        float width, height;
    };

    BoundingBox computeBoundingBox(const std::vector<NavPose>& contour);

    // 判断线段是否与多边形边相交，返回交点
    bool lineSegmentIntersect(
        float x1, float y1, float x2, float y2,
        float x3, float y3, float x4, float y4,
        float& outX, float& outY);

    // 点是否在多边形内（射线法）
    bool pointInPolygon(float x, float y, const std::vector<NavPose>& poly);

    // 将轮廓向内腐蚀（简化：直接缩小包围盒+裁剪）
    std::vector<NavPose> erodeContour(
        const std::vector<NavPose>& contour, float erosion);
};
