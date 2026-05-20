#include "z_planner.h"
#include <cmath>
#include <algorithm>
#include <limits>

ZPlanner_t::ZPlanner_t() = default;

ZPlanner_t::BoundingBox ZPlanner_t::computeBoundingBox(const std::vector<NavPose>& contour) {
    BoundingBox bb;
    bb.minX = bb.minY = std::numeric_limits<float>::max();
    bb.maxX = bb.maxY = std::numeric_limits<float>::lowest();

    for (const auto& p : contour) {
        if (p.x < bb.minX) bb.minX = p.x;
        if (p.x > bb.maxX) bb.maxX = p.x;
        if (p.y < bb.minY) bb.minY = p.y;
        if (p.y > bb.maxY) bb.maxY = p.y;
    }

    bb.centerX = (bb.minX + bb.maxX) * 0.5f;
    bb.centerY = (bb.minY + bb.maxY) * 0.5f;
    bb.width = bb.maxX - bb.minX;
    bb.height = bb.maxY - bb.minY;

    // 主方向：长轴方向
    if (bb.width >= bb.height) {
        bb.mainAngle = 0.0f;  // 沿 X 轴
    } else {
        bb.mainAngle = M_PI / 2.0f;  // 沿 Y 轴
    }

    return bb;
}

bool ZPlanner_t::lineSegmentIntersect(
    float x1, float y1, float x2, float y2,
    float x3, float y3, float x4, float y4,
    float& outX, float& outY)
{
    float denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (std::fabs(denom) < 1e-6f) return false;

    float px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom;
    float py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom;

    // 检查交点是否在线段1上
    float minx1 = std::min(x1, x2), maxx1 = std::max(x1, x2);
    float miny1 = std::min(y1, y2), maxy1 = std::max(y1, y2);
    float minx2 = std::min(x3, x4), maxx2 = std::max(x3, x4);
    float miny2 = std::min(y3, y4), maxy2 = std::max(y3, y4);

    if (px < minx1 - 1e-3f || px > maxx1 + 1e-3f || py < miny1 - 1e-3f || py > maxy1 + 1e-3f)
        return false;
    if (px < minx2 - 1e-3f || px > maxx2 + 1e-3f || py < miny2 - 1e-3f || py > maxy2 + 1e-3f)
        return false;

    outX = px;
    outY = py;
    return true;
}

bool ZPlanner_t::pointInPolygon(float x, float y, const std::vector<NavPose>& poly) {
    if (poly.size() < 3) return false;
    bool inside = false;
    for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        float xi = poly[i].x, yi = poly[i].y;
        float xj = poly[j].x, yj = poly[j].y;
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9f) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

std::vector<NavPose> ZPlanner_t::erodeContour(
    const std::vector<NavPose>& contour, float erosion)
{
    // 简化版腐蚀：直接返回原轮廓（后续可接入 cv::approxPolyDP 等）
    // 实际效果通过后续扫描时跳过边缘 erosion 距离来实现
    (void)erosion;
    return contour;
}

std::vector<ZPlanLine> ZPlanner_t::generateZPath(
    const std::vector<NavPose>& contour,
    float step, float erosion, float mainAngle)
{
    std::vector<ZPlanLine> result;
    if (contour.size() < 3) return result;

    auto bb = computeBoundingBox(contour);
    float angle = (mainAngle < -100.0f) ? bb.mainAngle : mainAngle;

    // 将轮廓向内缩 erosion
    // 简化：调整包围盒
    float cosA = std::cos(angle);
    float sinA = std::sin(angle);

    // 扫描方向：垂直于主方向
    float scanAngle = angle + M_PI / 2.0f;
    (void)scanAngle;  // 当前使用局部坐标系变换，scanCos/scanSin 未直接使用

    // 将轮廓变换到以主方向为 X 轴的局部坐标系
    auto rotatePoint = [&](const NavPose& p) -> std::pair<float, float> {
        float dx = p.x - bb.centerX;
        float dy = p.y - bb.centerY;
        float lx = dx * cosA + dy * sinA;
        float ly = -dx * sinA + dy * cosA;
        return {lx, ly};
    };

    // 计算局部坐标系下的包围盒
    float localMinX = std::numeric_limits<float>::max();
    float localMaxX = std::numeric_limits<float>::lowest();
    float localMinY = std::numeric_limits<float>::max();
    float localMaxY = std::numeric_limits<float>::lowest();

    for (const auto& p : contour) {
        auto [lx, ly] = rotatePoint(p);
        localMinX = std::min(localMinX, lx);
        localMaxX = std::max(localMaxX, lx);
        localMinY = std::min(localMinY, ly);
        localMaxY = std::max(localMaxY, ly);
    }

    // 向内缩 erosion
    localMinX += erosion;
    localMaxX -= erosion;
    localMinY += erosion;
    localMaxY -= erosion;

    if (localMinX >= localMaxX || localMinY >= localMaxY) return result;

    // 沿局部 Y 方向（扫描方向）生成平行线
    std::vector<std::pair<NavPose, NavPose>> localLines;
    bool forward = true;

    for (float y = localMinY; y <= localMaxY; y += step) {
        // 在局部坐标系下，扫描线是水平线 y = const，X 从 min 到 max
        std::vector<float> intersections;

        // 与轮廓每条边求交
        for (size_t i = 0, j = contour.size() - 1; i < contour.size(); j = i++) {
            auto [x1, y1] = rotatePoint(contour[j]);
            auto [x2, y2] = rotatePoint(contour[i]);

            // 线段是否与水平线 y = const 相交？
            if ((y1 <= y && y2 > y) || (y2 <= y && y1 > y)) {
                float t = (y - y1) / (y2 - y1 + 1e-9f);
                float xint = x1 + t * (x2 - x1);
                intersections.push_back(xint);
            }
        }

        if (intersections.size() < 2) continue;

        std::sort(intersections.begin(), intersections.end());

        // 取成对的交点作为线段
        for (size_t k = 0; k + 1 < intersections.size(); k += 2) {
            float xStart = intersections[k];
            float xEnd = intersections[k + 1];

            // 裁剪到安全边界
            xStart = std::max(xStart, localMinX);
            xEnd = std::min(xEnd, localMaxX);
            if (xEnd - xStart < step * 0.5f) continue;

            // 变换回世界坐标
            auto worldPoint = [&](float lx, float ly) -> NavPose {
                float wx = bb.centerX + lx * cosA - ly * sinA;
                float wy = bb.centerY + lx * sinA + ly * cosA;
                return NavPose{wx, wy, angle};
            };

            NavPose pStart = worldPoint(xStart, y);
            NavPose pEnd = worldPoint(xEnd, y);

            localLines.push_back({pStart, pEnd});
        }
    }

    // 将线段按弓字形排序（交替方向）
    for (size_t i = 0; i < localLines.size(); ++i) {
        ZPlanLine line;
        if (forward) {
            line.start = localLines[i].first;
            line.end = localLines[i].second;
        } else {
            line.start = localLines[i].second;
            line.end = localLines[i].first;
        }
        result.push_back(line);
        forward = !forward;
    }

    return result;
}
