#include "GlobalMatchP.h"
#include "opencv2/opencv.hpp"
#include "xutils.h"
#include <map>
#include <mutex>
#include <unordered_map>


//#define GMP_DEBUG

#define GMP_START_LAYER 4

#define GMP_LAYERS 4
#define GMP_LAYER_SCALING_FACTOR 2

#define GMP_ANGLE_STEP_DEG 2

#define GMP_DECAY_KERNEL_SIZE 9
#define GMP_LASER_RADIUS_METER (4.8f)

int GMP_MAX_CADIDATES[GMP_LAYERS] = {5, 7, 15, 30};
#define GMP_NMS_RADIUS 2
#define GMP_LOCAL_MAXIMUM_THRESHOLD 0.5f
#define GMP_LOWER_LAYER_SEARCH_RADIUS 5

// refine
#define GMP_REFINE_POS_STEP_M 0.005

static std::recursive_mutex GMPmtx;
static int GMP_map_width = 0;
static int GMP_map_height = 0;
static float GMP_map_resolution = 0.05f;
static float GMP_map_origin_px = 0;
static float GMP_map_origin_py = 0;
static cv::Point GMP_laser_extend_offset;

static cv::Mat decay_kernel[GMP_LAYERS];
static cv::Mat map[GMP_LAYERS];
static cv::Mat map_display[GMP_LAYERS];
static cv::Mat map_dilation[GMP_LAYERS];
static cv::Mat prob_map[GMP_LAYERS];
static cv::Mat prob_map_direction[GMP_LAYERS];
static cv::Mat prob_map_nms[GMP_LAYERS];

static std::vector<cv::Rect> searchROIs[GMP_LAYERS];

#ifdef GMP_DEBUG
cv::Mat prob_map_debug;
cv::Mat prob_map_direction_debug;
cv::Mat prob_map_nms_debug;
#endif

static inline int32_t Key(const int32_t &x, const int32_t &y) {
  return ((x << 16) | (y & 0xFFFF));
}

OPTIMIZE_FUNC static int
hUpdateProbMap(cv::Mat &map_dilation, cv::Mat &prob_map,
               cv::Mat &prob_map_direction, cv::Rect roi,
               std::vector<std::vector<cv::Point3i>> &laserProjection,
               bool bLayer0) {
  if ((roi.br().x > prob_map.cols) || (roi.br().y > prob_map.rows)) {
    printf("invalid parameter\n");
    std::abort();
    return -1;
  }

  for (int v = roi.tl().y; v < roi.br().y; v++) {
    for (int u = roi.tl().x; u < roi.br().x; u++) {
#ifndef GMP_DEBUG
      if (bLayer0 && (map_display[0].at<uchar>(v, u) != 225)) // not free
      {
        continue;
      }

      if (prob_map.at<int16_t>(v, u) != 0) // overlaped by other ROI
      {
        continue;
      }
#endif
      float max_prob = 0;
      uint8_t max_rotation_idx = 0;
      for (size_t rotation_idx = 0; rotation_idx < laserProjection.size();
           rotation_idx++) {
        int valid_point_cnt = laserProjection[rotation_idx].size();
        int prob_sum = 0;
        int weight_sum = 0;
        cv::Point3i *pPoint =
            (cv::Point3i *)laserProjection[rotation_idx].data();
        for (int laser_point_idx = 0; laser_point_idx < valid_point_cnt;
             laser_point_idx++, pPoint++) {
          prob_sum +=
              map_dilation.at<uchar>(v + pPoint->y, u + pPoint->x) * pPoint->z;
          weight_sum += pPoint->z;
        }
        if (valid_point_cnt > 0) {
          float prob = prob_sum / 255.0 / weight_sum;
          if (prob > max_prob) {
            max_prob = prob;
            max_rotation_idx = rotation_idx;
          }
        }
      }
      prob_map.at<int16_t>(v, u) = (int16_t)(max_prob * 10000);
      prob_map_direction.at<uint8_t>(v, u) = max_rotation_idx;
    }
  }
  return 0;
}

OPTIMIZE_FUNC static int hNMS(cv::Mat &irProb_map, cv::Rect roi,
                              std::vector<std::pair<int16_t, cv::Point>> &orLMs,
                              cv::Mat *opProp_map_nms) {
  orLMs.clear();
  if ((roi.br().x > irProb_map.cols) || (roi.br().y > irProb_map.rows)) {
    printf("hNMS invalid parameter\n");
    std::abort();
    return -1;
  }
  cv::Point searchTL(roi.tl());
  cv::Point searchBR(roi.br());

  if (searchTL.x < GMP_NMS_RADIUS)
    searchTL.x = GMP_NMS_RADIUS;
  if (searchTL.y < GMP_NMS_RADIUS)
    searchTL.y = GMP_NMS_RADIUS;

  if (searchBR.x > irProb_map.cols - GMP_NMS_RADIUS)
    searchBR.x = irProb_map.cols - GMP_NMS_RADIUS;
  if (searchBR.y > irProb_map.rows - GMP_NMS_RADIUS)
    searchBR.y = irProb_map.rows - GMP_NMS_RADIUS;

  for (int v = searchTL.y; v < searchBR.y; v++) {
    for (int u = searchTL.x; u < searchBR.x; u++) {
      bool bMaximum = true;
      int16_t center_prob = irProb_map.at<int16_t>(v, u);
      if (center_prob > (int16_t)(GMP_LOCAL_MAXIMUM_THRESHOLD * 10000)) {
        for (int vv = v - GMP_NMS_RADIUS; vv <= v + GMP_NMS_RADIUS; vv++) {
          for (int uu = u - GMP_NMS_RADIUS; uu <= u + GMP_NMS_RADIUS; uu++) {
            if (irProb_map.at<int16_t>(vv, uu) > center_prob) {
              bMaximum = false;
              break;
            }
          }
          if (!bMaximum) {
            break;
          }
        }
        if (bMaximum) {
          orLMs.push_back(std::make_pair(center_prob, cv::Point(u, v)));
          if (opProp_map_nms) {
            opProp_map_nms->at<int16_t>(v, u) = center_prob;
          }
        }
      }
    }
  }

  if (orLMs.size() > 0) {
    std::sort(orLMs.begin(), orLMs.end(),
              [](std::pair<int16_t, cv::Point> &a,
                 std::pair<int16_t, cv::Point> &b) -> bool {
                return (a.first > b.first);
              });
  }
  return 0;
}

OPTIMIZE_FUNC static int
hMergeLM(std::vector<std::pair<int16_t, cv::Point>> &iorLMs,
         std::vector<std::pair<int16_t, cv::Point>> &tmp_LMs,
         int max_candidates) {
  std::vector<std::pair<int16_t, cv::Point>> lms = iorLMs;
  iorLMs.clear();
  iorLMs.reserve(max_candidates);
  // double counter based lms merge(sorted)
  size_t i = 0;
  size_t j = 0;
  int result_cnt = 0;
  for (;
       result_cnt < max_candidates && !(i >= lms.size() && j >= tmp_LMs.size());
       result_cnt++) {
    if (i >= lms.size()) {
      iorLMs.emplace_back(tmp_LMs[j]);
      j++;
    } else if (j >= tmp_LMs.size()) {
      iorLMs.emplace_back(lms[i]);
      i++;
    } else if (lms[i].first == tmp_LMs[j].first) {
      if ((lms[i].second.x == tmp_LMs[j].second.x) &&
          (lms[i].second.y == tmp_LMs[j].second.y)) {
        iorLMs.emplace_back(lms[i]);
        i++;
        j++; // same point
      } else {
        iorLMs.emplace_back(lms[i]);
        i++;
      }
    } else if (lms[i].first > tmp_LMs[j].first) {
      iorLMs.emplace_back(lms[i]);
      i++;
    } else {
      iorLMs.emplace_back(tmp_LMs[j]);
      j++;
    }
  }

  return 0;
}

OPTIMIZE_FUNC static std::vector<std::vector<cv::Point2f>>
hGetRawLaserProjection(int16_t *laser_scan_mm, int laser_scan_cnt,
                       float angle_resolution_degree,
                       int angle_rotate_start_idx, int angle_rotation_step,
                       int rotation_cnt, float map_resolution,
                       cv::Point laser_extend_offset) {
  if (laser_scan_cnt != (360 / angle_resolution_degree)) {
    std::abort();
  }
  float *mm2pixel_cos_table = new float[laser_scan_cnt];
  float *mm2pixel_sin_table = new float[laser_scan_cnt];
  ON_SCOPE_EXIT([=]() {
    delete[] mm2pixel_cos_table;
    delete[] mm2pixel_sin_table;
  });

  for (int i = 0; i < laser_scan_cnt; i++) {
    mm2pixel_cos_table[i] =
        std::cos(i * angle_resolution_degree * CV_PI / 180) / 1000.0 /
        map_resolution;
    mm2pixel_sin_table[i] =
        std::sin(i * angle_resolution_degree * CV_PI / 180) / 1000.0 /
        map_resolution;
  }

  float u_bias = 0.5 + laser_extend_offset.x;
  float v_bias = 0.5 + laser_extend_offset.y;

  std::vector<std::vector<cv::Point2f>> laserProjection;
  laserProjection.resize(rotation_cnt);
  for (int rotation_idx = 0; rotation_idx < rotation_cnt; rotation_idx++) {
    int angle_offset_idx =
        angle_rotate_start_idx + rotation_idx * angle_rotation_step;
    laserProjection[rotation_idx].reserve(laser_scan_cnt);
    for (int laser_point_idx = 0; laser_point_idx < laser_scan_cnt;
         laser_point_idx++) {
      if (laser_scan_mm[laser_point_idx] > 0) {
        int angle_index = (laser_point_idx + angle_offset_idx) % laser_scan_cnt;
        cv::Point2f p;
        p.x = u_bias +
              laser_scan_mm[laser_point_idx] * mm2pixel_cos_table[angle_index];
        p.y = v_bias +
              laser_scan_mm[laser_point_idx] * mm2pixel_sin_table[angle_index];
        laserProjection[rotation_idx].emplace_back(p);
      }
    }
  }

  return laserProjection;
}

OPTIMIZE_FUNC static std::vector<std::vector<cv::Point3i>>
hCompressLaserProjection2Pixel(
    std::vector<std::vector<cv::Point2f>> &irRawLaserProjection,
    cv::Point2f offset) {
  std::vector<std::vector<cv::Point3i>> results;
  results.resize(irRawLaserProjection.size());
  for (size_t rotation_idx = 0; rotation_idx < irRawLaserProjection.size();
       rotation_idx++) {
    std::unordered_map<int32_t, int32_t> tmp_map;
    int laser_point_cnt = irRawLaserProjection[rotation_idx].size();
    for (int laser_point_idx = 0; laser_point_idx < laser_point_cnt;
         laser_point_idx++) {
      int u = irRawLaserProjection[rotation_idx][laser_point_idx].x + offset.x;
      int v = irRawLaserProjection[rotation_idx][laser_point_idx].y + offset.y;
      int32_t key = Key(u, v);
      if (tmp_map.find(key) != tmp_map.end()) {
        tmp_map[key]++;
      } else {
        tmp_map.insert(std::make_pair(key, 1));
      }
    }
    results[rotation_idx].reserve(tmp_map.size());
    for (auto iter = tmp_map.begin(); iter != tmp_map.end(); iter++) {
      int u = (iter->first & 0xFFFF0000) >> 16;
      int v = (iter->first & 0x0000FFFF);
      results[rotation_idx].emplace_back(u, v, iter->second);
    }
  }
  return results;
}

OPTIMIZE_FUNC int updateGMPMap(uint8_t *pMapData, int width, int height,
                               float map_resolution, float map_origin_px,
                               float map_origin_py) {
  std::lock_guard<std::recursive_mutex> lock(GMPmtx);
  if ((!pMapData) || (width <= 8) || (height <= 8)) {
    printf("%s %d: invalid parameter %p, %d, %d\n", __FILE__, __LINE__,
           (void *)pMapData, width, height);
    std::abort();
  }

  uint64_t t0 = getTimeStap_us();

  GMP_map_width = width;
  GMP_map_height = height;
  GMP_map_resolution = map_resolution;
  GMP_map_origin_px = map_origin_px;
  GMP_map_origin_py = map_origin_py;

  // 0. pre-calculate
  for (int layer_idx = 0; layer_idx < GMP_LAYERS; layer_idx++) {
    int kernel_size = GMP_DECAY_KERNEL_SIZE - 2 * layer_idx;
    decay_kernel[layer_idx] =
        cv::Mat(kernel_size, kernel_size, CV_32FC1, cv::Scalar(0));
    for (int y = -kernel_size / 2; y <= kernel_size / 2; y++) {
      for (int x = -kernel_size / 2; x <= kernel_size / 2; x++) {
        int u = x + kernel_size / 2;
        int v = y + kernel_size / 2;
        decay_kernel[layer_idx].at<float>(v, u) = 0;

        float dist = std::sqrt(x * x + y * y);
        float radius = kernel_size / 2 + 1;
        if (dist <= radius) {
          decay_kernel[layer_idx].at<float>(v, u) = (1 - dist / radius);
        }
      }
    }
  }

  // 1. new mat
  GMP_laser_extend_offset =
      cv::Point((int)std::ceil(GMP_LASER_RADIUS_METER / map_resolution),
                (int)std::ceil(GMP_LASER_RADIUS_METER / map_resolution));
  for (int layer_idx = 0; layer_idx < GMP_LAYERS; layer_idx++) {
    int factor = std::pow(GMP_LAYER_SCALING_FACTOR, layer_idx);
    map[layer_idx] = cv::Mat(GMP_map_height / factor, GMP_map_width / factor,
                             CV_8UC1, cv::Scalar(0));
    map_display[layer_idx] =
        cv::Mat(GMP_map_height / factor, GMP_map_width / factor, CV_8UC1,
                cv::Scalar(0));
    map_dilation[layer_idx] =
        cv::Mat((GMP_map_height + 2 * GMP_laser_extend_offset.y) / factor,
                (GMP_map_width + 2 * GMP_laser_extend_offset.x) / factor,
                CV_8UC1, cv::Scalar(0));
  }

  // 2. populate mat
  uint8_t *pData = pMapData;
  for (int v = 0; v < height; v++) {
    for (int u = 0; u < width; u++) {
      if (*pData == 1) // obstacle
      {
        for (int layer_idx = 0; layer_idx < GMP_LAYERS; layer_idx++) {
          int factor = std::pow(GMP_LAYER_SCALING_FACTOR, layer_idx);
          map[layer_idx].at<uint8_t>(v / factor, u / factor) = 255;
          map_display[layer_idx].at<uint8_t>(v / factor, u / factor) = 0;
        }
      } else if (*pData == 0) // unknown
      {
        for (int layer_idx = 0; layer_idx < GMP_LAYERS; layer_idx++) {
          int factor = std::pow(GMP_LAYER_SCALING_FACTOR, layer_idx);
          map_display[layer_idx].at<uint8_t>(v / factor, u / factor) = 128;
        }
      } else if (*pData == 255) // free
      {
        for (int layer_idx = 0; layer_idx < GMP_LAYERS; layer_idx++) {
          int factor = std::pow(GMP_LAYER_SCALING_FACTOR, layer_idx);
          map_display[layer_idx].at<uint8_t>(v / factor, u / factor) = 225;
        }
      }
      pData++;
    }
  }
  // dialation map
  for (int layer_idx = 0; layer_idx < GMP_LAYERS; layer_idx++) {
    int factor = std::pow(GMP_LAYER_SCALING_FACTOR, layer_idx);
    int kernel_size = decay_kernel[layer_idx].rows;
    for (int v = kernel_size / 2; v < map[layer_idx].rows - kernel_size / 2;
         v++) {
      for (int u = kernel_size / 2; u < map[layer_idx].cols - kernel_size / 2;
           u++) {
        if (map[layer_idx].at<uchar>(v, u) > 0) {
          for (int yy = -kernel_size / 2; yy <= kernel_size / 2; yy++) {
            for (int xx = -kernel_size / 2; xx <= kernel_size / 2; xx++) {
              uchar tmp = map[layer_idx].at<uchar>(v, u) *
                          decay_kernel[layer_idx].at<float>(
                              yy + kernel_size / 2, xx + kernel_size / 2);
              if (tmp > map_dilation[layer_idx].at<uchar>(
                            v + yy + GMP_laser_extend_offset.y / factor,
                            u + xx + GMP_laser_extend_offset.x / factor)) {
                map_dilation[layer_idx].at<uchar>(
                    v + yy + GMP_laser_extend_offset.y / factor,
                    u + xx + GMP_laser_extend_offset.x / factor) = tmp;
              }
            }
          }
        }
      }
    }
  }

  printf("update GMP map %dx%d cost %d us\n", GMP_map_width, GMP_map_height,
         (uint32_t)(getTimeStap_us() - t0));

  return 0;
}

OPTIMIZE_FUNC int GMPMatch(float *laser_scan, int laser_point_cnt,
                           float laser_angle_resolution_deg,
                           std::vector<GMP_results_t> &orResults) {
  std::lock_guard<std::recursive_mutex> lock(GMPmtx);

  uint64_t t0 = getTimeStap_us();

  int16_t laser_scan_mm[360] = {0};
  memset(laser_scan_mm, 0, 360 * sizeof(int16_t));
  for (int i = 0; i < 360; i++) {
    int idx = i / laser_angle_resolution_deg;
    if (idx >= 0 && idx < laser_point_cnt)
      laser_scan_mm[i] = laser_scan[idx] * 1000;
    if (laser_scan_mm[i] >= (GMP_LASER_RADIUS_METER * 1000)) {
      laser_scan_mm[i] = 0;
    }
  }

  // 1. create prob map
  for (int layer_idx = 0; layer_idx < GMP_LAYERS; layer_idx++) {
    prob_map[layer_idx] = cv::Mat(map[layer_idx].rows, map[layer_idx].cols,
                                  CV_16SC1, cv::Scalar(0));
    prob_map_nms[layer_idx] = cv::Mat(map[layer_idx].rows, map[layer_idx].cols,
                                      CV_16SC1, cv::Scalar(0));
    prob_map_direction[layer_idx] = cv::Mat(
        map[layer_idx].rows, map[layer_idx].cols, CV_8UC1, cv::Scalar(0));
    searchROIs[layer_idx].clear();
  }
#ifdef GMP_DEBUG
  prob_map_debug = cv::Mat(map[0].rows, map[0].cols, CV_16SC1, cv::Scalar(0));
  prob_map_nms_debug =
      cv::Mat(map[0].rows, map[0].cols, CV_16SC1, cv::Scalar(0));
  prob_map_direction_debug =
      cv::Mat(map[0].rows, map[0].cols, CV_8UC1, cv::Scalar(0));
#endif

  // 2. calculate prob map and get LMs
  searchROIs[GMP_START_LAYER - 1].push_back(
      cv::Rect(0, 0, prob_map[GMP_START_LAYER - 1].cols,
               prob_map[GMP_START_LAYER - 1].rows));
  std::vector<std::pair<int16_t, cv::Point>> final_LMs;
  for (int layer_idx = (GMP_START_LAYER - 1); layer_idx >= 0; layer_idx--) {
    // 2.0 prepare
    int factor = std::pow(GMP_LAYER_SCALING_FACTOR, layer_idx);

    std::vector<std::vector<cv::Point2f>> rawLaserProjection =
        hGetRawLaserProjection(
            laser_scan_mm, 360, 1, 0, GMP_ANGLE_STEP_DEG * factor,
            360 / (GMP_ANGLE_STEP_DEG * factor), GMP_map_resolution * factor,
            GMP_laser_extend_offset / factor);
    std::vector<std::vector<cv::Point3i>> laserProjection =
        hCompressLaserProjection2Pixel(rawLaserProjection, cv::Point2f(0, 0));

    // 2.2 iterate ROIs and calculate LMs
    std::vector<std::pair<int16_t, cv::Point>> LMs;
    for (size_t roi_idx = 0; roi_idx < searchROIs[layer_idx].size(); roi_idx++) {
      hUpdateProbMap(map_dilation[layer_idx], prob_map[layer_idx],
                     prob_map_direction[layer_idx],
                     searchROIs[layer_idx][roi_idx], laserProjection,
                     (layer_idx == 0));
      std::vector<std::pair<int16_t, cv::Point>> tmp_LMs;
      hNMS(prob_map[layer_idx], searchROIs[layer_idx][roi_idx], tmp_LMs,
           &(prob_map_nms[layer_idx]));
      hMergeLM(LMs, tmp_LMs, GMP_MAX_CADIDATES[layer_idx]);
    }
#ifdef GMP_DEBUG
    if (layer_idx == 0) {
      cv::Rect roi(cv::Rect(0, 0, prob_map_debug.cols, prob_map_debug.rows));
      hUpdateProbMap(map_dilation[layer_idx], prob_map_debug,
                     prob_map_direction_debug, roi, laserProjection, true);
      std::vector<std::pair<int16_t, cv::Point>> tmp_LMs_debug;
      hNMS(prob_map_debug, roi, tmp_LMs_debug, &prob_map_nms_debug);
      LMs.clear();
      hMergeLM(LMs, tmp_LMs_debug, GMP_MAX_CADIDATES[layer_idx]);
    }
#endif

    // 2.3 calculate lower layer ROI if layer_idx > 0
    // discard LMs that with low score compared to the maximun
    if (LMs.size() > 0) {
      int16_t max_score = LMs[0].first;
      for (size_t i = 1; i < LMs.size(); i++) {
        if (LMs[i].first < max_score * 0.7) {
          LMs.resize(i);
          break;
        }
      }
      if (layer_idx == 0) {
        final_LMs = LMs;
      } else {
        searchROIs[layer_idx - 1].resize(LMs.size());
        for (size_t i = 0; i < LMs.size(); i++) {
          cv::Point tl = cv::Point(LMs[i].second.x * GMP_LAYER_SCALING_FACTOR -
                                       GMP_LOWER_LAYER_SEARCH_RADIUS,
                                   LMs[i].second.y * GMP_LAYER_SCALING_FACTOR -
                                       GMP_LOWER_LAYER_SEARCH_RADIUS);
          cv::Point br = cv::Point(LMs[i].second.x * GMP_LAYER_SCALING_FACTOR +
                                       GMP_LOWER_LAYER_SEARCH_RADIUS + 1,
                                   LMs[i].second.y * GMP_LAYER_SCALING_FACTOR +
                                       GMP_LOWER_LAYER_SEARCH_RADIUS + 1);
          if (tl.x < 0)
            tl.x = 0;
          if (tl.y < 0)
            tl.y = 0;
          if (br.x > prob_map[layer_idx].cols * GMP_LAYER_SCALING_FACTOR)
            br.x = prob_map[layer_idx].cols * GMP_LAYER_SCALING_FACTOR;
          if (br.y > prob_map[layer_idx].rows * GMP_LAYER_SCALING_FACTOR)
            br.y = prob_map[layer_idx].rows * GMP_LAYER_SCALING_FACTOR;

          searchROIs[layer_idx - 1][i] = cv::Rect(tl, br);
        }
      }
    }
    // printf("GMPMatch layer %d cost %d us\n", layer_idx,
    // (uint32_t)(getTimeStap_us() - tmp_t0_us));
  }

  // 3. refine maximum LM, 1 degree angle resolution
  for (size_t i = 0; i < final_LMs.size(); i++) {
    int16_t score = final_LMs[i].first;
    cv::Point pos = final_LMs[i].second;
    int angle = prob_map_direction->at<uchar>(pos) * GMP_ANGLE_STEP_DEG;
    std::vector<std::vector<cv::Point2f>> rawLaserProjection =
        hGetRawLaserProjection(laser_scan_mm, 360, 1, angle - 1, 1, 3,
                               GMP_map_resolution, GMP_laser_extend_offset);
    std::vector<std::vector<cv::Point3i>> laserProjection =
        hCompressLaserProjection2Pixel(rawLaserProjection, cv::Point2f(0, 0));
    prob_map[0].at<int16_t>(pos) = 0;
    hUpdateProbMap(map_dilation[0], prob_map[0], prob_map_direction[0],
                   cv::Rect(pos, pos + cv::Point(1, 1)), laserProjection, true);
    if (prob_map[0].at<int16_t>(pos) > score) {
      score = prob_map[0].at<int16_t>(pos);
      angle = angle + prob_map_direction[0].at<uchar>(pos) - 1;
    }

#ifdef GMP_DEBUG
    score = final_LMs[i].first;
    pos = final_LMs[i].second;
    angle = prob_map_direction_debug.at<uchar>(pos) * GMP_ANGLE_STEP_DEG;

    prob_map_debug.at<int16_t>(pos) = 0;
    hUpdateProbMap(map_dilation[0], prob_map_debug, prob_map_direction_debug,
                   cv::Rect(pos, pos + cv::Point(1, 1)), laserProjection, true);
    if (prob_map_debug.at<int16_t>(pos) > score) {
      score = prob_map_debug.at<int16_t>(pos);
      angle = angle + prob_map_direction_debug.at<uchar>(pos) - 1;
    }
#endif
    GMP_results_t result;
    result.score = score / 10000.0f;
    result.x = pos.x * GMP_map_resolution + GMP_map_origin_px;
    result.y = pos.y * GMP_map_resolution + GMP_map_origin_py;
    result.theta_deg = angle;

    orResults.push_back(result);
  }
  if (orResults.size() > 10) {
    orResults.resize(10);
  }

#if 0
	if (final_LMs.size() > 0)
	{
		// 3.1 get raw laser
		int16_t* raw_laser_scan_mm = new int16_t[laser_point_cnt];
		ON_SCOPE_EXIT([=]() {delete[]raw_laser_scan_mm; });
		memset(raw_laser_scan_mm, 0, laser_point_cnt * sizeof(int16_t));
		for (int i = 0; i < laser_point_cnt; i++)
		{
			raw_laser_scan_mm[i] = laser_scan[i] * 1000;
			if (raw_laser_scan_mm[i] >= (GMP_LASER_RADIUS_METER * 1000))
			{
				raw_laser_scan_mm[i] = 0;
			}
		}

		// 3.2 refine 
		cv::Point refine_candidates = final_LMs[0].second;
		float angle = prob_map_direction[0].at<uchar>(refine_candidates) * GMP_ANGLE_STEP_DEG;

		cv::Point2f refine_pos_meter(0,0);
		float refine_angle_degree = 0;
		int16_t refine_score = 0;

		if (refine_candidates.x > 2 && refine_candidates.y > 2 && 
			(refine_candidates.x < prob_map[0].cols - 2) && (refine_candidates.y < prob_map[0].rows - 2))
		{
			
			float angle_refine_radius = GMP_ANGLE_STEP_DEG / 2;
			float angle_refine_step = laser_angle_resolution_deg;
			int pos_refine_radius_idx = 5 * GMP_map_resolution / GMP_REFINE_POS_STEP_M;

			std::vector<std::vector<cv::Point2f>> rawLaserProjection = hGetRawLaserProjection(raw_laser_scan_mm, laser_point_cnt, laser_angle_resolution_deg, 
				(angle- angle_refine_radius) / laser_angle_resolution_deg, angle_refine_step / laser_angle_resolution_deg, 2 * angle_refine_radius / laser_angle_resolution_deg + 1, 
				GMP_map_resolution, GMP_laser_extend_offset);
			cv::Point2f refine_pos;
			int max_score = final_LMs[0].first;
			for (int vv = -pos_refine_radius_idx; vv <= pos_refine_radius_idx; vv++)
			{
				for (int uu = -pos_refine_radius_idx; uu <= pos_refine_radius_idx; uu++)
				{
					std::vector<std::vector<cv::Point3i>> laserProjection = hCompressLaserProjection2Pixel(rawLaserProjection, cv::Point2f(uu * GMP_REFINE_POS_STEP_M, vv * GMP_REFINE_POS_STEP_M));
					for (int rotation_idx = 0; rotation_idx < laserProjection.size(); rotation_idx++)
					{
						int valid_point_cnt = laserProjection[rotation_idx].size();
						int prob_sum = 0;
						int weight_sum = 0;
						cv::Point3i* pPoint = (cv::Point3i*)laserProjection[rotation_idx].data();
						for (int laser_point_idx = 0; laser_point_idx < valid_point_cnt; laser_point_idx++, pPoint++)
						{
							prob_sum += map_dilation[0].at<uchar>(refine_candidates.y + pPoint->y, refine_candidates.x + pPoint->x) * pPoint->z;
							weight_sum += pPoint->z;
						}
						if (valid_point_cnt > 0)
						{
							float prob = prob_sum / 255.0 / weight_sum;
							int16_t score = prob * 10000;
							if (score > max_score)
							{
								max_score = score;
								refine_score = score;
								refine_pos_meter.x = uu * GMP_REFINE_POS_STEP_M;
								refine_pos_meter.y = vv * GMP_REFINE_POS_STEP_M;
								refine_angle_degree = rotation_idx * angle_refine_step - angle_refine_radius;
							}
						}
					}
				}
			}
			
		}

		GMP_results_t result;
		if (refine_score > 0)
		{
			result.score = refine_score / 10000.0f;
			printf("[%d]refine info is %f, %f, %f. %d -> %d\n", 0, refine_pos_meter.x, refine_pos_meter.y, refine_angle_degree, final_LMs[0].first, refine_score);
		}
		else
		{
			result.score = final_LMs[0].first / 10000.0f;
			printf("[%d] no refine\n",0);
		}
		result.x = final_LMs[0].second.x * GMP_map_resolution + GMP_map_origin_px + refine_pos_meter.x;
		result.y = final_LMs[0].second.x * GMP_map_resolution + GMP_map_origin_py + refine_pos_meter.y;
		result.theta_deg = angle + refine_angle_degree;
		orResults.push_back(result);
		
	}
#endif
  // printf("GMPMatch refine cost %d us\n", (uint32_t)(getTimeStap_us() -
  // refine_start_ts_us));

  printf("GMPMatch total cost %d us\n", (uint32_t)(getTimeStap_us() - t0));

  return 0;
}

int GMPDrawDebugImage(char *file_path, float *laser_scan, int laser_point_cnt,
                      float laser_angle_resolution_deg,
                      GMP_results_t robot_pos) {
  cv::Mat img;
  cv::cvtColor(map_display[0], img, cv::COLOR_GRAY2BGR);

  for (int i = 0; i < laser_point_cnt; i++) {
    if (laser_scan[i] > 0) {
      float angle = robot_pos.theta_deg + i * laser_angle_resolution_deg;
      normalise_angle_degree(angle);
      int u = 0.5f +
              (robot_pos.x + laser_scan[i] * std::cos(angle * CV_PI / 180.0) -
               GMP_map_origin_px) /
                  GMP_map_resolution;
      int v = 0.5f +
              (robot_pos.y + laser_scan[i] * std::sin(angle * CV_PI / 180.0) -
               GMP_map_origin_py) /
                  GMP_map_resolution;
      cv::circle(img, cv::Point(u, v), 1, cv::Scalar(0, 0, 255), -1);
    }
  }

  cv::imwrite(std::string(file_path) + ".png", img);
  return 0;
}