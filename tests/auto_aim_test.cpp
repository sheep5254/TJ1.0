// #include <fmt/core.h>
//
// #include <chrono>
// #include <fstream>
// #include <nlohmann/json.hpp>
// #include <opencv2/opencv.hpp>
//
// #include "tasks/auto_aim/aimer.hpp"
// #include "tasks/auto_aim/solver.hpp"
// #include "tasks/auto_aim/tracker.hpp"
// #include "tasks/auto_aim/yolo.hpp"
// #include "tools/exiter.hpp"
// #include "tools/img_tools.hpp"
// #include "tools/logger.hpp"
// #include "tools/math_tools.hpp"
// #include "tools/plotter.hpp"
//
// const std::string keys =
//   "{help h usage ? |                   | 输出命令行参数说明 }"
//   "{config-path c  | configs/demo.yaml | yaml配置文件的路径}"
//   "{start-index s  | 0                 | 视频起始帧下标    }"
//   "{end-index e    | 0                 | 视频结束帧下标    }"
//   "{@input-path    | assets/demo/demo  | avi和txt文件的路径}";
//
// int main(int argc, char * argv[])
// {
//   // 读取命令行参数
//   cv::CommandLineParser cli(argc, argv, keys);
//   if (cli.has("help")) {
//     cli.printMessage();
//     return 0;
//   }
//   auto input_path = cli.get<std::string>(0);
//   auto config_path = cli.get<std::string>("config-path");
//   auto start_index = cli.get<int>("start-index");
//   auto end_index = cli.get<int>("end-index");
//
//   tools::Plotter plotter;
//   tools::Exiter exiter;
//
//   auto video_path = fmt::format("{}.avi", input_path);
//   auto text_path = fmt::format("{}.txt", input_path);
//   cv::VideoCapture video(video_path);
//   std::ifstream text(text_path);
//
//   auto_aim::YOLO yolo(config_path);
//   auto_aim::Solver solver(config_path);
//   auto_aim::Tracker tracker(config_path, solver);
//   auto_aim::Aimer aimer(config_path);
//
//   cv::Mat img, drawing;
//   auto t0 = std::chrono::steady_clock::now();
//
//   auto_aim::Target last_target;
//   io::Command last_command;
//   double last_t = -1;
//
//   video.set(cv::CAP_PROP_POS_FRAMES, start_index);
//   for (int i = 0; i < start_index; i++) {
//     double t, w, x, y, z;
//     text >> t >> w >> x >> y >> z;
//   }
//
//   for (int frame_count = start_index; !exiter.exit(); frame_count++) {
//     if (end_index > 0 && frame_count > end_index) break;
//
//     video.read(img);
//     if (img.empty()) break;
//
//     double t, w, x, y, z;
//     text >> t >> w >> x >> y >> z;
//     auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));
//
//     /// 自瞄核心逻辑
//
//     solver.set_R_gimbal2world({w, x, y, z});
//
//     auto yolo_start = std::chrono::steady_clock::now();
//     auto armors = yolo.detect(img, frame_count);
//
//     auto tracker_start = std::chrono::steady_clock::now();
//     auto targets = tracker.track(armors, timestamp);
//
//     auto aimer_start = std::chrono::steady_clock::now();
//     auto command = aimer.aim(targets, timestamp, 27, false);
//
//     if (
//       !targets.empty() && aimer.debug_aim_point.valid &&
//       std::abs(command.yaw - last_command.yaw) * 57.3 < 2)
//       command.shoot = true;
//
//     if (command.control) last_command = command;
//     /// 调试输出
//
//     auto finish = std::chrono::steady_clock::now();
//     tools::logger()->info(
//       "[{}] yolo: {:.1f}ms, tracker: {:.1f}ms, aimer: {:.1f}ms", frame_count,
//       tools::delta_time(tracker_start, yolo_start) * 1e3,
//       tools::delta_time(aimer_start, tracker_start) * 1e3,
//       tools::delta_time(finish, aimer_start) * 1e3);
//
//     tools::draw_text(
//       img,
//       fmt::format(
//         "command is {},{:.2f},{:.2f},shoot:{}", command.control, command.yaw * 57.3,
//         command.pitch * 57.3, command.shoot),
//       {10, 60}, {154, 50, 205});
//
//     Eigen::Quaternion gimbal_q = {w, x, y, z};
//     tools::draw_text(
//       img,
//       fmt::format(
//         "gimbal yaw{:.2f}", (tools::eulers(gimbal_q.toRotationMatrix(), 2, 1, 0) * 57.3)[0]),
//       {10, 90}, {255, 255, 255});
//
//     nlohmann::json data;
//
//     // 装甲板原始观测数据
//     data["armor_num"] = armors.size();
//     if (!armors.empty()) {
//       const auto & armor = armors.front();
//       data["armor_x"] = armor.xyz_in_world[0];
//       data["armor_y"] = armor.xyz_in_world[1];
//       data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
//       data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
//       data["armor_center_x"] = armor.center_norm.x;
//       data["armor_center_y"] = armor.center_norm.y;
//     }
//
//     Eigen::Quaternion q{w, x, y, z};
//     auto yaw = tools::eulers(q, 2, 1, 0)[0];
//     data["gimbal_yaw"] = yaw * 57.3;
//     data["cmd_yaw"] = command.yaw * 57.3;
//     data["shoot"] = command.shoot;
//
//     if (!targets.empty()) {
//       auto target = targets.front();
//
//       if (last_t == -1) {
//         last_target = target;
//         last_t = t;
//         continue;
//       }
//
//       std::vector<Eigen::Vector4d> armor_xyza_list;
//
//       // 当前帧target更新后
//       armor_xyza_list = target.armor_xyza_list();
//       for (const Eigen::Vector4d & xyza : armor_xyza_list) {
//         auto image_points =
//           solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
//         tools::draw_points(img, image_points, {0, 255, 0});
//       }
//
//       // aimer瞄准位置
//       auto aim_point = aimer.debug_aim_point;
//       Eigen::Vector4d aim_xyza = aim_point.xyza;
//       auto image_points =
//         solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
//       if (aim_point.valid) tools::draw_points(img, image_points, {0, 0, 255});
//
//       // 观测器内部数据
//       Eigen::VectorXd x = target.ekf_x();
//       data["x"] = x[0];
//       data["vx"] = x[1];
//       data["y"] = x[2];
//       data["vy"] = x[3];
//       data["z"] = x[4];
//       data["vz"] = x[5];
//       data["a"] = x[6] * 57.3;
//       data["w"] = x[7];
//       data["r"] = x[8];
//       data["l"] = x[9];
//       data["h"] = x[10];
//       data["last_id"] = target.last_id;
//
//       // 卡方检验数据
//       data["residual_yaw"] = target.ekf().data.at("residual_yaw");
//       data["residual_pitch"] = target.ekf().data.at("residual_pitch");
//       data["residual_distance"] = target.ekf().data.at("residual_distance");
//       data["residual_angle"] = target.ekf().data.at("residual_angle");
//       data["nis"] = target.ekf().data.at("nis");
//       data["nees"] = target.ekf().data.at("nees");
//       data["nis_fail"] = target.ekf().data.at("nis_fail");
//       data["nees_fail"] = target.ekf().data.at("nees_fail");
//       data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");
//     }
//
//     plotter.plot(data);
//
//     cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
//     cv::imshow("reprojection", img);
//     auto key = cv::waitKey(30);
//     if (key == 'q') break;
//   }
//
//   return 0;
// }

#include <fmt/core.h>
#include <fstream>  // 添加这个头文件
#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include "tasks/auto_aim/solver.hpp"
#include "io/camera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "yaml-cpp/yaml.h"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明}"
  "{config-path c  | configs/camera.yaml | yaml配置文件路径}"
  "{d display      |                     | 显示视频流       }";


int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;

  auto config_path = cli.get<std::string>("config-path");
  auto display = cli.has("display");

  // 在初始化组件之前添加详细的错误处理
  try {

    std::cout << "All string fields are valid!" << std::endl;

    tools::logger()->info("Starting auto_aim_test with config: {}", config_path);

    // 检查配置文件
    std::ifstream config_file(config_path);
    if (!config_file.good()) {
      tools::logger()->error("Config file not found: {}", config_path);
      return -1;
    }
    config_file.close();

    tools::logger()->info("Step 1: Initializing camera...");
    io::Camera camera(config_path);

    tools::logger()->info("Step 2: Initializing YOLO...");
    auto_aim::YOLO yolo(config_path);

    tools::logger()->info("Step 3: Initializing Solver...");
    auto_aim::Solver solver(config_path);

    tools::logger()->info("Step 4: Initializing Tracker...");
    auto_aim::Tracker tracker(config_path, solver);

    tools::logger()->info("Step 5: Initializing Aimer...");
    auto_aim::Aimer aimer(config_path);

    tools::logger()->info("All components initialized successfully!");

    // 从这里开始是原有的主循环代码
    cv::Mat img(720, 540, CV_8UC3); // 注意：参数顺序是(行, 列, 类型)
    std::chrono::steady_clock::time_point timestamp;
    auto last_stamp = std::chrono::steady_clock::now();

    // 自瞄相关的状态变量
    auto_aim::Target last_target;
    io::Command last_command;
    int frame_count = 0;

    // 固定云台姿态（在没有真实IMU数据的情况下使用）
    Eigen::Quaterniond fixed_quat(1.0, 0.0, 0.0, 0.0); // w=1, x=0, y=0, z=0

    while (!exiter.exit()) {
      // 从相机读取一帧
      camera.read(img, timestamp);
      if (img.cols != 720 || img.rows != 540) {
        cv::resize(img, img, cv::Size(720,540 ));
      }

      // 检查图像是否为空来判断读取是否成功
      if (img.empty()) {
        tools::logger()->warn("Failed to read from camera - empty image");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      auto current_frame_count = frame_count++;

      /// 自瞄核心逻辑开始

      // 设置云台姿态（这里使用固定值，在实际应用中应从IMU获取）
      solver.set_R_gimbal2world({
        fixed_quat.w(),
        fixed_quat.x(),
        fixed_quat.y(),
        fixed_quat.z()
      });

      // 第一步：YOLO目标检测
      auto yolo_start = std::chrono::steady_clock::now();//
      auto armors = yolo.detect(img, current_frame_count);

      auto detection1 = img.clone();
      if (!armors.empty()) {
        armors.sort([](const auto & a, const auto & b) {
        cv::Point2f img_center(720 / 2, 540 / 2);  // TODO
        auto distance_1 = cv::norm(a.center - img_center);
        auto distance_2 = cv::norm(b.center - img_center);
        return distance_1 < distance_2;
      });

      // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
      armors.sort(
        [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });
      auto & armor = armors.front();  // <-- 直接选择列表的第一个（优先级最高的）
      // 绘制装甲板四边形
      for (int i = 0; i < 4; i++) {
        int j = (i + 1) % 4;
        std::cout<<"第"<<i<<"个点:"<<armor.points[i]<<std::endl;
        cv::line(detection1, armor.points[i], armor.points[j],
                 cv::Scalar(0, 0, 255), 2);

        // 绘制角点
        cv::circle(detection1, armor.points[i], 5, cv::Scalar(0, 0, 255), -1);

        // 标注点序号
        cv::putText(detection1, std::to_string(i),
                   cv::Point(armor.points[i].x + 5, armor.points[i].y - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
      }
      imshow("detection1",detection1);

      // 首先确认检测到的点的实际位置
      // 打印每个点的坐标来判断顺序
      std::cout << "=== 检测到的角点坐标 ===" << std::endl;
      for (int i = 0; i < 4; i++) {
        std::cout << "points[" << i << "]: (" << armor.points[i].x << ", " << armor.points[i].y << ")" << std::endl;
      }

      // 分析点的位置，找出正确的对应关系
      // 根据x坐标判断左右，根据y坐标判断上下
      std::vector<cv::Point2f> sorted_points(4);
      std::vector<cv::Point2f> pts = armor.points;  // 直接复制vector

      // 找出左侧两个点和右侧两个点
      std::sort(pts.begin(), pts.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
      });

      // 左侧两个点
      cv::Point2f left_top, left_bottom;
      if (pts[0].y < pts[1].y) {
        left_top = pts[0];
        left_bottom = pts[1];
      } else {
        left_top = pts[1];
        left_bottom = pts[0];
      }

      // 右侧两个点
      cv::Point2f right_top, right_bottom;
      if (pts[2].y < pts[3].y) {
        right_top = pts[2];
        right_bottom = pts[3];
      } else {
        right_top = pts[3];
        right_bottom = pts[2];
      }

      // 3D点顺序：左上、右上、右下、左下
      std::vector<cv::Point3f> object_points;
      float armor_half_width = 67.5f;   // 小装甲板半宽 mm
      float armor_half_height = 28.0f;  // 小装甲板半高 mm

      object_points.push_back(cv::Point3f(-armor_half_width, -armor_half_height, 0.f));  // 左上
      object_points.push_back(cv::Point3f(armor_half_width, -armor_half_height, 0.f));   // 右上
      object_points.push_back(cv::Point3f(armor_half_width, armor_half_height, 0.f));    // 右下
      object_points.push_back(cv::Point3f(-armor_half_width, armor_half_height, 0.f));   // 左下

      // 2D点按照相同顺序：左上、右上、右下、左下
      std::vector<cv::Point2f> image_points;
      image_points.push_back(left_top);      // 左上
      image_points.push_back(right_top);     // 右上
      image_points.push_back(right_bottom);  // 右下
      image_points.push_back(left_bottom);   // 左下


      cv::Mat rvec, tvec;

      cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
            901.628836608727,    0.0,                 368.99147138139285,
            0.0,                 903.1485486250545,   281.9078681333959,
            0.0,                 0.0,                 1.0);

      cv::Mat distortion_coeffs = (cv::Mat_<double>(5, 1) <<
            -0.447569535375274,
            -0.000446903384808,
             0.000278897924377,
             0.001563208302172,
             0.0);

        std::cout << "图像分辨率: " << img.cols << "x" << img.rows << std::endl;


      cv::solvePnP(object_points, image_points, camera_matrix, distortion_coeffs,
                   rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

      double distance = cv::norm(tvec);
      std::cout << "距离: " << distance << " mm" << std::endl;

      }  // end of if (!armors.empty())
      
      cv::waitKey(1);
    }  // end of while
    
  } catch (const YAML::Exception& e) {
    std::cerr << "YAML Exception: " << e.what() << std::endl;
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }
  
  return 0;
}

//  ./build/auto_aim_test -c=configs/demo.yaml