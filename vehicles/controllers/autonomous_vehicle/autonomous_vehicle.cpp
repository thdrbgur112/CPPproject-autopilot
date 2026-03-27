#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/vehicle/Driver.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <sys/stat.h> // mkdir 사용을 위해 추가

using namespace std;
using namespace webots;

// 상수 설정
const int TIME_STEP = 50;
const int SAVE_INTERVAL = 20;
const double LANE_OFFSET = 1.9; // 중앙선에서 차로 정중앙까지의 거리

struct Point {
    double x, z;
};

class AutonomousController {
private:
    Driver *driver;
    Camera *camera;
    GPS *gps;
    Compass *compass;

    vector<Point> road_points;
    string base_path = "/Users/baggyeongsu/Documents/vehicles/dataset";
    int image_count = 0;
    int step_counter = 0;

public:
    AutonomousController() {
        driver = new Driver();

        camera = driver->getCamera("camera");
        camera->enable(TIME_STEP);

        gps = driver->getGPS("gps");
        gps->enable(TIME_STEP);

        compass = driver->getCompass("compass");
        compass->enable(TIME_STEP);

        // 데이터셋 저장을 위한 폴더 생성
        mkdir("/Users/baggyeongsu/Documents/vehicles/dataset", 0777);
        mkdir("/Users/baggyeongsu/Documents/vehicles/dataset/images", 0777);
    }

    ~AutonomousController() {
        delete driver;
    }

    // 지도 로드 함수
    void load_road_map(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "⚠️ road_map.txt 로드 실패! 경로를 확인하세요." << endl;
            return;
        }

        road_points.clear();
        string line;
        while (getline(file, line)) {
            size_t comma = line.find(',');
            if (comma != string::npos) {
                try {
                    double x = stod(line.substr(0, comma));
                    double z = stod(line.substr(comma + 1));
                    road_points.push_back({x, z});
                } catch (...) { continue; }
            }
        }
        cout << "✅ 지도 로드 완료 (" << road_points.size() << "개 좌표)" << endl;
    }

    void run() {
        while (driver->step() != -1) {
            const unsigned char *image = camera->getImage();
            const double *pos = gps->getValues();
            const double *com = compass->getValues();

            // 센서 초기화 대기
            if (isnan(pos[0]) || isnan(com[0])) continue;

            // --- PART 1: 라벨링 로직 (데이터 산출) ---
            double yaw = atan2(com[0], com[2]);

            int idx = 0;
            double min_d = 1e10;
            if (road_points.empty()) continue;

            for (size_t i = 0; i < road_points.size(); i++) {
                double d = sqrt(pow(pos[0] - road_points[i].x, 2) + pow(pos[2] - road_points[i].z, 2));
                if (d < min_d) { min_d = d; idx = i; }
            }

            int n_idx = (idx + 1) % road_points.size();
            double road_angle = atan2(road_points[n_idx].x - road_points[idx].x,
                                      road_points[n_idx].z - road_points[idx].z);

            double target_x = road_points[idx].x + cos(road_angle) * LANE_OFFSET;
            double target_z = road_points[idx].z - sin(road_angle) * LANE_OFFSET;
            double dx = target_x - pos[0];
            double dz = target_z - pos[2];

            // 차량 기준 로컬 Y 좌표 계산
            double label3_current_center_y = dx * sin(-yaw) + dz * cos(-yaw);
            double label1_lateral_offset = label3_current_center_y;
            double label2_heading_error = road_angle - yaw;

            while (label2_heading_error > M_PI) label2_heading_error -= 2 * M_PI;
            while (label2_heading_error < -M_PI) label2_heading_error += 2 * M_PI;


            // --- PART 2: 주행 제어 (기존 로직) ---
            control_vehicle(image);


            // --- PART 3: 데이터 저장 ---
            if (++step_counter >= SAVE_INTERVAL) {
                save_data(label1_lateral_offset, label2_heading_error, label3_current_center_y);
                step_counter = 0;
                if (image_count >= 30000) break;
            }
        }
    }

private:
    void control_vehicle(const unsigned char *image) {
        int width = camera->getWidth();
        int height = camera->getHeight();
        int left_lane_pos = -1, right_edge_pos = -1;
        int scan_line = (int)(height * 0.75);

        for (int x = 0; x < width; x++) {
            int r = Camera::imageGetRed(image, width, x, scan_line);
            int g = Camera::imageGetGreen(image, width, x, scan_line);
            int b = Camera::imageGetBlue(image, width, x, scan_line);

            if (left_lane_pos == -1 && r > 170 && g > 170 && b > 170) left_lane_pos = x;
            if (left_lane_pos != -1 && right_edge_pos == -1 && x > left_lane_pos + 40) {
                if (g > (r + 25) && g > 60) right_edge_pos = x;
            }
        }

        double steering = 0.0;
        if (left_lane_pos != -1 && right_edge_pos != -1) {
            int target = (left_lane_pos + right_edge_pos) / 2 + 10;
            steering = ((double)target - (width / 2.0)) * 0.004;
        } else if (left_lane_pos != -1) {
            steering = ((double)left_lane_pos + 200 - (width / 2.0)) * 0.003;
        }

        driver->setSteeringAngle(steering);
        driver->setCruisingSpeed(23.0);
    }

    void save_data(double offset, double angle, double center_y) {
        ofstream csv(base_path + "/labels.csv", ios::app);
        if (csv.is_open()) {
            stringstream ss;
            ss << "frame_" << setw(5) << setfill('0') << image_count << ".jpg";
            string filename = ss.str();

            csv << filename << "," << fixed << setprecision(4) 
                << offset << "," << angle << "," << center_y << "\n";
            csv.close();

            string img_path = base_path + "/images/" + filename;
            camera->saveImage(img_path, 80);

            cout << "[SAVE] #" << setw(5) << setfill('0') << image_count 
                 << " | Offset:" << fixed << setprecision(2) << offset 
                 << " | Angle:" << angle << " | LaneCenter:" << center_y << endl;
            image_count++;
        }
    }
};

int main(int argc, char **argv) {
    AutonomousController controller;
    controller.load_road_map("/Users/baggyeongsu/Documents/vehicles/road_map.txt");
    controller.run();
    return 0;
}