/*
 * ================================================================================
 * 파일명: autonomous_data_collector.cpp
 * 작성자: 박경수 (AI소프트웨어학부 2학년)
 *
 * [프로그램 개요]
 * Webots 시뮬레이터 환경에서 제공된 도로 지도(road_map.txt)를 기반으로,
 * 자율주행 차량이 차선을 따라 주행하며 딥러닝 학습용 데이터를 수집하는 프로그램입니다.
 * 
 * [수집 및 저장 데이터 (CSV)]
 * - A열: 카메라 이미지 파일명 (예: frame_0.jpg)
 * - B열: 횡방향 오차 (차량이 차선 중앙에서 좌우로 이탈한 정도, 단위: m)
 * - C열: 헤딩 오차 (차량의 진행 방향과 도로의 방향 차이, 단위: rad)
 * - D열~: 차량 전방 1~10m 도로 중심선의 상대 좌표 [(좌우거리 : 전후거리)]
 * ================================================================================
 */

#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <webots/vehicle/Driver.hpp>
#include <webots/GPS.hpp>

#include <iostream>   // 콘솔 입출력 (cout으로 진행상황 출력)
#include <vector>     // 동적 배열 (지도 좌표, 경로 샘플 저장에 사용)
#include <fstream>    // 파일 입출력 (road_map.txt 읽기, labels.csv 쓰기)
#include <string>     // 문자열 처리 (파일 경로, 파일명 조합에 사용)
#include <cmath>      // 수학 함수 (sqrt: 제곱근, pow: 거듭제곱, atan2: 각도 변환)
#include <iomanip>    // 출력 형식 설정 (setprecision으로 CSV 소수점 자릿수 고정)
#include <sys/stat.h> // 운영체제 파일 시스템 함수 (mkdir로 저장 폴더 생성)

using namespace std;
using namespace webots;

// 지도의 좌표 한 점(X, Y)을 담기 위한 구조체입니다.
struct Point { double x, y; };

// ================================================================================
// 전역 상수 (프로그램 설정값)
// ================================================================================
const int    TIME_STEP      = 10;   // 시뮬레이션 주기 (10ms)
const int    SAMPLE_COUNT   = 10;   // 차량 앞쪽으로 추출할 경로 샘플 점의 개수
const double SAMPLE_DIST    = 1.0;  // 샘플 점 사이의 간격 (1m)
const double MIN_MOVE_DIST  = 0.3;  // 최소 이동 거리 (0.3m 이상 움직였을 때만 데이터 저장)


class AutonomousController {
public:
    Driver *driver;
    Camera *camera;
    GPS    *gps;

    AutonomousController() {
        driver = new Driver();
        camera = driver->getCamera("camera"); camera->enable(TIME_STEP);
        gps    = driver->getGPS("gps");       gps->enable(TIME_STEP);
    }
    ~AutonomousController() { delete driver; }

   
    void do_driving() {
        const unsigned char *image = camera->getImage();
        int w = camera->getWidth(); int h = camera->getHeight();
        int scan_y = (int)(h * 0.75);
        int left_edge = -1; int right_edge = -1;

        for (int x = w/2; x > 10; x--) {
            if (Camera::imageGetRed(image, w, x, scan_y) > 150) { left_edge = x; break; }
        }
        for (int x = w/2; x < w - 10; x++) {
            if (Camera::imageGetRed(image, w, x, scan_y) > 150) { right_edge = x; break; }
        }

        double steering = 0.0;
        if (left_edge != -1 && right_edge != -1) steering = ((double)(left_edge + right_edge) / 2.0 - (w / 2.0)) * 0.008;
        else if (left_edge != -1) steering = ((double)left_edge + 145 - (w / 2.0)) * 0.006;
        else if (right_edge != -1) steering = ((double)right_edge - 145 - (w / 2.0)) * 0.006;

        if (steering >  0.5) steering =  0.5;
        if (steering < -0.5) steering = -0.5;
        
        driver->setSteeringAngle(steering);
        driver->setCruisingSpeed(14.0); 
    }
};


/*
 * ================================================================================
 * 클래스: DataCollector (학습 데이터 수집기)
 * 
 * [역할]
 * 1. Webots에서 추출된 도로 맵(road_map.txt)을 메모리에 불러옵니다.
 * 2. 차량이 이동할 때마다 GPS 좌표를 기반으로 횡방향 오차, 헤딩 오차, 경로 샘플을
 *    산출하여 이미지와 함께 저장합니다.
 * ================================================================================
 */
class DataCollector {
private:
    string my_save_folder, my_image_folder, my_csv_path, my_map_path;
    vector<Point> my_map; // 불러온 지도 데이터를 담는 배열

    int    my_file_count; // 지금까지 저장한 데이터 갯수
    double last_save_x;   // 이전 저장 시점의 X 좌표 (주행 벡터 계산용)
    double last_save_y;   // 이전 저장 시점의 Y 좌표 (주행 벡터 계산용)

public:
    //생성자 
    DataCollector(string folder_path, string map_path) {
        my_save_folder  = folder_path;
        my_image_folder = folder_path + "/images";
        my_csv_path     = folder_path + "/labels.csv";
        my_map_path     = map_path;
        my_file_count   = 0; //사진개수
        last_save_x     = 99999.0; // 초기 상태를 의미하는 임의의 큰 값
        last_save_y     = 99999.0;
        
        mkdir(my_save_folder.c_str(),  0777); // 폴더 만들기 0777통해 권한부여
        mkdir(my_image_folder.c_str(), 0777);
    }

    /*
     * --------------------------------------------------------------------------------
     * 1. 도로 지도 로드 함수 (load_road_map)
     * Webots에서 추출한 'road_map.txt' 파일을 읽어와 my_map 배열에 넣습니다.
     * --------------------------------------------------------------------------------
     */
    void load_road_map() {
        ifstream my_file(my_map_path); //파일 연결
        double x, y; //gps에서 가져온 위치
        char comma;
        // 중간 콤마를 제거하기위한 코드
        // my_file >> x >> comma >> y 파일에서 데이터를 빨아들여(>>) 각각의 변수에 넣습니다
        // 파일에 1.5,3.2라고 적혀있다면, x에는 1.5, comma에는 쉼표(,), y에는 3.2가  들어갑니다
        while (my_file >> x >> comma >> y) {
            my_map.push_back({x, y});
            //push_back를 통해 my_map(불러온 지도 데이터를 담는 배열)에 (x, y) 형태로 저장합니다
        }
        cout << "Webots 지도 불러오기 완료: 총 " << my_map.size() << "개 포인트" << endl;
    }

    /*
     * --------------------------------------------------------------------------------
     * 2. 순환 인덱스 계산 함수 (get_valid_idx)
     * 처음에 이코드가 없었는데 도로의 끝부분이나 시작부분에 도달하는 순간 프로그램이 종료되어서 추가했음
     * 지도가 원형 트랙일 때 배열 인덱스가 범위를 벗어나지 않도록 만듭니다
     * (예: 총 100칸일 때, 101번 인덱스는 1번으로, -1번 인덱스는 99번으로 변환)
     * (100+1)%100 =1 자료구조 수업에서 배운 circular queue 구현방식 아이디어를 참조했습니다
     *                               (front+1+)%capacity
     * --------------------------------------------------------------------------------
     */
    int get_valid_idx(int idx) {
        int sz = (int)my_map.size();
        if (sz == 0) return 0;
        return (idx % sz + sz) % sz; 
    }

    /*
     * --------------------------------------------------------------------------------
     * 3. 헤딩 오차 계산 함수 (calc_heading_error)
     * 차량 진행 각도와 도로 방향 각도를 뺀 뒤,
     *  값이 튀지 않게 보정 << Ai 활용했습니다
     * 처음에 이코드가 없었으나 게속 수집했던 값이 +180도와 -180도 경계에서 수치가 급변하는 오류 발생하였고
     * 수정을 하는부분에 ai에 도움을 받았습니다. 오류가 났던이유는 atan2 함수가 각도를 -180도(-π)에서 +180도(π) 
     * 사이로만 반환하기 때문입니다. 만약 도로가 +170도를 향하고 내 차가 -170도를 향할 때 
     * 단순 뺄셈을 하면 실제 물리적 차이는 20도임에도 불구하고 오차가 340도로 잘못 계산됩니다.
     *  이로 인해 차량이 핸들을 급격하게 꺾어버리는 각도 점프 현상이 발생했기 때문입니다.
     * 
     * 
     * fwd_x fwd_y 내차 앞범퍼가 가리키는 방향 단위백터 
     * closest 현재 차량 위치 
     * my_map[closest] 내 차와 가장 가까운 '지도 중앙선 위에 찍혀있는 가상의 점
     * p_dir 경로 방향 :  정방향 1 역방향 -1 이 변수가 있는이뉴는 데이터 수집시에 
     * 차량을 180도 돌려서 수집을 한번 더 하기 때문에 존재
     * 
     * --------------------------------------------------------------------------------
     */
    double calc_heading_error(double fwd_x, double fwd_y, int closest, int p_dir) {
        // 도로 방향 확인을 위해 현재 위치에서 5칸 앞의 점을 확인합니다
        // 한칸앞은 너무 가까워 도로의 전체적인 흐름 방향을 더욱 안정적으로 읽기위해서 5m앞으로 설정했습니다.
        int n_idx = get_valid_idx(closest + p_dir * 5); //내 위치 + 5칸앞의 인덱스
        double r_dx = my_map[n_idx].x - my_map[closest].x;
        double r_dy = my_map[n_idx].y - my_map[closest].y;
        //my_map에서 n_idx번째 서랍장을 꺼낸 다음(.), 그 안에 있는 x칸의 값만 가져오기
        //(5칸 앞 좌표 - 현재 위치 좌표) 하여 r_dx,r_dy백터를 만듦

        if (abs(r_dx) < 1e-7 && abs(r_dy) < 1e-7) return 0.0;
        //오류방지용 코드 0.0000001보다 작은값은 0처리 이부분도 ai 활용 받았습니다

        // atan2 함수: X, Y 벡터를 라디안 각도로 변환해줍니다 (-π ~ π 범위)
        double c_ang = atan2(fwd_y, fwd_x);  // 내 차량의 각도
        double r_ang = atan2(r_dy,  r_dx);   // 도로의 각도
        double err = r_ang - c_ang;          // 두 각도의 차이=헤딩각도 오차

        // atan2 함수는 각도를 반드시 -π(-180도) ~ +π(+180도) 사이로만 반환합니다.
        // 그런데 두 각도를 뺄셈하면 이 범위를 벗어나는 값이 나올 수 있습니다.
        //
        // 예시)
        //  내 차량 각도(c_ang) = -170도 (거의 왼쪽을 향함)
        //  도로 방향 각도(r_ang) = +170도 (거의 오른쪽을 향함)
        //
        //  단순 뺄셈: err = 170 - (-170) = +340도
        //
        //  하지만 실제로 두 방향의 차이는 20도에 불과합니다.
        //  (+170도와 -170도는 원 위에서 20도 차이이기 때문)
        //
        //  340도짜리 오차를 그대로 학습 데이터로 쓰면,
        //  모델이 "핸들을 340도만큼 꺾어야 한다"고 잘못 학습하게 됩니다.
        //
        while (err >  M_PI) err -= 2 * M_PI;
        while (err < -M_PI) err += 2 * M_PI;

        return err;
    }

    /*
     * --------------------------------------------------------------------------------
     * 4. 데이터 수집 메인 함수 (do_collect)
     * --------------------------------------------------------------------------------
     */
    void do_collect(Camera* cam, double x, double y) {
        if (my_map.empty()) return;
        if (last_save_x > 90000.0) { last_save_x = x; last_save_y = y; return; }
        //시작시에 시뮬레이션을 켜서 "이전 위치"라는 개념이 없을 때 현재위치 저장하게 끔 오류방지 

        // [A] 수직거리를 이용한 이동 거리 측정
        // 피타고라스 정리를 사용하여 (이전 위치 ~ 현재 위치)의 직선 거리를 구합니다.
        // 이 거리가 0.3m 이상일 때만 데이터를 저장하여, 제자리에서 데이터가 중복되는 것을 막습니다.
        //제자리에서 데이터가 중복되는 것을 막는 부분도 ai 활용 했습니다 처음에 없이 데이터 산출하였는데 
        //전방 좌표값들이 전부 같은 값으로 중복되는 현상을 발견하였고 ai 활용하여 문제원인을 알아내고 코드 수정에 
        //도움을 받았습니다
        //
        double move_dist = sqrt(pow(x - last_save_x, 2) + pow(y - last_save_y, 2));
        if (move_dist < MIN_MOVE_DIST) return;


        // [B] 차량 중심 기준의 방향 벡터 산출
        double fwd_x = (x - last_save_x) / move_dist; // 전방을 향하는 단위 벡터 (현재 위치 - 이전 위치)
        double fwd_y = (y - last_save_y) / move_dist; // move_dist로 나누는이유 단위백터로 나타내기위해
        double rgt_x =  fwd_y;                        // 전방 벡터를 시계방향 90도 회전한 우측 벡터
        double rgt_y = -fwd_x;


        // [C] 지도에서 현재 위치와 가장 가까운 점(인덱스) 찾기
        int closest = 0;
        double min_d = 999999.0;
        for (int i = 0; i < (int)my_map.size(); i++) {
            double d = sqrt(pow(x - my_map[i].x, 2) + pow(y - my_map[i].y, 2));
            if (d < min_d) { min_d = d; closest = i; }
        }
        if (min_d > 15.0) return; // 도로를 크게 이탈하면 저장하지 않음



        // [D] 벡터의 내적(Dot Product)을 이용한 주행 방향 판별
        // [AI 활용 설명] 지도 상 앞쪽 점과 뒤쪽 점을 각각 내 차량의 전방 벡터와 내적합니다.
        // 내적값이 클수록 각도가 일치하는 것이므로, 차가 지도의 역방향으로 도는지 순방향으로 도는지 파악합니다.
        // 처음에 이코드 없이 데이터 산출하였는데 차량을 180도 돌려서 데이터 산출할시에 차량은 앞으로 달리는데, 
        // 추출되는 1~10m 경로 데이터는 차량 뒤쪽의 궤적을 가져와버리는
        // 문제가 발생하여 ai를 활용하여 도움받았습니다

        // [역방향이 발생하는 이유]
        // 더 다양한 학습 데이터를 얻기 위해 차량을 180도 반대로 놓고 데이터를 한 번 더 수집합니다.
        // 이때 p_dir이 없으면, 차는 앞으로 달리지만 경로 샘플은 뒤쪽 위치를 참조하는 문제가 생깁니다.

        // [내적으로 방향 판별하는 원리]
        // 내적(Dot Product) = A벡터 · B벡터 = |A||B|cos(θ)
        // 두 벡터의 방향이 같으면(θ = 0°) 내적이 최대(양수), 반대면(θ = 180°) 최소(음수)가 됩니다.

        // 현재 위치(closest)에서 5칸 앞(idx_f)과 5칸 뒤(idx_b)의 지도 점을
        // 각각 차량 전방 벡터(fwd)와 내적합니다.
        // 내적값이 더 큰 쪽이 차량이 실제로 향하는 방향이므로, 그쪽을 순방향으로 결정합니다.

        int idx_f = get_valid_idx(closest + 5); //" 내 위치 + 5 "의점 순환인덱스로 구하기
        int idx_b = get_valid_idx(closest - 5);
        double ry_f = (my_map[idx_f].x - x) * fwd_x + (my_map[idx_f].y - y) * fwd_y;
        double ry_b = (my_map[idx_b].x - x) * fwd_x + (my_map[idx_b].y - y) * fwd_y;
        int p_dir = (ry_f > ry_b) ? 1 : -1;
        // ry_f(앞쪽 내적값) > ry_b(뒤쪽 내적값) 이면 지도 순방향(+1), 아니면 역방향(-1)

        // [E] [헤딩 오차 (Heading Error)]
        // 차량의 진행 방향과 도로 방향 사이의 각도 차이입니다. (단위: rad)
        // 위에서만든 calc_heading_error을 통해 헤딩오차를 구합니다.


        // [횡방향 오차 (Lateral Error)]
        // 차량이 도로 중심선에서 좌우로 얼마나 벗어났는지를 나타냅니다. (단위: m)
        //
        // [계산 원리: 벡터 투영(Projection)]
        // (가장 가까운 도로 중심점 - 현재 차량 위치) 벡터를 차량의 우측 벡터(rgt)에 투영합니다.
        // 투영값이 양수(+)이면 차량이 도로 중심보다 오른쪽에, 음수(-)이면 왼쪽에 있다는 뜻입니다.
        //
        // 수식: lateral_err = (도로점 - 차량위치) · rgt벡터(차량의 우측 벡터)
        //                   = (Δx * rgt_x) + (Δy * rgt_y)
        double h_err = calc_heading_error(fwd_x, fwd_y, closest, p_dir); // 헤딩 오차
        double lateral_err = (my_map[closest].x - x) * rgt_x + (my_map[closest].y - y) * rgt_y; // 횡방향 오차

        // [F] 벡터 투영(Projection)을 이용한 경로 샘플 상대 좌표 변환
        // [AI 활용 설명] 절대 GPS 좌표(px, py)를 차량이 기준인 (전후 거리, 좌우 거리)로 바꿉니다.
        // 지도의 점을 차량의 우측 벡터(rgt)와 전방 벡터(fwd)에 투영시키는 수학적 원리를 사용했습니다.
        // 차량 전방 1m 간격으로 SAMPLE_COUNT(10)개의 도로 중심점을 추출합니다.
        // GPS 기반의 절대 좌표를 그대로 저장하면 학습 모델이 '지구상 어디에 있는지'를 학습해버리므로
        // 차량을 중심으로 한 상대 좌표 (좌우 거리 rx, 전후 거리 ry)로 변환하여 저장합니다.
        //
        // [상대 좌표 변환 원리: 벡터 투영]
        // 절대 좌표의 점 P를, 차량 기준 축(rgt, fwd)에 각각 투영합니다.
        //   rx = (P - 차량위치) · rgt  →  차량 기준 좌우 거리 (양수: 오른쪽, 음수: 왼쪽)
        //   ry = (P - 차량위치) · fwd  →  차량 기준 전후 거리 (양수: 앞쪽)
        //
        // [샘플 추출 방법]
        // 지도 점과 점 사이를 잇는 선분을 따라 누적 거리(acc)를 쌓아가다가,
        // 목표 거리(1m, 2m, ... 10m)에 도달하는 순간 선형 보간(t값)으로 정확한 좌표를 계산합니다.
        // 선형 보간 공식: P = P1 + (P2 - P1) * t  (t = 목표거리까지 남은 비율)
        vector<pair<double, double>> path;
        double acc = 0.0; // 지금까지 누적된 경로 길이 (m)
        int s_num = 0;    // 지금까지 추출한 샘플 점의 개수

        for (int step = 0; step < (int)my_map.size() - 1 && s_num < SAMPLE_COUNT; step++) {
            // p_dir에 따라 순방향(+) 또는 역방향(-)으로 지도 점을 순서대로 탐색합니다.
            int i1 = get_valid_idx(closest + step * p_dir);
            int i2 = get_valid_idx(closest + (step + 1) * p_dir);

            // 현재 선분(i1 → i2)의 길이를 계산합니다.
            double sd_x = my_map[i2].x - my_map[i1].x;
            double sd_y = my_map[i2].y - my_map[i1].y;
            double slen = sqrt(sd_x * sd_x + sd_y * sd_y);
            if (slen < 1e-6) continue; // 길이가 0에 가까운 선분은 건너뜁니다.

            double p_acc = acc; // 이 선분이 시작하는 시점의 누적 거리
            acc += slen;        // 이 선분을 지나고 난 뒤의 누적 거리

            // 이 선분 안에 1개 이상의 샘플 목표 거리가 포함될 수 있으므로 while로 반복합니다.
            while (s_num < SAMPLE_COUNT) {
                double target = (s_num + 1) * SAMPLE_DIST; // 다음 목표 거리 (1m, 2m, ...)
                if (acc < target) break; // 아직 목표 거리에 못 미치면 다음 선분으로 넘어갑니다.

                // [선형 보간] 선분 위에서 목표 거리에 해당하는 정확한 좌표를 구합니다.
                // t: 이 선분의 시작점(p_acc)에서 목표 지점(target)까지의 비율 (0.0 ~ 1.0)
                double t  = (target - p_acc) / slen;
                double px = my_map[i1].x + sd_x * t; // 보간된 절대 X 좌표
                double py = my_map[i1].y + sd_y * t; // 보간된 절대 Y 좌표

                // [절대 좌표 → 차량 기준 상대 좌표 변환]
                double rx = (px - x) * rgt_x + (py - y) * rgt_y; // 좌우 거리
                double ry = (px - x) * fwd_x + (py - y) * fwd_y; // 전방 거리
                path.push_back({rx, ry});
                s_num++;
            }
        }


        // ============================================================
        // [G단계] CSV 라벨 파일 및 이미지 저장
        // ============================================================
        // ios::app 모드로 열어 기존 내용 뒤에 한 줄씩 이어서 씁니다.
        // CSV 한 행의 형식:
        //   frame_N.jpg, 횡방향오차, 헤딩오차, (rx1:ry1), (rx2:ry2), ... , (rx10:ry10)
        ofstream csv(my_csv_path, ios::app);
        csv << "frame_" << my_file_count << ".jpg,"
            << fixed << setprecision(4) << lateral_err << "," << h_err;
        for (auto& p : path) csv << ",(" << p.first << ":" << p.second << ")";
        csv << "\n";

        // 카메라 이미지를 JPEG 파일로 저장합니다. (품질 85%)
        cam->saveImage(my_image_folder + "/frame_" + to_string(my_file_count) + ".jpg", 85);

        // 다음 호출을 위해 현재 위치를 "이전 위치"로 갱신하고, 저장 카운터를 올립니다.
        last_save_x = x;
        last_save_y = y;
        my_file_count++;

        // 100장마다 진행 상황을 터미널에 출력합니다.
        if (my_file_count % 100 == 0) cout << "🚀 주행 데이터 수집 중: " << my_file_count << "장" << endl;
    }
};

// ================================================================================
// 메인 루프
// ================================================================================
int main(int argc, char **argv) {
    AutonomousController auto_car;

    // 데이터 수집기 초기화 및 Webots 맵 로드
    DataCollector my_collector(
        "/Users/baggyeongsu/Documents/vehicles/dataset",
        "/Users/baggyeongsu/Documents/vehicles/road_map.txt"
    );
    my_collector.load_road_map();

    // 시뮬레이터가 실행되는 동안 반복
    while (auto_car.driver->step() != -1) {
        const double *pos = auto_car.gps->getValues();
        
        // 데이터 수집 및 차선 주행
        my_collector.do_collect(auto_car.camera, pos[0], pos[1]);
        auto_car.do_driving();
    }
    return 0;
}