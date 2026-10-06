#pragma once // 혹시나 중복적으로 include 될 거를 대비하여.
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>

using namespace std;

// State = [x, y, yaw, v, steer]^T
// - y축 양의 방향이 차량 전방
// - x축 음수: 왼쪽, x축 양수: 오른쪽
using State = Eigen::Matrix<double, 5, 1>;

// Control = [a, steer_rate]^T
using Control = Eigen::Matrix<double, 2, 1>;

class KinematicMPC
{
private:
    int N = 8;
    double dt;
    double L = 2.995; // 차 크기 (m) 나중에 변환 필요

    static constexpr int nx = 5; // state 정의 개수
    static constexpr int nu = 2;

    static constexpr double min_v = 0.0; // m/s 단위 속도 제한
    static constexpr double max_v = 20.0;

    static constexpr double min_steer = -0.42;
    static constexpr double max_steer = 0.42;

    static constexpr double inf = 1.0e10;

    Eigen::Matrix<double, nx, nx> Q;
    Eigen::Matrix<double, nu, nu> R;
    Eigen::Matrix<double, nx, nx> Qf;

    Control u_min;
    Control u_max;

    Eigen::Matrix<double, nx, nx> A;
    Eigen::Matrix<double, nx, nu> B;
    State c;

    vector<State> predict_states;
    vector<Control> predict_controls;

private:
    int stateIndex(int t, int i) const // 나중에 P, q 만들떄 쓸 idx함수
    {
        return t * nx + i;
    }

    int controlIndex(int t, int i) const
    {
        return (N + 1) * nx + t * nu + i;
    }

    int numVariables() const // solver에 쓸 전체 z길이 벡터 구하는 함수 9*5 + 8*2 = 61 (전체 state개수 control)
    {
        return (N + 1) * nx + N * nu;
    }

    int numConstraints() const // constraint 개수
    {
        // N+1개의 동역학 제한용 state 개수 + 자체 범위 제한용 numVariables
        return (N + 1) * nx + numVariables();
    }

    void updateLinearizedModel(double initial_yaw, double initial_v, double initial_steer)
    {
        A.setZero();
        B.setZero();
        c.setZero();

        double phi = initial_yaw + initial_steer;

        double sin_phi = std::sin(phi);
        double cos_phi = std::cos(phi);
        double sin_steer = std::sin(initial_steer);
        double cos_steer = std::cos(initial_steer);

        // 이부분에 대해 설명하자면 kinematic single bicycle model을 기준으로 각 매개 변수에 대해 편미분을 통해
        // linearlize한 식을 행렬에 넣어준 것임.

        A(0, 0) = 1.0;
        A(0, 2) = dt * initial_v * cos_phi;
        A(0, 3) = dt * sin_phi;
        A(0, 4) = dt * initial_v * cos_phi;

        A(1, 1) = 1.0;
        A(1, 2) = -dt * initial_v * sin_phi;
        A(1, 3) = dt * cos_phi;
        A(1, 4) = -dt * initial_v * sin_phi;

        A(2, 2) = 1.0;
        A(2, 3) = dt * sin_steer / L;
        A(2, 4) = dt * initial_v * cos_steer / L;

        A(3, 3) = 1.0;

        A(4, 4) = 1.0;

        B(3, 0) = dt;
        B(4, 1) = dt;

        c(0) = -dt * initial_v * cos_phi * (initial_yaw + initial_steer);
        c(1) = dt * initial_v * sin_phi * (initial_yaw + initial_steer);
        c(2) = -dt * initial_v * cos_steer * initial_steer / L;
    }

    State dynamics(const State &state, const Control &control)
    {
        return A * state + B * control + c;
    }

    State makeReferenceState(
        int t,                                               // t번째 ref 탐색
        const std::vector<std::array<double, 2>> &waypoints, // 웨이포인트 묶음의 배열을 넣음
        const State &current_state) const                    // 현재 state
    {
        State ref;
        ref.setZero();

        int idx = std::min(t, static_cast<int>(waypoints.size()) - 1);

        ref(0) = waypoints[idx][0]; // target x
        ref(1) = waypoints[idx][1]; // target y

        // y축 전방이므로 atan2(dx, dy)를 써서 yaw를 계산하는데, 좀 더 안정적인 heading을 위해 앞 뒤 좌표의 차이를 이용함.
        if (idx > 0 && idx + 1 < static_cast<int>(waypoints.size())) // 경고 때문에 형변환함
        {
            double dx = waypoints[idx + 1][0] - waypoints[idx - 1][0];
            double dy = waypoints[idx + 1][1] - waypoints[idx - 1][1];
            ref(2) = std::atan2(dx, dy);
        }
        else if (idx + 1 < static_cast<int>(waypoints.size()))
        {
            double dx = waypoints[idx + 1][0] - waypoints[idx][0];
            double dy = waypoints[idx + 1][1] - waypoints[idx][1];
            ref(2) = std::atan2(dx, dy);
        }
        else if (idx > 0)
        {
            double dx = waypoints[idx][0] - waypoints[idx - 1][0];
            double dy = waypoints[idx][1] - waypoints[idx - 1][1];
            ref(2) = std::atan2(dx, dy);
        }
        else
        {
            ref(2) = current_state(2);
        }

        // 현재 속도를 유지하는 쪽으로 둠 -> 나중에 수정.
        ref(3) = current_state(3);

        // 기준 조향각은 0
        ref(4) = 0.0;

        return ref;
    }

public:
    KinematicMPC(int horizon = 10, double dt_ = 0.1) // 초기화 하면서 A,B,C matrix 초기화
        : N(horizon), dt(dt_)
    {
        A.setZero();
        B.setZero();
        c.setZero();

        u_min << -2.0, -0.8; // accel min, steer_rate min
        u_max << 1.5, 0.8;   // accel max, steer_rate max

        predict_states.resize(N + 1);
        predict_controls.resize(N);

        Q.setZero();
        R.setZero();
        Qf.setZero();

        // Q는 state 패널티임
        Q(0, 0) = 10.0; // x 좌표 차이
        Q(1, 1) = 3.0;  // y 좌표 차이
        Q(2, 2) = 2.0;  // heading 차이
        Q(3, 3) = 0.2;  // speed 유지
        Q(4, 4) = 1.0;  // steering 안정성

        // R은 control 패널티임
        R(0, 0) = 0.3; // 가속 페널티
        R(1, 1) = 2.0; // 감속 페널티

        Qf = Q * 5.0; // 계산이 다 끝나고 마지막 state에 대해서 가중치를 더 크게 줌.
    }

    //<OsqpEigen/OsqpEigen.h>라는 헤더파일 이용해서 문제 풀건데 이 Qsqp가 받는 인풋 형태로 메트릭스 정의를 다시 해야함
    //          l ≤ A z ≤ u 를 만족하면서
    //     0.5 z^T P z + q^T z 를 최소화 하는 z를 찾아줌.
    // 최종적으로 필요한 P, q, l, u, A를 형식에 맞게 만들어줘야함.

    std::vector<Control> solve(const State &current_state, const std::vector<std::array<double, 2>> &waypoints)
    {
        if (waypoints.empty())
        {
            throw std::runtime_error("KinematicMPC::solve(): waypoints is empty.");
        }

        updateLinearizedModel( // 행렬 A,B,C 초기화.
            current_state(2),  // yaw
            current_state(3),  // speed
            current_state(4)); // steer

        const int n_var = numVariables();
        const int n_con = numConstraints();

        std::vector<Eigen::Triplet<double>> P_triplet;           // zT P z에 들어갈 가중치 P 매트릭스의 0이 아닌 원소들만 저장.
        std::vector<Eigen::Triplet<double>> constraint_triplets; // l <= A z <= u 에서 A matrix.

        Eigen::VectorXd q_vector = Eigen::VectorXd::Zero(n_var);
        Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(n_con);
        Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(n_con);

        //(x - r)^T Q (x - r)= x^TQx - 2r^TQx (+ r^TQr) 임을 기억하자 (xT Q r^T 는 1*1이기 때문에 transpose해줘도 됨) + (+ r^TQr) 이거 상수라 고려 안해도 됨.

        // 이 for문은 state에 관한 P와 q 메트릭스를 만듬.
        for (int t = 0; t <= N; ++t)
        {
            State ref = makeReferenceState(t, waypoints, current_state);

            const Eigen::Matrix<double, nx, nx> &Q_use = (t == N) ? Qf : Q; // 맨 마지막인 경우만 Qf (맨 끝 인덱스의 정합성을 더 강하게 보려고)

            for (int i = 0; i < nx; ++i)
            {
                int idx = stateIndex(t, i); // 긴 하나의 벡터 안에서 몇번째인지 계산

                if (Q_use(i, i) != 0.0) // 이미 0으로 초기화임
                {
                    P_triplet.emplace_back(idx, idx, 2.0 * Q_use(i, i)); // 0.5 * z^T P z이므로 2*Q를 넣어야함.
                    q_vector(idx) += -2.0 * Q_use(i, i) * ref(i);        // q vector만들기. 원래는 Q의 row 전체를 넣어야 하지만
                                                                         // diagonal matrix이기 때문에 Q(i,i)번째 원소만 넣음
                }
            }
        }
        // 여기서는 N+1개의 state를 다 넣고 난 후 그 뒤에 action에 관한 가중치 P부분을 채움.
        // gradient를 안채우는 이유는 u^T R u에서 q^T z항이 따로 없기 때문임.
        for (int t = 0; t < N; ++t)
        {
            for (int i = 0; i < nu; ++i)
            {
                int idx = controlIndex(t, i);

                if (R(i, i) != 0.0)
                {
                    P_triplet.emplace_back(idx, idx, 2.0 * R(i, i)); // 0.5 * P * u^2 이므로 2R을 넣음
                }
            }
        }
        // constraint 제한 두는 부분
        // 맨 처음 state는 바꾸면 안되므로 current_state=<state[0]=<current_state로 고정해버림.
        int row = 0;

        for (int i = 0; i < nx; ++i)
        {
            constraint_triplets.emplace_back(row, stateIndex(0, i), 1.0);
            lower_bound(row) = current_state(i);
            upper_bound(row) = current_state(i);
            row++;
        }

        // 첫 번째 state를 제외한 나머지 state들은 x_{t+1} - A x_t - B u_t = c를 만족시키도록 식을 제한해야함 (이미 만들어 놓은 역학식을 만족시키도록)

        for (int t = 0; t < N; ++t)
        {
            for (int i = 0; i < nx; ++i)
            {
                constraint_triplets.emplace_back(row, stateIndex(t + 1, i), 1.0); // x_{t+1}는 그대로 넣음.

                for (int j = 0; j < nx; ++j)
                {
                    if (std::abs(A(i, j)) > 1e-12)
                    {
                        constraint_triplets.emplace_back(row, stateIndex(t, j), -A(i, j)); // x_t에 해당하는 A x_t
                    }
                }

                for (int j = 0; j < nu; ++j)
                {
                    if (std::abs(B(i, j)) > 1e-12)
                    {
                        constraint_triplets.emplace_back(row, controlIndex(t, j), -B(i, j)); // x_t에 해당하는 B x_t
                    }
                }

                lower_bound(row) = c(i); // x_{t+1} - A x_t - B u_t = c 를 만족시키는 c로 고정. (범위 설정이 아님)
                upper_bound(row) = c(i);
                row++;
            }
        }

        // 각 varialbe에 대해 각각 자체 제한을 거는 부분임
        for (int idx = 0; idx < n_var; ++idx)
        {
            constraint_triplets.emplace_back(row, idx, 1.0); // 자체 제한이기 때문에 각 변수에 대해 1을 곱함.

            double lb = -inf;
            double ub = inf;

            const int state_size = (N + 1) * nx; // 절대 안바뀔 값이므로

            if (idx < state_size) // state
            {
                int state_component = idx % nx; //

                // State = [x, y, yaw, v, steer]일 때
                if (state_component == 3) // State = [x, y, yaw, v, steer] 에서 v 제한
                {
                    lb = min_v;
                    ub = max_v;
                }
                else if (state_component == 4) // steer 제한
                {
                    lb = min_steer;
                    ub = max_steer;
                }
            }
            else
            {
                int control_component = (idx - state_size) % nu;

                lb = u_min(control_component);
                ub = u_max(control_component);
            }

            lower_bound(row) = lb;
            upper_bound(row) = ub;
            row++;
        }
        // SparseMatrix를 쓰는 이유는 대부분 원소가 0이기 때문임
        Eigen::SparseMatrix<double> P_matrix(n_var, n_var);
        Eigen::SparseMatrix<double> linear_constraints(n_con, n_var);

        // 지금까지 만든 triplets을 실제 matrix로 바꿔줌
        P_matrix.setFromTriplets(P_triplet.begin(), P_triplet.end());
        linear_constraints.setFromTriplets(
            constraint_triplets.begin(),
            constraint_triplets.end());

        OsqpEigen::Solver solver;

        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        solver.settings()->setPolish(true);
        // 지금까지 만들어준 l, u, A, P, q 넣어줌(라이브러리 사용)
        solver.data()->setNumberOfVariables(n_var);
        solver.data()->setNumberOfConstraints(n_con);
        solver.data()->setHessianMatrix(P_matrix);
        solver.data()->setGradient(q_vector);
        solver.data()->setLinearConstraintsMatrix(linear_constraints);
        solver.data()->setLowerBound(lower_bound);
        solver.data()->setUpperBound(upper_bound);
        solver.initSolver();
        solver.solveProblem();

        Eigen::VectorXd solution = solver.getSolution();

        std::vector<Control> optimal_controls;
        optimal_controls.resize(N);

        predict_states.resize(N + 1);
        predict_controls.resize(N);

        // 최종적으로 만든 z벡터에서 각 t별 state, action vector들로 다시 반환.
        for (int t = 0; t <= N; ++t)
        {
            for (int i = 0; i < nx; ++i)
            {
                predict_states[t](i) = solution(stateIndex(t, i));
            }
        }

        for (int t = 0; t < N; ++t)
        {
            for (int i = 0; i < nu; ++i)
            {
                optimal_controls[t](i) = solution(controlIndex(t, i));
                predict_controls[t](i) = optimal_controls[t](i);
            }
        }

        return optimal_controls;
    }

    /*
        CNN input 형식:
            offset, heading_angle,
            x0, y0,
            x1, y1,
            ...
            x9, y9

        x0, y0은 항상 0, 0이라 의미 없으므로 waypoint에서 제외함.
    */
    std::vector<Control> solve_CNN_inputs(const std::vector<double> &input, double current_v, double current_steer)
    {

        double offset = input[0];
        double heading_angle = input[1];

        State current_state;
        current_state << offset,
            0.0,
            heading_angle,
            current_v,
            current_steer;

        std::vector<std::array<double, 2>> waypoints;
        waypoints.reserve(N);

        // x0, y0는 넘김.
        for (int i = 1; i <= 9; ++i)
        {
            double x = input[2 + 2 * i];
            double y = input[2 + 2 * i + 1];
            waypoints.push_back({x, y});
        }

        return solve(current_state, waypoints);
    }

    Control one_step_control(
        const State &current_state,
        const std::vector<std::array<double, 2>> &waypoints)
    {
        auto controls = solve(current_state, waypoints);

        if (controls.empty())
        {
            Control safe;
            safe << 0.0, 0.0;
            return safe;
        }

        return controls[0];
    }

    Control make_control(const std::vector<double> &input, double current_v, double current_steer)
    {
        auto controls = solve_CNN_inputs(input, current_v, current_steer);

        return controls[0];
    }
};