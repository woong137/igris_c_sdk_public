/**
 * @file cyclonedds_gui_client.cpp
 * @brief GUI client example demonstrating IGRIS SDK features
 *
 * This example demonstrates:
 * - LowState subscription (real-time robot state monitoring)
 * - LowCmd publishing (low-level motor control at 300Hz)
 * - Service API calls (BMS init, torque control, control mode switching)
 *
 * GUI Layout:
 * - Left panel: 31 sliders with per-chain(waist/ankle/wrist) space selection
 * - Center panel: IMU state display + LowCmd publishing status
 * - Right panel: Service API buttons + response log window
 *
 * ---------------------------------------------------------------------------
 * [한국어 요약]
 * IGRIS 로봇을 PC에서 직접 만져볼 수 있는 GUI 개발 도구 예제입니다.
 *
 * 이 프로그램이 하는 일 3가지:
 *   1) 로봇 상태 받기 (Subscribe) : rt/lowstate, rt/robotstate 토픽을 구독해서 화면에 표시
 *   2) 로봇에 명령 보내기 (Publish): rt/lowcmd(관절 위치 명령, 300Hz), rt/cmd_vel(보행 속도, 20Hz)
 *   3) 서비스 호출 (Request/Reply) : BMS 초기화, 토크 ON/OFF, 제어 모드 전환 등 "한 번 요청하고 응답 받는" 명령
 *
 * 스레드 구성 (누가 어떤 일을 하는지):
 *   - 메인 스레드          : 화면(GUI)을 그리고 버튼/슬라이더 입력을 처리
 *   - LowCmdPublishThread  : 300Hz로 관절 명령을 계속 발행
 *   - TwistPublishThread   : 20Hz로 보행 속도 명령을 발행
 *   - DDS 콜백 스레드      : 메시지가 도착하면 LowStateCallback / RobotStateCallback 을 호출 (SDK 내부에서 생성)
 *   - 서비스 호출 스레드   : 버튼을 누를 때마다 잠깐 생겼다 사라짐 (응답을 기다리는 동안 GUI가 멈추지 않도록)
 *
 * 여러 스레드가 같은 전역 변수를 건드리므로, 단순 값은 std::atomic, 배열/구조체는 std::mutex 로 보호합니다.
 *
 * 화면 구성: 왼쪽(관절 슬라이더) | 가운데(연결 상태, IMU, 보행 속도, 발행 중인 명령) | 오른쪽(서비스 버튼)
 *            맨 아래에는 응답 로그 창이 가로로 길게 깔립니다.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <functional>
#include <future>
#include <igris_c_sdk/channel_factory.hpp>
#include <igris_c_sdk/igris_c_client.hpp>
#include <igris_c_sdk/publisher.hpp>
#include <igris_c_sdk/subscriber.hpp>
#include <iostream>
#include <mutex>
#include <thread>

// ImGui includes
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

using namespace igris_c_sdk;
using namespace igris_c::msg::dds;

// ============================================================================
//  이름 / 한계값 테이블 (모두 크기 31 = 로봇의 관절 개수, 배열 인덱스가 곧 관절 번호)
// ============================================================================
// MOTOR_NAMES : "모터 공간(MS)"에서 볼 때의 이름. 실제 액추에이터(모터) 하나하나를 가리킵니다.
// JOINT_NAMES : "관절 공간(PJS)"에서 볼 때의 이름. 사람이 생각하는 관절 축(Roll/Pitch/Yaw)을 가리킵니다.
//
// 대부분의 관절은 모터 1개 = 관절 1개라서 두 이름이 같습니다.
// 하지만 허리/발목/손목은 "병렬 링크" 구조라 모터 2개가 함께 움직여 관절 2축을 만듭니다.
//   예) 허리: 모터 Waist_L + Waist_R  <->  관절 Waist_Roll + Waist_Pitch
// 그래서 이 부위들만 두 테이블의 이름이 다릅니다.
// Motor and Joint names (1-indexed in data, 0-indexed in array)
static const std::array<const char *, 31> MOTOR_NAMES = {
    "Waist_Yaw",    "Waist_L",          "Waist_R",         "Hip_Pitch_L",    "Hip_Roll_L",    "Hip_Yaw_L",    "Knee_Pitch_L",
    "Ankle_Out_L",  "Ankle_In_L",       "Hip_Pitch_R",     "Hip_Roll_R",     "Hip_Yaw_R",     "Knee_Pitch_R", "Ankle_Out_R",
    "Ankle_In_R",   "Shoulder_Pitch_L", "Shoulder_Roll_L", "Shoulder_Yaw_L", "Elbow_Pitch_L", "Wrist_Yaw_L",  "Wrist_Front_L",
    "Wrist_Back_L", "Shoulder_Pitch_R", "Shoulder_Roll_R", "Shoulder_Yaw_R", "Elbow_Pitch_R", "Wrist_Yaw_R",  "Wrist_Front_R",
    "Wrist_Back_R", "Neck_Yaw",         "Neck_Pitch"};

static const std::array<const char *, 31> JOINT_NAMES = {
    "Waist_Yaw",     "Waist_Roll",       "Waist_Pitch",     "Hip_Pitch_L",    "Hip_Roll_L",    "Hip_Yaw_L",    "Knee_Pitch_L",
    "Ankle_Pitch_L", "Ankle_Roll_L",     "Hip_Pitch_R",     "Hip_Roll_R",     "Hip_Yaw_R",     "Knee_Pitch_R", "Ankle_Pitch_R",
    "Ankle_Roll_R",  "Shoulder_Pitch_L", "Shoulder_Roll_L", "Shoulder_Yaw_L", "Elbow_Pitch_L", "Wrist_Yaw_L",  "Wrist_Roll_L",
    "Wrist_Pitch_L", "Shoulder_Pitch_R", "Shoulder_Roll_R", "Shoulder_Yaw_R", "Elbow_Pitch_R", "Wrist_Yaw_R",  "Wrist_Roll_R",
    "Wrist_Pitch_R", "Neck_Yaw",         "Neck_Pitch"};

// 관절 공간(PJS) 기준 위치 한계 [rad]. 슬라이더의 최소/최대 범위로 사용됩니다.
// Joint position limits (rad) from params.yaml
static const std::array<float, 31> JOINT_POS_MAX = {
    1.57f,                                                   // Waist_Yaw
    0.310f, 0.28f,                                           // Waist_Roll, Waist_Pitch
    0.480f, 2.300f, 1.570f, 2.280f, 0.698f, 0.349f,          // Left leg
    0.480f, 0.330f, 1.570f, 2.280f, 0.698f, 0.349f,          // Right leg
    1.047f, 3.140f, 1.570f, 0.0f,   1.570f, 0.870f, 0.650f,  // Left arm
    1.047f, 0.170f, 1.570f, 0.0f,   1.570f, 1.221f, 0.650f,  // Right arm
    1.221f, 0.520f                                           // Neck
};

static const std::array<float, 31> JOINT_POS_MIN = {
    -1.57f,                                                       // Waist_Yaw
    -0.310f, -0.87f,                                              // Waist_Roll, Waist_Pitch
    -2.000f, -0.330f, -1.570f, 0.0f,  -0.70f,  -0.350f,           // Left leg
    -2.000f, -2.300f, -1.570f, 0.0f,  -0.70f,  -0.350f,           // Right leg
    -3.141f, -0.170f, -1.570f, -2.0f, -1.570f, -1.221f, -0.650f,  // Left arm
    -3.141f, -3.140f, -1.570f, -2.0f, -1.570f, -0.870f, -0.650f,  // Right arm
    -1.221f, -0.520f                                              // Neck
};

// 모터 공간(MS) 기준 위치 한계 [rad]. 병렬 링크 부위(허리/발목/손목)만 관절 공간 한계와 값이 다릅니다.
// Motor position limits (rad) from params.yaml
static const std::array<float, 31> MOTOR_POS_MAX = {
    1.57f,                                                  // Waist_Yaw
    0.87f,  0.87f,                                          // Waist_L, Waist_R
    0.480f, 2.300f, 1.570f, 2.280f, 0.609f, 0.523f,         // Left leg
    0.480f, 0.330f, 1.570f, 2.280f, 0.609f, 0.523f,         // Right leg
    1.047f, 3.140f, 1.570f, 0.0f,   1.570f, 0.75f,  0.75f,  // Left arm
    1.047f, 0.170f, 1.570f, 0.0f,   1.570f, 0.98f,  0.98f,  // Right arm
    1.221f, 0.520f                                          // Neck
};

static const std::array<float, 31> MOTOR_POS_MIN = {
    -1.57f,                                                      // Waist_Yaw
    -0.34f,  -0.34f,                                             // Waist_L, Waist_R
    -2.000f, -0.330f, -1.570f, 0.0f,  -0.630f, -0.617f,          // Left leg
    -2.000f, -2.300f, -1.570f, 0.0f,  -0.630f, -0.617f,          // Right leg
    -3.141f, -0.170f, -1.570f, -2.0f, -1.570f, -0.98f,  -0.98f,  // Left arm
    -3.141f, -3.140f, -1.570f, -2.0f, -1.570f, -0.75f,  -0.75f,  // Right arm
    -1.221f, -0.520f                                             // Neck
};

// ============================================================================
//  전역 상태 (여러 스레드가 함께 쓰는 변수들)
// ============================================================================
// g_running                 : false가 되면 모든 스레드와 메인 루프가 종료됩니다 (Ctrl+C 또는 창 닫기).
// g_lowstate_received_count : LowState 메시지를 지금까지 몇 번 받았는지 (화면 표시용).
// Global state
static std::atomic<bool> g_running(true);
static std::atomic<uint32_t> g_lowstate_received_count(0);

// 가장 최근에 받은 LowState(관절/모터/IMU 상태). 콜백 스레드가 쓰고 GUI 스레드가 읽으므로
// 접근할 때는 반드시 g_lowstate_mutex 를 잡아야 합니다.
// g_first_state_received : 첫 메시지를 받았는지 여부 (받기 전에는 "Waiting..." 표시).
// Latest state data
static LowState g_latest_lowstate;
static std::mutex g_lowstate_mutex;
static bool g_first_state_received = false;

// RobotState(로봇의 현재 제어 상태, 실기/시뮬 구분 등). 프로그램 시작 시 자동으로 구독합니다.
// g_last_robotstate_time 은 마지막 수신 시각으로, 2초 이상 끊기면 "Disconnected"로 표시하는 데 씁니다.
// RobotState subscription (auto-subscribe at startup)
static RobotState g_latest_robotstate;
static std::mutex g_robotstate_mutex;
static std::atomic<bool> g_robotstate_connected(false);
static std::atomic<uint32_t> g_robotstate_received_count(0);
static std::unique_ptr<Subscriber<RobotState>> g_robotstate_sub;
static std::chrono::steady_clock::time_point g_last_robotstate_time;

// 첫 LowState를 받았을 때의 모터/관절 위치를 기록해 둔 기준값.
// Initial reference values (set on first state receive)
static std::array<float, 31> g_initial_motor_pos = {};
static std::array<float, 31> g_initial_joint_pos = {};

// 슬라이더에 표시할 값. LowCmd 발행을 하지 않는 "표시 모드"에서 현재 로봇 위치를 보여주는 데 사용됩니다.
// GUI state
static std::array<float, 31> g_slider_values = {};

// 명령을 어느 공간 기준으로 보낼지 선택하는 값.
//   MOTOR(MS)  : 모터 각도를 직접 지정
//   JOINT(PJS) : 관절 각도(Roll/Pitch 등)를 지정 -> 로봇 쪽에서 모터 각도로 변환
enum class SpaceSelection : int {
    MOTOR = 0,
    JOINT = 1,
};

// 병렬 링크 부위는 총 5곳. 부위마다 MS / PJS 를 따로 고를 수 있습니다.
// g_parallel_space[n] 에 각 부위의 선택값이 들어갑니다. (기본값은 전부 MOTOR)
//   0: 허리, 1: 왼발목, 2: 오른발목, 3: 왼손목, 4: 오른손목
static constexpr int kParallelLinkCount                     = 5;  // waist, left_ankle, right_ankle, left_wrist, right_wrist
static std::array<int, kParallelLinkCount> g_parallel_space = {
    static_cast<int>(SpaceSelection::MOTOR),  // waist
    static_cast<int>(SpaceSelection::MOTOR),  // left_ankle
    static_cast<int>(SpaceSelection::MOTOR),  // right_ankle
    static_cast<int>(SpaceSelection::MOTOR),  // left_wrist
    static_cast<int>(SpaceSelection::MOTOR),  // right_wrist
};

// ----------------------------------------------------------------------------
//  저수준(LOW_LEVEL) 제어 상태
// ----------------------------------------------------------------------------
// 위치 값이 3종류 있는데, 역할이 다르니 구분해서 보세요.
//   target  (목표)  : 사용자가 슬라이더로 지정한 "최종적으로 가고 싶은 위치"
//   cmd     (명령)  : 실제로 로봇에 보내는 값. target을 향해 매 주기 조금씩 다가갑니다 (급격한 움직임 방지)
//   current (현재)  : 로봇이 보고한 실제 위치
// 각각 관절 공간(joint)용과 모터 공간(motor)용이 따로 있습니다.
// 위 배열들은 g_target_mutex 로 보호합니다.
//
// g_lowlevel_active    : true일 때만 300Hz 스레드가 LowCmd를 실제로 발행합니다.
// g_convergence_time   : cmd가 target에 도달하는 데 걸리는 시간(초). 슬라이더로 조절.
// g_reset_mode_active  : Reset/Home 버튼을 눌렀을 때 true. 이때는 안전하게 5초(RESET_DURATION)에 걸쳐 천천히 이동.
// g_last_published_cmd : 마지막으로 발행한 명령의 복사본 (가운데 패널 표에 표시하기 위함).
// LOW_LEVEL control state
static std::atomic<bool> g_lowlevel_active(false);
static std::array<float, 31> g_target_joint_pos  = {};  // Target joint positions for LOW_LEVEL mode
static std::array<float, 31> g_target_motor_pos  = {};  // Target motor positions for LOW_LEVEL mode
static std::array<float, 31> g_cmd_joint_pos     = {};  // Command joint positions (gradually converging)
static std::array<float, 31> g_cmd_motor_pos     = {};  // Command motor positions (gradually converging)
static std::array<float, 31> g_current_joint_pos = {};  // Current actual joint positions
static std::array<float, 31> g_current_motor_pos = {};  // Current actual motor positions
static std::mutex g_target_mutex;
static float g_convergence_time   = 1.5f;             // Convergence time in seconds (0=instant, 1.5=default, 3=slow)
static const float RESET_DURATION = 5.0f;             // Fixed duration for Reset/Home buttons
static std::atomic<bool> g_reset_mode_active(false);  // Use fixed duration for Reset/Home
static std::atomic<uint32_t> g_lowcmd_publish_count(0);
static LowCmd g_last_published_cmd;
static std::mutex g_last_cmd_mutex;

// ----------------------------------------------------------------------------
//  보행 속도(Twist) 명령 상태
// ----------------------------------------------------------------------------
// g_twist_publishing : true인 동안 20Hz로 속도 명령을 발행.
// g_twist_flushing   : Stop을 누른 직후, 속도 0을 잠깐 더 보내는 중인지 여부.
// vx: 전후 속도, vy: 좌우 속도, wz: 제자리 회전 속도.
// Walk velocity (Twist) state — published on rt/cmd_vel at 20Hz while active.
// Steered with +/- step buttons (no multi-touch needed); switch the robot into
// walkmode before publishing.
static std::atomic<bool> g_twist_publishing(false);
static std::atomic<bool> g_twist_flushing(false);  // sending the post-stop zero burst
static std::atomic<double> g_twist_vx(0.0);        // linear.x  (m/s)
static std::atomic<double> g_twist_vy(0.0);        // linear.y  (m/s)
static std::atomic<double> g_twist_wz(0.0);        // angular.z (rad/s)
static std::atomic<uint32_t> g_twist_publish_count(0);
static constexpr double TWIST_STEP_LIN = 0.05;  // m/s per click
static constexpr double TWIST_STEP_ANG = 0.1;   // rad/s per click
// On Stop, keep publishing zero velocity for this long so the robot decelerates
// to a halt instead of holding the last commanded velocity.
static constexpr auto TWIST_STOP_FLUSH = std::chrono::seconds(1);

// 화면 하단 로그 창에 표시할 문자열들. 최근 MAX_LOG_LINES(50)줄만 유지합니다.
// Response log
static std::deque<std::string> g_response_log;
static std::mutex g_log_mutex;
static const size_t MAX_LOG_LINES = 50;

// ----------------------------------------------------------------------------
//  PD 게인 (Kp: 위치 오차에 대한 강성, Kd: 속도에 대한 감쇠)
// ----------------------------------------------------------------------------
// Kp가 클수록 목표 위치로 강하게 당기고, Kd가 클수록 움직임이 덜 출렁입니다.
// 다리처럼 큰 힘이 필요한 관절은 Kp가 크고, 목/손목처럼 가벼운 관절은 작게 잡혀 있습니다.
// PD gains default values
static const std::array<float, 31> DEFAULT_KP = {
    50.0f,  25.0f,  25.0f,                               // Waist
    500.0f, 200.0f, 50.0f, 500.0f, 50.0f, 50.0f,         // Left leg
    500.0f, 200.0f, 50.0f, 500.0f, 50.0f, 50.0f,         // Right leg
    50.0f,  50.0f,  30.0f, 30.0f,  15.0f, 15.0f, 15.0f,  // Left arm  (wrist Yaw/Front/Back: 15)
    50.0f,  50.0f,  30.0f, 30.0f,  15.0f, 15.0f, 15.0f,  // Right arm (wrist Yaw/Front/Back: 15)
    2.0f,   5.0f                                         // Neck
};
static const std::array<float, 31> DEFAULT_KD = {
    0.8f,  0.8f, 0.8f,                            // Waist
    3.0f,  0.5f, 0.5f,  3.0f,  3.0f, 3.0f,        // Left leg
    3.0f,  0.5f, 0.5f,  3.0f,  3.0f, 3.0f,        // Right leg
    0.5f,  0.5f, 0.15f, 0.15f, 0.1f, 0.1f, 0.1f,  // Left arm
    0.5f,  0.5f, 0.15f, 0.15f, 0.1f, 0.1f, 0.1f,  // Right arm
    0.05f, 0.1f                                   // Neck
};

// g_kp / g_kd           : 실제로 명령에 실려 나가는 게인.
// g_kp_edit / g_kd_edit : 입력칸에 타이핑 중인 임시 값. 확인 팝업에서 Apply를 눌러야 위의 실제 값에 반영됩니다.
// PD gains (editable, initialized from defaults)
static std::array<float, 31> g_kp = DEFAULT_KP;
static std::array<float, 31> g_kd = DEFAULT_KD;
// Temporary edit buffers for kp/kd (applied on enter or focus loss)
static std::array<float, 31> g_kp_edit = DEFAULT_KP;
static std::array<float, 31> g_kd_edit = DEFAULT_KD;
// Temporary edit buffers for position (applied on enter or focus loss)
static std::array<float, 31> g_pos_edit = {};

// 게인 변경 확인 팝업에 필요한 정보 (어느 관절의, Kp/Kd 중 무엇을, 얼마에서 얼마로 바꾸려는지).
// Kp/Kd confirmation popup state
static bool g_show_gain_confirm_popup = false;
static int g_pending_gain_joint       = -1;    // Joint index for pending change
static bool g_pending_is_kp           = true;  // true = kp, false = kd
static float g_pending_old_value      = 0.0f;
static float g_pending_new_value      = 0.0f;

// 서비스 명령 확인 팝업 상태.
// 위험할 수 있는 명령(전원/토크/제어 모드)은 버튼을 눌러도 바로 실행하지 않고,
// "실행할 동작"을 함수(g_pending_service_action)로 담아두었다가 팝업에서 Confirm을 눌러야 실행합니다.
// Service-command confirmation popup state.
// Power/Torque and Control-Mode buttons stage their action in g_pending_service_action;
// the modal popup runs the action only after explicit user confirmation.
static bool g_show_service_confirm_popup = false;
static std::string g_pending_service_label;
static std::function<void()> g_pending_service_action;

void RequestServiceConfirm(const char *label, std::function<void()> action) {
    g_pending_service_label      = label;
    g_pending_service_action     = std::move(action);
    g_show_service_confirm_popup = true;
}

// ----------------------------------------------------------------------------
//  UI 스케일 계산
// ----------------------------------------------------------------------------
// 1800x900 창을 기준(배율 1.0)으로 삼고, 창이 커지면 글자/버튼도 비례해서 키웁니다.
// 배율은 1.15 ~ 2.5 범위로 제한합니다.
static constexpr float BASE_WINDOW_WIDTH  = 1800.0f;
static constexpr float BASE_WINDOW_HEIGHT = 900.0f;

float ClampUiScale(float scale) { return std::clamp(scale, 1.15f, 2.5f); }

float ComputeUiScale(float width, float height) { return ClampUiScale(std::min(width / BASE_WINDOW_WIDTH, height / BASE_WINDOW_HEIGHT)); }

// ----------------------------------------------------------------------------
//  병렬 링크 관련 헬퍼 함수
// ----------------------------------------------------------------------------
// 관절 번호(idx)가 병렬 링크 부위에 속하는지 판별합니다.
//   허리: 1, 2 / 발목: 7, 8(왼쪽), 13, 14(오른쪽) / 손목: 20, 21(왼쪽), 27, 28(오른쪽)
bool IsWaistParallelJoint(int idx) { return idx == 1 || idx == 2; }

bool IsAnkleParallelJoint(int idx) { return idx == 7 || idx == 8 || idx == 13 || idx == 14; }

bool IsWristParallelJoint(int idx) { return idx == 20 || idx == 21 || idx == 27 || idx == 28; }

// 관절 번호 -> 병렬 링크 부위 번호(0~4)로 변환. 병렬 링크가 아닌 일반 관절이면 -1을 반환합니다.
int GetParallelLinkIndexForJoint(int idx) {
    if (IsWaistParallelJoint(idx)) {
        return 0;
    }
    if (idx == 7 || idx == 8) {
        return 1;
    }
    if (idx == 13 || idx == 14) {
        return 2;
    }
    if (idx == 20 || idx == 21) {
        return 3;
    }
    if (idx == 27 || idx == 28) {
        return 4;
    }
    return -1;
}

// 이 관절을 모터 공간(MS)으로 다룰지 여부.
// 병렬 링크 부위면 사용자가 고른 값을 따르고, 일반 관절이면 항상 false(관절 공간 값 사용)입니다.
// (일반 관절은 모터 각도 = 관절 각도라서 어느 쪽을 써도 결과가 같습니다.)
bool UseMotorSpaceForJoint(int idx) {
    const int link_idx = GetParallelLinkIndexForJoint(idx);
    if (link_idx >= 0) {
        return g_parallel_space[static_cast<size_t>(link_idx)] == static_cast<int>(SpaceSelection::MOTOR);
    }
    return false;
}

// 관절 한 줄마다 "MS / PJS" 라디오 버튼을 그립니다.
// 병렬 링크가 아닌 관절은 선택할 게 없으므로 버튼을 회색(비활성)으로만 보여줍니다.
// 참고) ImGui에서 라벨의 "##" 뒤쪽은 화면에 보이지 않는 고유 ID입니다. ("MS##ms_3" -> 화면에는 "MS"만 표시)
// Render the per-row MS / PJS radio buttons. For joints that don't belong to
// any parallel link the radios are drawn but disabled (they show a dummy
// selection that doesn't match either choice, so neither is highlighted).
void RenderModeRadiosForJoint(int joint_idx) {
    const int link_idx = GetParallelLinkIndexForJoint(joint_idx);
    const bool active  = (link_idx >= 0);

    static int kDisabledNoSelection = -1;  // does not match MOTOR (0) nor JOINT (1)
    int *space                      = active ? &g_parallel_space[static_cast<size_t>(link_idx)] : &kDisabledNoSelection;

    if (!active) {
        ImGui::BeginDisabled();
    }

    char ms_id[32];
    char pjs_id[32];
    std::snprintf(ms_id, sizeof(ms_id), "MS##ms_%d", joint_idx);
    std::snprintf(pjs_id, sizeof(pjs_id), "PJS##pjs_%d", joint_idx);

    ImGui::RadioButton(ms_id, space, static_cast<int>(SpaceSelection::MOTOR));
    ImGui::SameLine();
    ImGui::RadioButton(pjs_id, space, static_cast<int>(SpaceSelection::JOINT));

    if (!active) {
        ImGui::EndDisabled();
    }
}

// MS <-> PJS 를 전환한 순간에 호출됩니다.
// 전환 직후 예전 공간의 목표값이 남아 있으면 로봇이 갑자기 튈 수 있으므로,
// 해당 부위의 target/cmd 를 "지금 로봇의 실제 위치"로 덮어써서 제자리에서 부드럽게 이어가도록 합니다.
// 주의: 호출하는 쪽에서 g_target_mutex 를 이미 잡고 있어야 합니다.
void SyncParallelLinkTargetsToCurrentState(int link_idx) {
    static constexpr std::array<std::array<int, 2>, kParallelLinkCount> kJointPairs = {{
        {1, 2},
        {7, 8},
        {13, 14},
        {20, 21},
        {27, 28},
    }};

    if (link_idx < 0 || link_idx >= kParallelLinkCount) {
        return;
    }

    const bool use_motor_space = g_parallel_space[static_cast<size_t>(link_idx)] == static_cast<int>(SpaceSelection::MOTOR);
    for (int joint_idx : kJointPairs[static_cast<size_t>(link_idx)]) {
        // Keep both spaces fresh and make selected-space command immediate.
        g_target_motor_pos[joint_idx] = g_current_motor_pos[joint_idx];
        g_cmd_motor_pos[joint_idx]    = g_current_motor_pos[joint_idx];
        g_target_joint_pos[joint_idx] = g_current_joint_pos[joint_idx];
        g_cmd_joint_pos[joint_idx]    = g_current_joint_pos[joint_idx];

        if (use_motor_space) {
            g_cmd_motor_pos[joint_idx] = g_target_motor_pos[joint_idx];
        } else {
            g_cmd_joint_pos[joint_idx] = g_target_joint_pos[joint_idx];
        }
    }
}

// Ctrl+C(SIGINT) / 종료 신호(SIGTERM)를 받으면 종료 플래그만 내립니다. 실제 정리는 main 끝에서 수행.
void SignalHandler(int) { g_running = false; }

// ----------------------------------------------------------------------------
//  UI 꾸미기용 헬퍼 (큰 제목 / 작은 제목 / 큰 글씨 버튼)
// ----------------------------------------------------------------------------
// Render a section header in a slightly larger font for visual hierarchy.
void SectionHeader(const char *text) {
    constexpr float kHeaderFontScale = 1.35f;
    ImGui::SetWindowFontScale(kHeaderFontScale);
    ImGui::TextUnformatted(text);
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Separator();
}

// Sub-group label: smaller than SectionHeader but still bigger than body, used
// to label related groups of buttons within a section.
void SubsectionHeader(const char *text) {
    constexpr float kSubFontScale = 1.15f;
    ImGui::Spacing();
    ImGui::SetWindowFontScale(kSubFontScale);
    ImGui::TextColored(ImVec4(0.6f, 0.85f, 1.0f, 1.0f), "%s", text);
    ImGui::SetWindowFontScale(1.0f);
}

// Action button with a larger, visually heavier label. Used for Reset/Home and
// Service API command buttons so they share one look-and-feel policy.
bool ScaledButton(const char *label, const ImVec2 &size) {
    constexpr float kButtonFontScale = 1.2f;
    ImGui::SetWindowFontScale(kButtonFontScale);
    const bool clicked = ImGui::Button(label, size);
    ImGui::SetWindowFontScale(1.0f);
    return clicked;
}

// 로그 한 줄 추가. 앞에 [시:분:초] 를 붙이고, 50줄이 넘으면 가장 오래된 줄부터 지웁니다.
// 여러 스레드에서 호출되므로 mutex로 보호합니다.
// Add log entry
void AddLog(const std::string &msg) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    auto now    = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm     = *std::localtime(&time_t);

    char time_str[32];
    std::strftime(time_str, sizeof(time_str), "%H:%M:%S", &tm);

    g_response_log.push_back(std::string("[") + time_str + "] " + msg);
    if (g_response_log.size() > MAX_LOG_LINES) {
        g_response_log.pop_front();
    }
}

// ----------------------------------------------------------------------------
//  DDS 구독 콜백 (메시지가 도착할 때마다 SDK가 별도 스레드에서 호출)
// ----------------------------------------------------------------------------
// RobotState 수신: 최신 값을 저장하고, 수신 시각을 갱신해 "연결됨"으로 표시합니다.
// RobotState callback
void RobotStateCallback(const RobotState &state) {
    std::lock_guard<std::mutex> lock(g_robotstate_mutex);
    g_latest_robotstate = state;
    g_robotstate_received_count++;
    g_last_robotstate_time = std::chrono::steady_clock::now();
    g_robotstate_connected = true;
}

// 로봇 제어 상태(enum)를 화면에 표시할 문자열로 바꿉니다.
// Helper to convert RobotControlState to string. Mirrors the labels used by
// robros::lib::common::RobotControlState::toString() in igris_c_lib.
const char *RobotControlStateToString(RobotControlState state) {
    switch (state) {
    case RobotControlState::ROBOT_STATE_NOT_READY:
        return "NOT_READY";
    case RobotControlState::ROBOT_STATE_IDLE:
        return "IDLE";
    case RobotControlState::ROBOT_STATE_STOP:
        return "STOP";
    case RobotControlState::ROBOT_STATE_LOW:
        return "LOW";
    case RobotControlState::ROBOT_STATE_HIGH_MOTION_ACTIVE:
        return "HIGH_MOTION_ACTIVE";
    case RobotControlState::ROBOT_STATE_HIGH_MOTION_STANDBY:
        return "HIGH_MOTION_STANDBY";
    case RobotControlState::ROBOT_STATE_WALK_LOW:
        return "WALK_LOW";
    case RobotControlState::ROBOT_STATE_WALK_HIGH_MOTION_ACTIVE:
        return "WALK_HIGH_MOTION_ACTIVE";
    case RobotControlState::ROBOT_STATE_WALK_HIGH_MOTION_STANDBY:
        return "WALK_HIGH_MOTION_STANDBY";
    case RobotControlState::ROBOT_STATE_WHOLE_BODY_CONTROL:
        return "WHOLE_BODY_CONTROL";
    default:
        return "UNKNOWN";
    }
}

// LowState 수신: 최신 상태 저장 + 현재 관절/모터 위치(current) 갱신.
// 맨 처음 받았을 때는 그 위치를 초기 기준값으로 기록합니다.
// LowState callback
void LowStateCallback(const LowState &state) {
    std::lock_guard<std::mutex> lock(g_lowstate_mutex);
    g_latest_lowstate = state;
    g_lowstate_received_count++;

    // Update current joint and motor state for control
    {
        std::lock_guard<std::mutex> lock_target(g_target_mutex);
        for (int i = 0; i < 31; i++) {
            g_current_joint_pos[i] = state.joint_state()[i].q();
            g_current_motor_pos[i] = state.motor_state()[i].q();
        }
    }

    // Set initial reference on first receive
    if (!g_first_state_received) {
        for (int i = 0; i < 31; i++) {
            g_initial_motor_pos[i] = state.motor_state()[i].q();
            g_initial_joint_pos[i] = state.joint_state()[i].q();
            g_slider_values[i]     = 0.0f;  // Start at 0 (relative to initial)
        }
        g_first_state_received = true;
        AddLog("First state received - reference values initialized");
    }
}

// ----------------------------------------------------------------------------
//  서비스 호출 (비동기)
// ----------------------------------------------------------------------------
// 서비스 호출은 응답이 올 때까지 최대 수십 초를 기다릴 수 있습니다.
// 메인 스레드에서 직접 기다리면 그동안 화면이 멈추므로, 호출마다 새 스레드를 띄워(detach) 거기서 기다립니다.
//
// g_service_call_in_progress : 호출이 진행 중이면 true. 동시에 두 개를 보내지 않도록 막는 용도입니다.
//
// 아래 함수들은 모두 같은 패턴입니다:
//   1) 이미 진행 중이면 로그만 남기고 리턴
//   2) 진행 중 플래그 on + "Calling ..." 로그
//   3) 새 스레드에서 client->XXX() 호출 -> 결과(SUCCESS/FAILED)를 로그에 기록 -> 플래그 off
// 호출 인자의 마지막 숫자(60000, 5000)는 타임아웃(ms)입니다.
// Async service call state
static std::atomic<bool> g_service_call_in_progress(false);

// Service call helpers (async versions to prevent GUI freeze)
void CallInitBmsAsync(IgrisC_Client *client, BmsInitType type, const char *type_name) {
    if (g_service_call_in_progress.load()) {
        AddLog("Service call already in progress, please wait...");
        return;
    }
    g_service_call_in_progress = true;
    AddLog(std::string("Calling InitBms(") + type_name + ")...");

    std::string type_name_copy(type_name);
    std::thread([client, type, type_name_copy]() {
        auto res = client->InitBms(type, 60000);
        std::string result =
            std::string("InitBms(") + type_name_copy + "): " + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message();
        AddLog(result);
        g_service_call_in_progress = false;
    }).detach();
}

void CallSetTorqueAsync(IgrisC_Client *client, TorqueType type, const char *type_name) {
    if (g_service_call_in_progress.load()) {
        AddLog("Service call already in progress, please wait...");
        return;
    }
    g_service_call_in_progress = true;
    AddLog(std::string("Calling SetTorque(") + type_name + ")...");

    std::string type_name_copy(type_name);
    std::thread([client, type, type_name_copy]() {
        auto res = client->SetTorque(type, 60000);
        std::string result =
            std::string("SetTorque(") + type_name_copy + "): " + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message();
        AddLog(result);
        g_service_call_in_progress = false;
    }).detach();
}

void CallInitHandAsync(IgrisC_Client *client) {
    if (g_service_call_in_progress.load()) {
        AddLog("Service call already in progress, please wait...");
        return;
    }
    g_service_call_in_progress = true;
    AddLog("Calling InitHand()...");

    std::thread([client]() {
        auto res = client->InitHand(60000);
        AddLog(std::string("InitHand(): ") + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message());
        g_service_call_in_progress = false;
    }).detach();
}

void CallSendControlModeCommandAsync(IgrisC_Client *client, ControlModeCommandType cmd_type, const char *label,
                                     const std::string &preset_id = "", bool is_cyclic = false) {
    if (g_service_call_in_progress.load()) {
        AddLog("Service call already in progress, please wait...");
        return;
    }
    g_service_call_in_progress = true;
    AddLog(std::string("Calling ControlModeCommand(") + label + ")...");

    std::string label_copy(label);
    std::thread([client, cmd_type, label_copy, preset_id, is_cyclic]() {
        auto res = client->SendControlModeCommand(cmd_type, preset_id, is_cyclic, 60000);
        AddLog(std::string("ControlModeCommand(") + label_copy + "): " + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message());
        g_service_call_in_progress = false;
    }).detach();
}

void CallSendMujocoSimCmdAsync(IgrisC_Client *client, MujocoSimCommandType cmd_type, const char *label) {
    if (g_service_call_in_progress.load()) {
        AddLog("Service call already in progress, please wait...");
        return;
    }
    g_service_call_in_progress = true;
    AddLog(std::string("Calling MujocoSim(") + label + ")...");

    std::string label_copy(label);
    std::thread([client, cmd_type, label_copy]() {
        auto res = client->SendMujocoSimCmd(cmd_type, 5000);
        AddLog(std::string("MujocoSim(") + label_copy + "): " + (res.success() ? "SUCCESS" : "FAILED") + " - " + res.message());
        g_service_call_in_progress = false;
    }).detach();
}

// ----------------------------------------------------------------------------
//  LowCmd 발행 시작 / 정지
// ----------------------------------------------------------------------------
// 시작할 때 target/cmd 를 로봇의 현재 위치로 맞춰 둡니다.
// 이렇게 해야 발행을 켜는 순간 로봇이 엉뚱한 위치(예: 0)로 튀지 않고 제자리를 유지합니다.
// Start LowCmd publishing
void StartLowCmdPublishing() {
    if (g_lowlevel_active) {
        AddLog("LowCmd publishing already active");
        return;
    }
    // Initialize target and command positions to current positions
    {
        std::lock_guard<std::mutex> lock_state(g_lowstate_mutex);
        std::lock_guard<std::mutex> lock_target(g_target_mutex);
        for (int i = 0; i < 31; i++) {
            g_target_joint_pos[i] = g_latest_lowstate.joint_state()[i].q();
            g_target_motor_pos[i] = g_latest_lowstate.motor_state()[i].q();
            g_cmd_joint_pos[i]    = g_latest_lowstate.joint_state()[i].q();
            g_cmd_motor_pos[i]    = g_latest_lowstate.motor_state()[i].q();
        }
    }
    g_lowlevel_active = true;
    AddLog("LowCmd publishing started - initialized to current positions");
}

// 정지: 플래그만 내립니다. 300Hz 스레드는 계속 돌지만 메시지를 보내지 않게 됩니다.
// Stop LowCmd publishing
void StopLowCmdPublishing() {
    if (!g_lowlevel_active) {
        AddLog("LowCmd publishing already stopped");
        return;
    }
    g_lowlevel_active = false;
    AddLog("LowCmd publishing stopped");
}

// ----------------------------------------------------------------------------
//  LowCmd 발행 스레드 (300Hz = 약 3.33ms 마다 1회)
// ----------------------------------------------------------------------------
// 매 주기마다:
//   1) 부위별 MS/PJS 모드를 메시지에 기록
//   2) 31개 관절 각각에 대해 cmd 를 target 쪽으로 조금 이동 (부드러운 보간)
//   3) 위치/게인을 담아 발행
// sleep_until(next_time) 방식이라 계산에 걸린 시간과 무관하게 주기가 일정하게 유지됩니다.
// 300Hz LowCmd publishing thread
void LowCmdPublishThread(Publisher<LowCmd> *publisher) {
    const auto period = std::chrono::microseconds(3333);  // ~300Hz
    auto next_time    = std::chrono::steady_clock::now();

    while (g_running) {
        if (g_lowlevel_active) {
            LowCmd cmd;

            {
                std::lock_guard<std::mutex> lock_target(g_target_mutex);

                // [1] 병렬 링크 5곳 각각을 관절 공간(PJS)으로 해석할지 모터 공간(MS)으로 해석할지 로봇에 알려줍니다.
                // Set kinematic modes by 5 parallel links
                for (int mode_idx = 0; mode_idx < kParallelLinkCount; ++mode_idx) {
                    cmd.kinematic_modes()[mode_idx] =
                        (g_parallel_space[static_cast<size_t>(mode_idx)] == static_cast<int>(SpaceSelection::JOINT)) ? KinematicMode::PJS
                                                                                                                     : KinematicMode::MS;
                }

                // [2] 보간 비율(alpha) 계산.
                // 매 주기 "남은 거리의 alpha 만큼"만 이동합니다. (지수 평활: 처음엔 빠르고 목표에 가까울수록 느려짐)
                // T초 뒤 남은 거리가 5%가 되도록 하려면 (1 - alpha)^(T*300) = 0.05 이므로 아래 식이 나옵니다.
                // Calculate alpha for exponential smoothing based on convergence time
                // At 300Hz, to reach 95% in T seconds: alpha = 1 - 0.05^(1/(T*300))
                // T=0 -> instant (alpha=1), T=1.5 -> ~0.01, T=3 -> ~0.005
                // Use RESET_DURATION (5s) for Reset/Home, otherwise use slider value
                float duration = g_reset_mode_active.load() ? RESET_DURATION : g_convergence_time;
                float alpha;
                const float min_delta = 0.0001f;  // Minimum delta for target reached
                if (duration <= 0.01f) {
                    alpha = 1.0f;  // Instant
                } else {
                    alpha = 1.0f - std::pow(0.05f, 1.0f / (duration * 300.0f));
                }

                bool all_reached = true;
                for (int i = 0; i < 31; i++) {
                    auto &motor_cmd = cmd.motors()[i];

                    // Set motor ID
                    motor_cmd.id(i);

                    // 이 관절에 쓸 cmd / target 을 선택 (모터 공간용 또는 관절 공간용).
                    // q_cmd_ref 는 참조(&)라서 여기서 값을 바꾸면 전역 배열이 바로 갱신됩니다.
                    const bool use_motor_space = UseMotorSpaceForJoint(i);
                    float &q_cmd_ref           = use_motor_space ? g_cmd_motor_pos[i] : g_cmd_joint_pos[i];
                    float q_target             = use_motor_space ? g_target_motor_pos[i] : g_target_joint_pos[i];

                    // Apply exponential smoothing (비율 기반 보간)
                    float delta = q_target - q_cmd_ref;
                    if (std::abs(delta) > min_delta) {
                        q_cmd_ref += delta * alpha;
                        all_reached = false;
                    } else {
                        q_cmd_ref = q_target;  // Target reached
                    }

                    // [3] 메시지에 값 채우기.
                    //   q       : 목표 위치 [rad]
                    //   dq, tau : 목표 속도와 추가 토크는 0 -> 순수 위치 제어
                    //   kp, kd  : PD 게인. 로봇 쪽 모터 제어기가 이 값으로 위치를 추종합니다.
                    // Set commands
                    motor_cmd.q(q_cmd_ref);
                    motor_cmd.dq(0.0f);
                    motor_cmd.tau(0.0f);
                    motor_cmd.kp(g_kp[i]);
                    motor_cmd.kd(g_kd[i]);
                }

                // Deactivate reset mode when all joints reached target
                if (all_reached && g_reset_mode_active.load()) {
                    g_reset_mode_active = false;
                }
            }

            // Publish
            publisher->write(cmd);

            // Update publish count and save last command
            g_lowcmd_publish_count++;
            {
                std::lock_guard<std::mutex> lock(g_last_cmd_mutex);
                g_last_published_cmd = cmd;
            }
        }

        // 다음 주기 시각까지 대기 (발행 중이 아닐 때도 같은 주기로 깨어나 플래그만 확인).
        // Sleep until next cycle
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

// ----------------------------------------------------------------------------
//  보행 속도 발행 스레드 (20Hz = 50ms 마다 1회)
// ----------------------------------------------------------------------------
// g_twist_publishing 이 켜져 있는 동안 현재 속도값(vx, vy, wz)을 계속 보냅니다.
// Stop을 누르면 바로 끊지 않고, 속도 0을 1초간 더 보내서 로봇이 확실히 멈추게 합니다.
// 20Hz WalkTwist publishing thread (rt/cmd_vel). Publishes the current
// linear.x / linear.y / angular.z values while g_twist_publishing is set.
void TwistPublishThread(Publisher<WalkTwist> *publisher) {
    const auto period   = std::chrono::milliseconds(50);  // 20Hz
    auto next_time      = std::chrono::steady_clock::now();
    uint32_t seq        = 0;
    bool prev_active    = false;
    auto flush_deadline = std::chrono::steady_clock::now();

    while (g_running) {
        const bool active = g_twist_publishing.load();
        // "방금 꺼졌다"(직전 주기엔 켜짐, 지금은 꺼짐)를 감지 -> 지금부터 1초간 0 속도 전송(flush) 시작.
        // flush 중에 다시 켜지거나 1초가 지나면 flush 종료.
        // Falling edge (Stop pressed): the toggle has already zeroed the
        // velocity, so keep publishing those zeros for TWIST_STOP_FLUSH.
        if (!active && prev_active) {
            flush_deadline   = std::chrono::steady_clock::now() + TWIST_STOP_FLUSH;
            g_twist_flushing = true;
        }
        if (g_twist_flushing && (active || std::chrono::steady_clock::now() >= flush_deadline)) {
            g_twist_flushing = false;
        }

        if (active || g_twist_flushing) {
            WalkTwist msg;
            auto &h        = msg.header();
            const auto now = std::chrono::system_clock::now().time_since_epoch();
            const auto sec = std::chrono::duration_cast<std::chrono::seconds>(now);
            h.seq(seq++);
            h.sec(static_cast<uint32_t>(sec.count()));
            h.nanosec(static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now - sec).count()));
            msg.twist().linear().x(g_twist_vx.load());
            msg.twist().linear().y(g_twist_vy.load());
            msg.twist().angular().z(g_twist_wz.load());
            publisher->write(msg);
            g_twist_publish_count++;
        }

        prev_active = active;
        next_time += period;
        std::this_thread::sleep_until(next_time);
    }
}

// ============================================================================
//  main
// ============================================================================
// 흐름: 인자 확인 -> 창/ImGui 준비 -> DDS 초기화(구독자, 발행자, 서비스 클라이언트)
//       -> 발행 스레드 시작 -> GUI 메인 루프 -> 종료 정리
//
// 실행 예: ./cyclonedds_gui_client 0 ""      (도메인 ID 0, 네임스페이스 없음)
// 로봇 쪽과 도메인 ID / 네임스페이스가 같아야 서로 통신이 됩니다.
int main(int argc, char **argv) {
    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    std::cout << "╔════════════════════════════════════════╗" << std::endl;
    std::cout << "║         IGRIS-SDK Dev Tool            ║" << std::endl;
    std::cout << "╚════════════════════════════════════════╝\n" << std::endl;

    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <domain_id> <namespace>" << std::endl;
        std::cerr << "         " << argv[0] << " 0 \"\" (no namespace)" << std::endl;
        return 1;
    }
    int domain_id  = std::atoi(argv[1]);
    std::string ns = argv[2];

    std::cout << "Domain ID: " << domain_id << std::endl;
    std::cout << "Namespace: \"" << ns << "\"" << std::endl;
    std::cout << "Make sure the robot controller is running!\n" << std::endl;

    // ---- 창 생성 (GLFW: 창과 키보드/마우스 입력 담당, OpenGL 3.3 사용) ----
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return 1;
    }

    // GL 3.3 + GLSL 330
    const char *glsl_version = "#version 330";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create a large default window relative to the monitor so the UI stays usable
    // on high-resolution displays instead of opening at a fixed pixel size.
    int initial_width  = static_cast<int>(BASE_WINDOW_WIDTH);
    int initial_height = static_cast<int>(BASE_WINDOW_HEIGHT);
    if (GLFWmonitor *primary_monitor = glfwGetPrimaryMonitor()) {
        int work_x = 0, work_y = 0, work_width = 0, work_height = 0;
        glfwGetMonitorWorkarea(primary_monitor, &work_x, &work_y, &work_width, &work_height);
        if (work_width > 0 && work_height > 0) {
            initial_width  = std::max(static_cast<int>(BASE_WINDOW_WIDTH), static_cast<int>(work_width * 0.9f));
            initial_height = std::max(static_cast<int>(BASE_WINDOW_HEIGHT), static_cast<int>(work_height * 0.9f));
        }
    }

    GLFWwindow *window = glfwCreateWindow(initial_width, initial_height, "IGRIS-SDK Dev Tool", NULL, NULL);
    if (window == NULL) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return 1;
    }
    // Enforce a minimum window size proportional to the monitor's work area so
    // the 3-panel + log-strip layout never collapses. On 4K monitors the floor
    // ends up much larger than on 1080p, keeping fonts/buttons readable.
    {
        int min_width  = 1100;
        int min_height = 650;
        if (GLFWmonitor *primary_monitor = glfwGetPrimaryMonitor()) {
            int work_x = 0, work_y = 0, work_width = 0, work_height = 0;
            glfwGetMonitorWorkarea(primary_monitor, &work_x, &work_y, &work_width, &work_height);
            if (work_width > 0 && work_height > 0) {
                // ~60% of the monitor's usable area, clamped to a sane floor.
                min_width  = std::max(min_width, static_cast<int>(work_width * 0.6f));
                min_height = std::max(min_height, static_cast<int>(work_height * 0.6f));
            }
        }
        glfwSetWindowSizeLimits(window, min_width, min_height, GLFW_DONT_CARE, GLFW_DONT_CARE);
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // Enable vsync

    // ---- ImGui 초기화 (버튼, 슬라이더 등 UI 위젯을 그려주는 라이브러리) ----
    // base_style 은 배율 적용 전 원본 스타일. 창 크기가 바뀔 때마다 여기서부터 다시 배율을 곱합니다.
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    ImGuiStyle base_style  = ImGui::GetStyle();
    float applied_ui_scale = 0.0f;

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // ---- DDS 통신 초기화 ----
    // ChannelFactory: 모든 발행자/구독자가 공유하는 통신 기반. 가장 먼저 초기화해야 합니다.
    // Initialize ChannelFactory
    std::cout << "Initializing ChannelFactory..." << std::endl;
    ChannelFactory::Instance()->Init(domain_id, ns);

    if (!ChannelFactory::Instance()->IsInitialized()) {
        std::cerr << "Failed to initialize ChannelFactory" << std::endl;
        return 1;
    }

    // 서비스 호출용 클라이언트 (BMS 초기화, 토크, 제어 모드 전환 등에 사용).
    // Initialize IgrisC_Client
    std::cout << "Initializing IgrisC_Client..." << std::endl;
    IgrisC_Client client;
    client.Init();
    client.SetTimeout(5.0f);

    // 구독자: rt/lowstate 메시지가 오면 LowStateCallback 이 호출됩니다.
    // SensorData QoS = 고주기 센서 데이터용 설정 (오래된 데이터를 재전송하기보다 최신 값을 우선).
    // Create LowState subscriber
    Subscriber<LowState> lowstate_sub("rt/lowstate", QosProfile::SensorData());
    if (!lowstate_sub.init(LowStateCallback)) {
        std::cerr << "Failed to initialize LowState subscriber" << std::endl;
        return 1;
    }

    // 발행자: rt/lowcmd 로 관절 명령을 보냅니다. 실제 전송은 LowCmdPublishThread 에서 합니다.
    // Create LowCmd publisher
    std::cout << "Initializing LowCmd publisher..." << std::endl;
    Publisher<LowCmd> lowcmd_pub("rt/lowcmd", QosProfile::SensorData());
    if (!lowcmd_pub.init()) {
        std::cerr << "Failed to initialize LowCmd publisher" << std::endl;
        return 1;
    }

    // 발행자: rt/cmd_vel 로 보행 속도 명령을 보냅니다. 실제 전송은 TwistPublishThread 에서 합니다.
    // Create WalkTwist publisher (walk velocity command on rt/cmd_vel).
    // Default QoS (~rclcpp::QoS(10)) matches the bridge's rt/cmd_vel subscriber.
    Publisher<WalkTwist> twist_pub("rt/cmd_vel", QosProfile::Default());
    if (!twist_pub.init()) {
        std::cerr << "Failed to initialize WalkTwist publisher" << std::endl;
        return 1;
    }

    // 구독자: rt/robotstate (로봇 제어 상태). 실패해도 프로그램은 계속 진행합니다.
    // Auto-subscribe to RobotState
    std::cout << "Subscribing to RobotState..." << std::endl;
    g_robotstate_sub = std::make_unique<Subscriber<RobotState>>("rt/robotstate");
    if (g_robotstate_sub->init(RobotStateCallback)) {
        AddLog("RobotState subscription started");
    } else {
        AddLog("Failed to subscribe to RobotState");
        g_robotstate_sub.reset();
    }

    // ---- 백그라운드 발행 스레드 시작 ----
    // 스레드는 지금부터 돌지만, 각 플래그(g_lowlevel_active / g_twist_publishing)가 켜지기 전에는 아무것도 보내지 않습니다.
    // Start LowCmd publishing thread
    std::thread publish_thread(LowCmdPublishThread, &lowcmd_pub);

    // Start WalkTwist publishing thread (rt/cmd_vel)
    std::thread twist_thread(TwistPublishThread, &twist_pub);

    std::cout << "GUI Client ready!\n" << std::endl;
    AddLog("GUI Client initialized successfully");

    // ============================================================================
    //  GUI 메인 루프 (창을 닫거나 Ctrl+C 를 누를 때까지 매 프레임 반복)
    // ============================================================================
    // ImGui는 "즉시 모드" 방식입니다. 매 프레임 화면 전체를 처음부터 다시 그리며,
    // if (Button(...)) { ... } 처럼 "그리는 코드"가 곧 "클릭 처리 코드"가 됩니다.
    // Main loop
    while (!glfwWindowShouldClose(window) && g_running) {
        // Poll events
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 창 크기에 맞춰 UI 배율 계산. 배율이 달라졌을 때만 스타일/글자 크기를 다시 적용합니다.
        const float ui_scale = ComputeUiScale(io.DisplaySize.x, io.DisplaySize.y);
        if (std::abs(ui_scale - applied_ui_scale) > 0.001f) {
            ImGui::GetStyle() = base_style;
            ImGui::GetStyle().ScaleAllSizes(ui_scale);
            io.FontGlobalScale = ui_scale;
            applied_ui_scale   = ui_scale;
        }

        // ---- 레이아웃 치수 계산 ----
        // ImGui에서 크기를 음수로 주면 "남은 공간에서 그만큼 뺀 크기"라는 뜻입니다.
        //   left_panel_width (음수) : 가운데+오른쪽 패널 폭을 뺀 나머지 전부를 왼쪽 패널이 차지
        //   top_panel_height (음수) : 하단 로그 창 높이를 뺀 나머지 전부를 위쪽 3개 패널이 차지
        // Right and Center are fixed-width (sized for content); the Left panel
        // absorbs the remaining horizontal space so on wide monitors there is
        // no empty area to the right of the center sections.
        const float right_panel_width  = io.DisplaySize.x * (320.0f / BASE_WINDOW_WIDTH);
        const float center_panel_width = 720.0f * ui_scale;
        const float left_panel_width   = -(center_panel_width + right_panel_width + 2.0f * ImGui::GetStyle().ItemSpacing.x);
        // Unified height for action buttons (Reset/Home group + Service API group).
        const float action_btn_height = 38.0f * ui_scale;
        // Column layout: [Name] [Mode (MS/PJS)] [Position slider] [Kp] [Kd]
        const float name_column_width    = 180.0f * ui_scale;
        const float mode_column_width    = 90.0f * ui_scale;
        const float mode_column_offset   = name_column_width + 10.0f * ui_scale;
        const float slider_column_offset = mode_column_offset + mode_column_width + 10.0f * ui_scale;
        const float kp_width             = 60.0f * ui_scale;
        const float kd_width             = 50.0f * ui_scale;
        const float popup_button_width   = 180.0f * ui_scale;
        const float log_panel_height     = 160.0f * ui_scale;
        const float top_panel_height     = -(log_panel_height + ImGui::GetStyle().ItemSpacing.y);

        // Create main window
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(io.DisplaySize);
        ImGui::Begin("IGRIS-SDK Dev Tool", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                         ImGuiWindowFlags_NoBringToFrontOnFocus);

        // ==========================================================================
        //  [왼쪽 패널] 관절 슬라이더 + Kp/Kd 입력
        // ==========================================================================
        // Left panel: Sliders
        ImGui::BeginChild("LeftPanel", ImVec2(left_panel_width, top_panel_height), true);
        {
            SectionHeader("LowCmd Sliders");

            // Capture parallel-space before per-row radios are rendered; we
            // detect changes after the joint-row loop and sync each link once.
            static constexpr std::array<const char *, kParallelLinkCount> kParallelNames = {"Waist", "Left Ankle", "Right Ankle",
                                                                                            "Left Wrist", "Right Wrist"};
            std::array<int, kParallelLinkCount> prev_parallel_space                      = g_parallel_space;

            // 상단 버튼 3개
            //   Reset Pos   : 모든 관절 목표를 0으로 (5초에 걸쳐 천천히 이동)
            //   Reset Gains : Kp/Kd 를 기본값으로 되돌림
            //   Home        : 모든 관절 0 + 양쪽 어깨만 살짝 벌린 자세(±0.4rad)로 (5초)
            // Reset Pos / Home 은 LowCmd 발행 중일 때만 동작합니다.
            // cmd 는 건드리지 않고 target 만 바꾸므로, 300Hz 스레드가 알아서 부드럽게 따라갑니다.
            // Reset, Reset Gains, and Home buttons (3 buttons side by side)
            float button_width = (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x * 2) / 3.0f;
            if (ScaledButton("Reset Pos", ImVec2(button_width, action_btn_height))) {
                if (g_lowlevel_active) {
                    std::lock_guard<std::mutex> lock_target(g_target_mutex);
                    for (int i = 0; i < 31; i++) {
                        g_target_motor_pos[i] = 0.0f;
                        g_target_joint_pos[i] = 0.0f;
                        // Don't reset cmd - let it converge gradually
                        g_slider_values[i] = 0.0f;
                    }
                    g_reset_mode_active = true;  // Use 5s duration
                    AddLog("All target positions reset to zero (5s transition)");
                } else {
                    AddLog("Enable LOW_LEVEL mode first to use Reset");
                }
            }
            ImGui::SameLine();
            if (ScaledButton("Reset Gains", ImVec2(button_width, action_btn_height))) {
                for (int i = 0; i < 31; i++) {
                    g_kp[i]      = DEFAULT_KP[i];
                    g_kd[i]      = DEFAULT_KD[i];
                    g_kp_edit[i] = DEFAULT_KP[i];
                    g_kd_edit[i] = DEFAULT_KD[i];
                }
                AddLog("All Kp/Kd gains reset to defaults");
            }
            ImGui::SameLine();
            if (ScaledButton("Home", ImVec2(button_width, action_btn_height))) {
                if (g_lowlevel_active) {
                    std::lock_guard<std::mutex> lock_target(g_target_mutex);
                    // Set all targets to zero first (cmd will converge gradually)
                    for (int i = 0; i < 31; i++) {
                        g_target_motor_pos[i] = 0.0f;
                        g_target_joint_pos[i] = 0.0f;
                        // Don't reset cmd - let it converge gradually
                        g_slider_values[i] = 0.0f;
                    }
                    // Set Shoulder Roll L (16) to 0.4, Shoulder Roll R (23) to -0.4
                    const int SHOULDER_ROLL_L           = 16;
                    const int SHOULDER_ROLL_R           = 23;
                    g_target_joint_pos[SHOULDER_ROLL_L] = 0.4f;
                    g_target_joint_pos[SHOULDER_ROLL_R] = -0.4f;
                    g_target_motor_pos[SHOULDER_ROLL_L] = 0.4f;   // Also set motor pos
                    g_target_motor_pos[SHOULDER_ROLL_R] = -0.4f;  // Also set motor pos
                    g_slider_values[SHOULDER_ROLL_L]    = 0.4f;
                    g_slider_values[SHOULDER_ROLL_R]    = -0.4f;
                    g_reset_mode_active                 = true;  // Use 5s duration
                    AddLog("Home position (5s transition)");
                } else {
                    AddLog("Enable LOW_LEVEL mode first to use Home");
                }
            }

            // LowCmd 발행 시작/정지 토글 버튼. 상태에 따라 버튼 글자가 바뀝니다.
            // LowCmd publish toggle, sized to span the same row width as the
            // 3 buttons above so they align visually.
            if (!g_lowlevel_active) {
                if (ScaledButton("LowCmd Publish Start", ImVec2(-1, action_btn_height))) {
                    StartLowCmdPublishing();
                }
            } else {
                if (ScaledButton("LowCmd Publish Stop", ImVec2(-1, action_btn_height))) {
                    StopLowCmdPublishing();
                }
            }
            ImGui::Separator();

            // LowState 수신 횟수 + 수렴 시간(Duration) 슬라이더. 0이면 즉시 이동, 값이 클수록 천천히 이동합니다.
            // Status and Convergence Time slider on same line.
            // Pre-compute the trailing "Duration: X.Xs" width using the current
            // font scale so the slider fills the remaining space exactly and
            // never overflows the panel right edge.
            const std::string duration_text   = "Duration: 9.9s";  // worst-case width
            const float duration_text_width   = ImGui::CalcTextSize(duration_text.c_str()).x;
            const float duration_item_spacing = ImGui::GetStyle().ItemSpacing.x;
            ImGui::Text("LowState: %u", g_lowstate_received_count.load());
            ImGui::SameLine();
            const float slider_width = std::max(80.0f, ImGui::GetContentRegionAvail().x - duration_text_width - duration_item_spacing);
            ImGui::SetNextItemWidth(slider_width);
            ImGui::SliderFloat("##convergence", &g_convergence_time, 0.0f, 3.0f, "");
            ImGui::SameLine();
            ImGui::Text("Duration: %.1fs", g_convergence_time);
            if (!g_first_state_received) {
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "Waiting for first state...");
            } else {
                ImGui::TextColored(ImVec4(0, 1, 0, 1), "Reference initialized");
            }
            ImGui::Separator();

            // ---- 관절 31개 목록 (스크롤 영역) ----
            // 한 줄 구성: [이름] [MS/PJS 선택] [위치 슬라이더] [Kp] [Kd]
            // Sliders (31 motors/joints)
            ImGui::BeginChild("SliderScroll", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

            // Show LOW_LEVEL mode status
            if (g_lowlevel_active) {
                ImGui::TextColored(ImVec4(1, 0.5f, 0, 1), "LOW_LEVEL Mode - Arrow:0.001 Ctrl:0.01 Shift:0.1 | Ctrl+Click:Custom");
            }

            // Column headers (aligned with slider layout)
            float content_width = ImGui::GetContentRegionAvail().x;
            float spacing       = ImGui::GetStyle().ItemSpacing.x;

            ImGui::Text("Name");
            ImGui::SameLine(mode_column_offset);
            ImGui::Text("Space");
            ImGui::SameLine(slider_column_offset);
            ImGui::Text("Position");
            ImGui::SameLine(content_width - kd_width - spacing - kp_width);
            ImGui::Text("  Kp");
            ImGui::SameLine(content_width - kd_width);
            ImGui::Text("  Kd");
            ImGui::Separator();

            const float FINE_STEP     = 0.001f;  // Fine adjustment step for arrow keys
            static int selected_joint = -1;      // Currently selected joint for fine control
            // Scroll to the selected joint only on the frame the selection
            // changes. Continuous SetScrollHereY would fight manual scroll.
            static bool scroll_to_selected = false;

            // 키보드 조작: 위/아래 화살표로 관절 선택 (끝에서 넘어가면 반대쪽 끝으로 순환).
            // 선택된 관절은 아래에서 좌/우 화살표로 미세 조정할 수 있습니다.
            // Handle Up/Down arrow keys for joint selection (outside loop to prevent multiple triggers)
            if (g_lowlevel_active) {
                if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
                    if (selected_joint < 0) {
                        selected_joint = 0;
                    } else {
                        selected_joint = (selected_joint > 0) ? selected_joint - 1 : 30;
                    }
                    scroll_to_selected = true;
                }
                if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
                    if (selected_joint < 0) {
                        selected_joint = 0;
                    } else {
                        selected_joint = (selected_joint < 30) ? selected_joint + 1 : 0;
                    }
                    scroll_to_selected = true;
                }
            }

            for (int i = 0; i < 31; i++) {
                // Separator between body sections (waist | L leg | R leg | L arm | R arm | neck)
                if (i == 3 || i == 9 || i == 15 || i == 22 || i == 29) {
                    ImGui::Separator();
                }

                char label[64];
                char slider_label[64];
                const bool use_motor_space_for_joint = UseMotorSpaceForJoint(i);
                const char *name                     = use_motor_space_for_joint ? MOTOR_NAMES[i] : JOINT_NAMES[i];
                snprintf(label, sizeof(label), "%2d. %s", i, name);
                snprintf(slider_label, sizeof(slider_label), "##slider%d", i);

                if (g_lowlevel_active) {
                    // (A) LowCmd 발행 중: 슬라이더를 움직이면 target 값이 바뀌고 로봇이 실제로 따라 움직입니다.
                    // target_val 은 현재 선택된 공간(MS/PJS)의 목표값을 가리키는 포인터입니다.
                    // LOW_LEVEL mode: Show editable target positions
                    std::lock_guard<std::mutex> lock_target(g_target_mutex);
                    float *target_val = use_motor_space_for_joint ? &g_target_motor_pos[i] : &g_target_joint_pos[i];
                    float pos_min     = use_motor_space_for_joint ? MOTOR_POS_MIN[i] : JOINT_POS_MIN[i];
                    float pos_max     = use_motor_space_for_joint ? MOTOR_POS_MAX[i] : JOINT_POS_MAX[i];

                    // Selectable name - click to select for fine control
                    bool is_selected = (selected_joint == i);
                    if (is_selected) {
                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 1, 0, 1));  // Green when selected
                    }
                    if (ImGui::Selectable(label, is_selected, 0, ImVec2(name_column_width, 0))) {
                        if (selected_joint == i) {
                            selected_joint = -1;  // Deselect
                        } else {
                            selected_joint     = i;
                            scroll_to_selected = true;  // Scroll once on new selection
                        }
                    }
                    if (is_selected) {
                        ImGui::PopStyleColor();
                        if (scroll_to_selected) {
                            ImGui::SetScrollHereY(0.5f);
                            scroll_to_selected = false;
                        }
                    }

                    ImGui::SameLine(mode_column_offset);
                    RenderModeRadiosForJoint(i);
                    ImGui::SameLine(slider_column_offset);

                    // Slider (fill remaining space minus kp/kd fields) - immediate apply while dragging
                    float kp_kd_width = kp_width + kd_width + ImGui::GetStyle().ItemSpacing.x * 2;
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - kp_kd_width);
                    if (ImGui::SliderFloat(slider_label, target_val, pos_min, pos_max, "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
                        // Slider value changed - use normal duration instead of reset duration
                        g_reset_mode_active = false;
                    }

                    // Kp 입력칸. 흐름: 임시 버퍼(g_kp_edit)에 타이핑 -> Enter 또는 다른 곳 클릭 -> 확인 팝업 예약
                    // 값이 바뀌지 않았고 편집 중도 아니면, 임시 버퍼를 실제 값과 다시 맞춰 둡니다.
                    // (아래 Kd 입력칸도 동일한 방식)
                    // kp input (with confirmation popup)
                    ImGui::SameLine();
                    char kp_label[32];
                    snprintf(kp_label, sizeof(kp_label), "##kp%d", i);
                    ImGui::SetNextItemWidth(kp_width);
                    const bool kp_enter_commit =
                        ImGui::InputFloat(kp_label, &g_kp_edit[i], 0, 0, "%.1f", ImGuiInputTextFlags_EnterReturnsTrue);
                    const bool kp_focus_out_commit = ImGui::IsItemDeactivatedAfterEdit();
                    if ((kp_enter_commit || kp_focus_out_commit) && g_kp_edit[i] != g_kp[i] && !ImGui::IsPopupOpen("Confirm Gain Change")) {
                        g_pending_gain_joint      = i;
                        g_pending_is_kp           = true;
                        g_pending_old_value       = g_kp[i];
                        g_pending_new_value       = g_kp_edit[i];
                        g_show_gain_confirm_popup = true;
                    } else if (!ImGui::IsItemActive()) {
                        g_kp_edit[i] = g_kp[i];
                    }

                    // kd input (with confirmation popup)
                    ImGui::SameLine();
                    char kd_label[32];
                    snprintf(kd_label, sizeof(kd_label), "##kd%d", i);
                    ImGui::SetNextItemWidth(kd_width);
                    const bool kd_enter_commit =
                        ImGui::InputFloat(kd_label, &g_kd_edit[i], 0, 0, "%.2f", ImGuiInputTextFlags_EnterReturnsTrue);
                    const bool kd_focus_out_commit = ImGui::IsItemDeactivatedAfterEdit();
                    if ((kd_enter_commit || kd_focus_out_commit) && g_kd_edit[i] != g_kd[i] && !ImGui::IsPopupOpen("Confirm Gain Change")) {
                        g_pending_gain_joint      = i;
                        g_pending_is_kp           = false;
                        g_pending_old_value       = g_kd[i];
                        g_pending_new_value       = g_kd_edit[i];
                        g_show_gain_confirm_popup = true;
                    } else if (!ImGui::IsItemActive()) {
                        g_kd_edit[i] = g_kd[i];
                    }

                    // 선택된 관절을 좌/우 화살표로 미세 조정. 한계값(pos_min/pos_max)을 넘지 않도록 잘라냅니다.
                    // Arrow key fine adjustment when this joint is selected
                    // Shift+Arrow: 0.1, Ctrl+Arrow: 0.01, Arrow: 0.001
                    if (is_selected) {
                        float step = FINE_STEP;  // 0.001 default
                        if (ImGui::GetIO().KeyShift) {
                            step = 0.1f;
                        } else if (ImGui::GetIO().KeyCtrl) {
                            step = 0.01f;
                        }

                        if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) {
                            *target_val         = std::max(pos_min, *target_val - step);
                            g_reset_mode_active = false;  // Use normal duration
                        }
                        if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) {
                            *target_val         = std::min(pos_max, *target_val + step);
                            g_reset_mode_active = false;  // Use normal duration
                        }
                    }
                } else {
                    // (B) LowCmd 발행 중이 아님: 슬라이더는 로봇의 현재 위치를 보여주기만 합니다.
                    // 매 프레임 최신 상태값으로 덮어쓰므로 드래그해도 로봇은 움직이지 않습니다. (Kp/Kd 는 미리 수정 가능)
                    // Display mode: Show current state (read-only)
                    if (g_first_state_received) {
                        std::lock_guard<std::mutex> lock(g_lowstate_mutex);
                        g_slider_values[i] =
                            use_motor_space_for_joint ? g_latest_lowstate.motor_state()[i].q() : g_latest_lowstate.joint_state()[i].q();
                    }
                    // Use limits based on per-joint space selection
                    float pos_min = use_motor_space_for_joint ? MOTOR_POS_MIN[i] : JOINT_POS_MIN[i];
                    float pos_max = use_motor_space_for_joint ? MOTOR_POS_MAX[i] : JOINT_POS_MAX[i];

                    // Match layout with LOW_LEVEL mode
                    ImGui::SetNextItemWidth(name_column_width);
                    ImGui::Text("%s", label);

                    ImGui::SameLine(mode_column_offset);
                    RenderModeRadiosForJoint(i);
                    ImGui::SameLine(slider_column_offset);

                    // Slider (fill remaining space minus kp/kd fields)
                    float kp_kd_width = kp_width + kd_width + ImGui::GetStyle().ItemSpacing.x * 2;
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x - kp_kd_width);
                    ImGui::SliderFloat(slider_label, &g_slider_values[i], pos_min, pos_max, "%.3f");

                    // kp input (with confirmation popup)
                    ImGui::SameLine();
                    char kp_label[32];
                    snprintf(kp_label, sizeof(kp_label), "##kp%d", i);
                    ImGui::SetNextItemWidth(kp_width);
                    const bool kp_enter_commit =
                        ImGui::InputFloat(kp_label, &g_kp_edit[i], 0, 0, "%.1f", ImGuiInputTextFlags_EnterReturnsTrue);
                    const bool kp_focus_out_commit = ImGui::IsItemDeactivatedAfterEdit();
                    if ((kp_enter_commit || kp_focus_out_commit) && g_kp_edit[i] != g_kp[i] && !ImGui::IsPopupOpen("Confirm Gain Change")) {
                        g_pending_gain_joint      = i;
                        g_pending_is_kp           = true;
                        g_pending_old_value       = g_kp[i];
                        g_pending_new_value       = g_kp_edit[i];
                        g_show_gain_confirm_popup = true;
                    } else if (!ImGui::IsItemActive()) {
                        g_kp_edit[i] = g_kp[i];
                    }

                    // kd input (with confirmation popup)
                    ImGui::SameLine();
                    char kd_label[32];
                    snprintf(kd_label, sizeof(kd_label), "##kd%d", i);
                    ImGui::SetNextItemWidth(kd_width);
                    const bool kd_enter_commit =
                        ImGui::InputFloat(kd_label, &g_kd_edit[i], 0, 0, "%.2f", ImGuiInputTextFlags_EnterReturnsTrue);
                    const bool kd_focus_out_commit = ImGui::IsItemDeactivatedAfterEdit();
                    if ((kd_enter_commit || kd_focus_out_commit) && g_kd_edit[i] != g_kd[i] && !ImGui::IsPopupOpen("Confirm Gain Change")) {
                        g_pending_gain_joint      = i;
                        g_pending_is_kp           = false;
                        g_pending_old_value       = g_kd[i];
                        g_pending_new_value       = g_kd_edit[i];
                        g_show_gain_confirm_popup = true;
                    } else if (!ImGui::IsItemActive()) {
                        g_kd_edit[i] = g_kd[i];
                    }
                }
            }
            ImGui::EndChild();

            // 이번 프레임에 MS/PJS 선택이 바뀐 부위가 있으면, 그 부위의 목표값을 현재 위치로 맞춥니다 (튐 방지).
            // 프레임 시작 때 저장해 둔 prev_parallel_space 와 비교해서 변경 여부를 판단합니다.
            // Detect parallel-space changes from per-row radios and sync once.
            {
                std::lock_guard<std::mutex> lock_target(g_target_mutex);
                for (int link_idx = 0; link_idx < kParallelLinkCount; ++link_idx) {
                    if (g_parallel_space[static_cast<size_t>(link_idx)] == prev_parallel_space[static_cast<size_t>(link_idx)]) {
                        continue;
                    }
                    SyncParallelLinkTargetsToCurrentState(link_idx);
                    const bool now_motor = g_parallel_space[static_cast<size_t>(link_idx)] == static_cast<int>(SpaceSelection::MOTOR);
                    AddLog(std::string(kParallelNames[static_cast<size_t>(link_idx)]) + " space changed to " + (now_motor ? "MS" : "PJS") +
                           " (synced to current state)");
                }
            }
        }
        ImGui::EndChild();

        ImGui::SameLine();

        // ==========================================================================
        //  [가운데 패널] 연결 상태 / IMU / 보행 속도 / 발행 중인 LowCmd
        // ==========================================================================
        // Center panel: IMU State
        ImGui::BeginChild("CenterPanel", ImVec2(center_panel_width, top_panel_height), true);
        {
            // CenterPanel itself is now fixed-width (sized for LowCmd table),
            // so each sub-section just fills the panel (width = 0).

            // (1) DDS 연결 상태: 도메인, 네임스페이스, 연결 여부, 로봇 제어 상태, 실기(REAL)/시뮬(SIM) 구분.
            // RobotState 가 2초 이상 안 오면 끊긴 것으로 판단합니다.
            // DDS Connection Status — fixed-width sub-frame.
            ImGui::BeginChild("DdsConnInfo", ImVec2(0.0f, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Border);
            {
                SectionHeader("DDS Connection Status");

                auto *factory        = ChannelFactory::Instance();
                const std::string ns = factory->GetNamespace();
                ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x);
                ImGui::Text("Domain:    %d", factory->GetDomainId());
                ImGui::Text("Namespace: %s", ns.empty() ? "(none)" : ns.c_str());
                ImGui::PopTextWrapPos();
                ImGui::Separator();

                // Check connection timeout (2 seconds without message = disconnected)
                auto now          = std::chrono::steady_clock::now();
                auto elapsed      = std::chrono::duration_cast<std::chrono::milliseconds>(now - g_last_robotstate_time).count();
                bool is_connected = g_robotstate_connected && (elapsed < 2000);

                bool is_sim_env = false;
                if (is_connected) {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Status: Connected");
                    {
                        std::lock_guard<std::mutex> lock(g_robotstate_mutex);
                        ImGui::Text("Robot State: %s", RobotControlStateToString(g_latest_robotstate.state()));
                        is_sim_env = (g_latest_robotstate.environment() == RobotEnvironment::ROBOT_ENV_SIM);
                        ImGui::TextColored(is_sim_env ? ImVec4(0.5f, 0.8f, 1.0f, 1.0f) : ImVec4(1.0f, 0.5f, 0.3f, 1.0f), "Environment: %s",
                                           is_sim_env ? "SIM" : "REAL");
                    }
                } else {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Status: Disconnected");
                    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), "Robot State: --");
                    ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), "Environment: --");
                }

                // 시뮬레이터에 연결된 경우에만 나타나는 버튼들 (일시정지 / 재개 / 모델 다시 읽기 / 상태 초기화).
                // Mujoco Sim controls — only shown when bridge reports SIM.
                if (is_connected && is_sim_env) {
                    ImGui::Separator();
                    SubsectionHeader("Mujoco Sim");
                    const float spacing      = ImGui::GetStyle().ItemSpacing.x;
                    const float button_width = (ImGui::GetContentRegionAvail().x - spacing) / 2.0f;
                    if (ScaledButton("Pause", ImVec2(button_width, action_btn_height))) {
                        CallSendMujocoSimCmdAsync(&client, MujocoSimCommandType::MUJOCO_SIM_PAUSE, "PAUSE");
                    }
                    ImGui::SameLine();
                    if (ScaledButton("Resume", ImVec2(button_width, action_btn_height))) {
                        CallSendMujocoSimCmdAsync(&client, MujocoSimCommandType::MUJOCO_SIM_RESUME, "RESUME");
                    }
                    if (ScaledButton("Reload Model", ImVec2(button_width, action_btn_height))) {
                        CallSendMujocoSimCmdAsync(&client, MujocoSimCommandType::MUJOCO_SIM_RELOAD, "RELOAD");
                    }
                    ImGui::SameLine();
                    if (ScaledButton("Reset State", ImVec2(button_width, action_btn_height))) {
                        CallSendMujocoSimCmdAsync(&client, MujocoSimCommandType::MUJOCO_SIM_RESET, "RESET");
                    }
                }
            }
            ImGui::EndChild();

            ImGui::Spacing();

            // (2) IMU 값 표시: 자세(쿼터니언, Roll/Pitch/Yaw), 각속도(자이로), 가속도.
            // IMU State — same compact width as DDS section.
            ImGui::BeginChild("ImuStateBox", ImVec2(0.0f, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Border);
            {
                SectionHeader("IMU State");

                if (g_first_state_received) {
                    std::lock_guard<std::mutex> lock(g_lowstate_mutex);
                    const auto &imu          = g_latest_lowstate.imu_state();
                    const ImVec4 label_color = ImVec4(0.5f, 1.0f, 0.5f, 1.0f);

                    ImGui::TextColored(label_color, "Quaternion:");
                    ImGui::Text("  w: %8.4f   x: %8.4f   y: %8.4f   z: %8.4f", imu.quaternion()[0], imu.quaternion()[1],
                                imu.quaternion()[2], imu.quaternion()[3]);

                    ImGui::TextColored(label_color, "Gyroscope (rad/s):");
                    ImGui::Text("  x: %8.4f   y: %8.4f   z: %8.4f", imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2]);

                    ImGui::TextColored(label_color, "Accelerometer (m/s^2):");
                    ImGui::Text("  x: %8.4f   y: %8.4f   z: %8.4f", imu.accelerometer()[0], imu.accelerometer()[1], imu.accelerometer()[2]);

                    ImGui::TextColored(label_color, "Roll-Pitch-Yaw (degree):");
                    ImGui::Text("  R: %8.4f   P: %8.4f   Y: %8.4f", imu.rpy()[0], imu.rpy()[1], imu.rpy()[2]);
                } else {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Waiting for IMU data...");
                }
            }
            ImGui::EndChild();

            ImGui::Spacing();

            // (3) 보행 속도 명령. 먼저 오른쪽 패널에서 Walkmode 로 전환한 뒤 사용해야 합니다.
            // Walk Velocity (Twist) — publishes WalkTwist on rt/cmd_vel at 20Hz.
            ImGui::BeginChild("WalkTwistBox", ImVec2(0.0f, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Border);
            {
                SectionHeader("Walk Velocity (rt/cmd_vel)");
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "Switch to walkmode first (Control Mode > Walkmode).");
                ImGui::Spacing();

                const float step_btn_w = 36.0f * ui_scale;
                // 속도 한 축(한 줄)을 그리는 람다: [이름] [-] 현재값 [+]. 버튼을 누를 때마다 step 만큼 증감합니다.
                // One steer axis row: label, [-], current value, [+].
                auto twist_row = [&](const char *label, std::atomic<double> &val, double step, const char *unit) {
                    char minus_id[40];
                    char plus_id[40];
                    std::snprintf(minus_id, sizeof(minus_id), "-##%s", label);
                    std::snprintf(plus_id, sizeof(plus_id), "+##%s", label);
                    ImGui::Text("%-10s", label);
                    ImGui::SameLine();
                    if (ImGui::Button(minus_id, ImVec2(step_btn_w, 0))) {
                        val.store(val.load() - step);
                    }
                    ImGui::SameLine();
                    ImGui::Text("% .2f %-5s", val.load(), unit);
                    ImGui::SameLine();
                    if (ImGui::Button(plus_id, ImVec2(step_btn_w, 0))) {
                        val.store(val.load() + step);
                    }
                };
                twist_row("linear.x", g_twist_vx, TWIST_STEP_LIN, "m/s");
                twist_row("linear.y", g_twist_vy, TWIST_STEP_LIN, "m/s");
                twist_row("angular.z", g_twist_wz, TWIST_STEP_ANG, "rad/s");

                ImGui::Spacing();
                if (ScaledButton("Zero", ImVec2(step_btn_w * 3.0f, action_btn_height))) {
                    g_twist_vx.store(0.0);
                    g_twist_vy.store(0.0);
                    g_twist_wz.store(0.0);
                }
                ImGui::SameLine();
                if (!g_twist_publishing) {
                    if (ScaledButton("Publish cmd_vel Start", ImVec2(-1, action_btn_height))) {
                        g_twist_publishing = true;
                        AddLog("WalkTwist publishing started (rt/cmd_vel, 20Hz)");
                    }
                } else {
                    if (ScaledButton("Publish cmd_vel Stop", ImVec2(-1, action_btn_height))) {
                        // Zero the velocity; the publish thread sends these zeros
                        // for 1s so the robot decelerates to a halt.
                        g_twist_vx.store(0.0);
                        g_twist_vy.store(0.0);
                        g_twist_wz.store(0.0);
                        g_twist_publishing = false;
                        AddLog("WalkTwist stopped (sending zero velocity for 1s)");
                    }
                }
                if (g_twist_publishing) {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Status: PUBLISHING (%u msgs)", g_twist_publish_count.load());
                } else if (g_twist_flushing) {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "Status: stopping (zero velocity)...");
                } else {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Status: idle");
                }
            }
            ImGui::EndChild();

            ImGui::Spacing();

            // (4) 지금 발행 중인 LowCmd 내용을 표로 표시.
            // LowCmd Publish Info — same width, fills remaining vertical space.
            ImGui::BeginChild("LowCmdPubBox", ImVec2(0.0f, 0), ImGuiChildFlags_Border);
            {
                SectionHeader("LowCmd Publishing");

                if (g_lowlevel_active) {
                    ImGui::TextColored(ImVec4(0, 1, 0, 1), "Status: ACTIVE");
                    ImGui::Text("Published: %u msgs", g_lowcmd_publish_count.load());
                    ImGui::Separator();

                    // Tab bar for LowCmd and Cmd vs State comparison
                    if (ImGui::BeginTabBar("CmdStateTabBar")) {
                        constexpr ImGuiTableFlags kTableFlags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                                                ImGuiTableFlags_BordersOuter | ImGuiTableFlags_ScrollY |
                                                                ImGuiTableFlags_SizingFixedFit;

                        // 탭 1: 마지막으로 보낸 명령값 그대로 (위치 q, 토크 tau, kp, kd).
                        // Tab 1: LowCmd only
                        if (ImGui::BeginTabItem("LowCmd")) {
                            std::lock_guard<std::mutex> lock(g_last_cmd_mutex);
                            if (ImGui::BeginTable("LowCmdTable", 6, kTableFlags)) {
                                ImGui::TableSetupScrollFreeze(0, 1);
                                ImGui::TableSetupColumn("#");
                                ImGui::TableSetupColumn("Joint");
                                ImGui::TableSetupColumn("q (rad)");
                                ImGui::TableSetupColumn("tau (Nm)");
                                ImGui::TableSetupColumn("kp");
                                ImGui::TableSetupColumn("kd");
                                ImGui::TableHeadersRow();
                                for (int i = 0; i < 31; i++) {
                                    const auto &cmd = g_last_published_cmd.motors()[i];
                                    ImGui::TableNextRow();
                                    ImGui::TableSetColumnIndex(0);
                                    ImGui::Text("%d", i);
                                    ImGui::TableSetColumnIndex(1);
                                    ImGui::TextUnformatted(JOINT_NAMES[i]);
                                    ImGui::TableSetColumnIndex(2);
                                    ImGui::Text("%7.3f", cmd.q());
                                    ImGui::TableSetColumnIndex(3);
                                    ImGui::Text("%6.2f", cmd.tau());
                                    ImGui::TableSetColumnIndex(4);
                                    ImGui::Text("%5.1f", cmd.kp());
                                    ImGui::TableSetColumnIndex(5);
                                    ImGui::Text("%4.2f", cmd.kd());
                                }
                                ImGui::EndTable();
                            }
                            ImGui::EndTabItem();
                        }

                        // 탭 2: 보낸 명령(cmd)과 실제 상태(state)를 비교. 차이(diff)가 0.05rad 를 넘으면 해당 줄을 빨갛게 표시합니다.
                        // 로봇이 명령을 잘 따라오고 있는지 한눈에 확인하는 용도입니다.
                        // 명령을 MS 로 보냈으면 상태도 모터 값과, PJS 로 보냈으면 관절 값과 비교해야 의미가 있습니다.
                        // Tab 2: Cmd vs State comparison.
                        // Each parallel-link joint can be commanded in motor (MS) or
                        // joint (PJS) space; the state we compare against must be in
                        // the SAME space, otherwise the diff is meaningless.
                        if (ImGui::BeginTabItem("Cmd vs State")) {
                            std::lock_guard<std::mutex> lock_cmd(g_last_cmd_mutex);
                            std::lock_guard<std::mutex> lock_state(g_lowstate_mutex);
                            if (ImGui::BeginTable("CmdStateTable", 6, kTableFlags)) {
                                ImGui::TableSetupScrollFreeze(0, 1);
                                ImGui::TableSetupColumn("#");
                                ImGui::TableSetupColumn("Joint");
                                ImGui::TableSetupColumn("Space");
                                ImGui::TableSetupColumn("cmd (rad)");
                                ImGui::TableSetupColumn("state (rad)");
                                ImGui::TableSetupColumn("diff");
                                ImGui::TableHeadersRow();
                                for (int i = 0; i < 31; i++) {
                                    // Non-parallel-link joints have no MS/PJS distinction
                                    // (motor space and joint space coincide), so tag them
                                    // "MS/PJS" instead of forcing one of the two labels.
                                    const bool has_parallel    = (GetParallelLinkIndexForJoint(i) >= 0);
                                    const bool use_motor_space = UseMotorSpaceForJoint(i);
                                    const char *space_tag      = has_parallel ? (use_motor_space ? "MS" : "PJS") : "MS/PJS";
                                    const char *display_name   = (has_parallel && use_motor_space) ? MOTOR_NAMES[i] : JOINT_NAMES[i];
                                    const float cmd_q          = g_last_published_cmd.motors()[i].q();
                                    const float state_q =
                                        use_motor_space ? g_latest_lowstate.motor_state()[i].q() : g_latest_lowstate.joint_state()[i].q();
                                    const float diff      = cmd_q - state_q;
                                    const bool out_of_tol = std::abs(diff) > 0.05f;

                                    ImGui::TableNextRow();
                                    if (out_of_tol) {
                                        // Tint the row to flag a deviation > 0.05 rad.
                                        ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0,
                                                               ImGui::GetColorU32(ImVec4(0.5f, 0.1f, 0.1f, 0.35f)));
                                        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
                                    }

                                    ImGui::TableSetColumnIndex(0);
                                    ImGui::Text("%d", i);
                                    ImGui::TableSetColumnIndex(1);
                                    ImGui::TextUnformatted(display_name);
                                    ImGui::TableSetColumnIndex(2);
                                    ImGui::TextUnformatted(space_tag);
                                    ImGui::TableSetColumnIndex(3);
                                    ImGui::Text("%7.3f", cmd_q);
                                    ImGui::TableSetColumnIndex(4);
                                    ImGui::Text("%7.3f", state_q);
                                    ImGui::TableSetColumnIndex(5);
                                    ImGui::Text("%+7.3f", diff);

                                    if (out_of_tol) {
                                        ImGui::PopStyleColor();
                                    }
                                }
                                ImGui::EndTable();
                            }
                            ImGui::EndTabItem();
                        }

                        ImGui::EndTabBar();
                    }
                } else {
                    ImGui::TextColored(ImVec4(1, 0, 0, 1), "Status: INACTIVE");
                    ImGui::Text("Press LowCmd Publish Start to begin");
                }
            }
            ImGui::EndChild();
        }
        ImGui::EndChild();

        ImGui::SameLine();

        // ==========================================================================
        //  [오른쪽 패널] 서비스 API 버튼
        // ==========================================================================
        // 모든 버튼이 같은 패턴입니다:
        //   버튼 클릭 -> RequestServiceConfirm(이름, 실행할 람다) -> 확인 팝업 -> Confirm 시 람다 실행 -> 비동기 서비스 호출
        // 실제 로봇이 움직이거나 전원 상태가 바뀌는 명령이라 반드시 확인 단계를 거칩니다.
        //
        //   Power & Torque : BMS(배터리)/모터 초기화, 전원 OFF, 토크 ON/OFF
        //   Hand           : 손 초기화
        //   Control Mode   : 제어 모드 전환 (슬라이더로 관절을 직접 움직이려면 Low-Level 로 전환)
        //   Custom Modes   : 사용자 정의 모드 1~10
        //   Motion Presets : 미리 정의된 자세로 이동 (HOME, WALK_STANDBY)
        // Right panel: Service API buttons
        ImGui::BeginChild("RightPanel", ImVec2(right_panel_width, top_panel_height), true);
        {
            SectionHeader("Service API Commands");

            // --- Power & Torque -------------------------------------------------
            SubsectionHeader("Power & Torque");
            if (ScaledButton("Init BMS", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Init BMS", [&client]() { CallInitBmsAsync(&client, BmsInitType::BMS_INIT, "BMS_INIT"); });
            }
            if (ScaledButton("Init Motor", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Init Motor", [&client]() { CallInitBmsAsync(&client, BmsInitType::MOTOR_INIT, "MOTOR_INIT"); });
            }
            if (ScaledButton("Init BMS + Motor", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Init BMS + Motor",
                                      [&client]() { CallInitBmsAsync(&client, BmsInitType::BMS_AND_MOTOR_INIT, "BMS_AND_MOTOR_INIT"); });
            }
            if (ScaledButton("BMS OFF", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("BMS OFF", [&client]() { CallInitBmsAsync(&client, BmsInitType::BMS_INIT_NONE, "BMS_INIT_NONE"); });
            }
            if (ScaledButton("Torque ON", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Torque ON", [&client]() { CallSetTorqueAsync(&client, TorqueType::TORQUE_ON, "TORQUE_ON"); });
            }
            if (ScaledButton("Torque OFF", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Torque OFF", [&client]() { CallSetTorqueAsync(&client, TorqueType::TORQUE_OFF, "TORQUE_OFF"); });
            }

            // --- Hand -----------------------------------------------------------
            SubsectionHeader("Hand");
            if (ScaledButton("Init Hand", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Init Hand", [&client]() { CallInitHandAsync(&client); });
            }

            // --- Control Mode ---------------------------------------------------
            SubsectionHeader("Control Mode");
            if (ScaledButton("High-Level", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Control Mode: High-Level", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_HIGH_LEVEL_JOINT_CONTROL,
                                                    "HIGH_LEVEL_JOINT_CONTROL");
                });
            }
            if (ScaledButton("Low-Level", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Control Mode: Low-Level", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_LOW_LEVEL_JOINT_CONTROL,
                                                    "LOW_LEVEL_JOINT_CONTROL");
                });
            }
            if (ScaledButton("Joint Position Hold", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Joint Position Hold", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_JOINT_POSITION_HOLD,
                                                    "JOINT_POSITION_HOLD");
                });
            }
            if (ScaledButton("Motion Stop", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Motion Stop", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_MOTION_STOP, "MOTION_STOP");
                });
            }
            if (ScaledButton("Walkmode", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Walkmode", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_WALKMODE_ON, "WALKMODE_ON");
                });
            }
            if (ScaledButton("Low-Level Walkmode", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Low-Level Walkmode", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_LOW_LEVEL_WALKMODE_ON,
                                                    "LOW_LEVEL_WALKMODE_ON");
                });
            }
            if (ScaledButton("High-Level Walkmode", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("High-Level Walkmode", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_HIGH_LEVEL_WALKMODE_ON,
                                                    "HIGH_LEVEL_WALKMODE_ON");
                });
            }

            // --- Custom Modes ---------------------------------------------------
            SubsectionHeader("Custom Modes");
            if (ScaledButton("Custom Mode 1", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 1", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_1, "CUSTOM_MODE_1");
                });
            }
            if (ScaledButton("Custom Mode 2", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 2", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_2, "CUSTOM_MODE_2");
                });
            }
            if (ScaledButton("Custom Mode 3", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 3", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_3, "CUSTOM_MODE_3");
                });
            }
            if (ScaledButton("Custom Mode 4", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 4", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_4, "CUSTOM_MODE_4");
                });
            }
            if (ScaledButton("Custom Mode 5", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 5", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_5, "CUSTOM_MODE_5");
                });
            }
            if (ScaledButton("Custom Mode 6", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 6", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_6, "CUSTOM_MODE_6");
                });
            }
            if (ScaledButton("Custom Mode 7", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 7", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_7, "CUSTOM_MODE_7");
                });
            }
            if (ScaledButton("Custom Mode 8", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 8", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_8, "CUSTOM_MODE_8");
                });
            }
            if (ScaledButton("Custom Mode 9", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 9", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_9, "CUSTOM_MODE_9");
                });
            }
            if (ScaledButton("Custom Mode 10", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Custom Mode 10", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_CUSTOM_MODE_10, "CUSTOM_MODE_10");
                });
            }

            // --- Motion Presets -------------------------------------------------
            SubsectionHeader("Motion Presets");
            if (ScaledButton("HOME", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Motion Preset: HOME", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_MOTION_PRESET, "MOTION_PRESET(HOME)",
                                                    "HOME", false);
                });
            }
            if (ScaledButton("WALKMODE STANDBY", ImVec2(-1, action_btn_height))) {
                RequestServiceConfirm("Motion Preset: WALKMODE STANDBY", [&client]() {
                    CallSendControlModeCommandAsync(&client, ControlModeCommandType::CONTROL_MODE_CMD_MOTION_PRESET,
                                                    "MOTION_PRESET(WALK_STANDBY)", "WALK_STANDBY", false);
                });
            }
        }
        ImGui::EndChild();

        // ==========================================================================
        //  [하단] 응답 로그 창
        // ==========================================================================
        // 스크롤이 맨 아래에 있을 때만 새 로그를 따라 자동 스크롤합니다. (위로 올려서 읽는 중이면 그대로 둠)
        // Bottom log strip: full window width below the three panels.
        ImGui::BeginChild("LogPanel", ImVec2(0, 0), true);
        {
            SectionHeader("Response Log");

            ImGui::BeginChild("LogScroll", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);
            {
                std::lock_guard<std::mutex> lock(g_log_mutex);
                for (const auto &line : g_response_log) {
                    ImGui::TextUnformatted(line.c_str());
                }
            }
            // Auto-scroll to bottom
            if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
                ImGui::SetScrollHereY(1.0f);
            ImGui::EndChild();
        }
        ImGui::EndChild();

        // ==========================================================================
        //  확인 팝업 2종 (화면 중앙에 뜨는 모달 창)
        // ==========================================================================
        // [서비스 명령 확인] Confirm 또는 Enter: 담아둔 동작 실행 / Cancel 또는 Esc: 취소
        // 팝업이 열린 바로 그 프레임의 Enter 입력은 무시합니다 (팝업을 연 Enter 가 곧바로 확인으로 처리되는 것 방지).
        // Service-command confirmation popup (Power/Torque + Control Mode)
        if (g_show_service_confirm_popup) {
            ImGui::OpenPopup("Confirm Service Command");
            g_show_service_confirm_popup = false;
        }

        ImVec2 svc_center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(svc_center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal("Confirm Service Command", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::TextUnformatted("Execute the following command?");
            ImGui::Spacing();
            ImGui::SetWindowFontScale(1.3f);
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.4f, 1.0f), "%s", g_pending_service_label.c_str());
            ImGui::SetWindowFontScale(1.0f);
            ImGui::Spacing();
            ImGui::Separator();
            ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "This affects the live robot.");
            ImGui::Separator();

            // Focus Confirm so the popup is keyboard-friendly: Enter to OK,
            // Esc to cancel.
            const bool popup_just_opened = ImGui::IsWindowAppearing();
            if (popup_just_opened) {
                ImGui::SetKeyboardFocusHere();
            }
            const bool enter_pressed =
                !popup_just_opened && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter));
            const bool escape_pressed = ImGui::IsKeyPressed(ImGuiKey_Escape);

            if (ScaledButton("Confirm", ImVec2(popup_button_width, action_btn_height)) || enter_pressed) {
                if (g_pending_service_action) {
                    g_pending_service_action();
                }
                g_pending_service_action = nullptr;
                g_pending_service_label.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ScaledButton("Cancel", ImVec2(popup_button_width, action_btn_height)) || escape_pressed) {
                g_pending_service_action = nullptr;
                g_pending_service_label.clear();
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        // [게인 변경 확인] Apply: 임시 값을 실제 게인(g_kp/g_kd)에 반영 / Cancel: 입력칸을 이전 값으로 되돌림
        // Kp/Kd change confirmation popup
        if (g_show_gain_confirm_popup) {
            ImGui::OpenPopup("Confirm Gain Change");
            g_show_gain_confirm_popup = false;
        }

        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal("Confirm Gain Change", NULL, ImGuiWindowFlags_AlwaysAutoResize)) {
            const char *gain_name                = g_pending_is_kp ? "Kp" : "Kd";
            const bool use_motor_space_for_joint = UseMotorSpaceForJoint(g_pending_gain_joint);
            const char *joint_name = use_motor_space_for_joint ? MOTOR_NAMES[g_pending_gain_joint] : JOINT_NAMES[g_pending_gain_joint];

            ImGui::Text("Change %s gain for Joint %d (%s)?", gain_name, g_pending_gain_joint, joint_name);
            ImGui::Separator();
            ImGui::Text("Current: %.2f", g_pending_old_value);
            ImGui::Text("New:     %.2f", g_pending_new_value);
            ImGui::Separator();

            const bool popup_just_opened = ImGui::IsWindowAppearing();
            if (popup_just_opened) {
                ImGui::SetKeyboardFocusHere();
            }
            const bool enter_pressed =
                !popup_just_opened && (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter));
            if (ScaledButton("Apply", ImVec2(popup_button_width, action_btn_height)) || enter_pressed) {
                if (g_pending_is_kp) {
                    g_kp[g_pending_gain_joint]      = g_pending_new_value;
                    g_kp_edit[g_pending_gain_joint] = g_pending_new_value;
                } else {
                    g_kd[g_pending_gain_joint]      = g_pending_new_value;
                    g_kd_edit[g_pending_gain_joint] = g_pending_new_value;
                }
                AddLog("Gain changed: Joint " + std::to_string(g_pending_gain_joint) + " " + gain_name + " = " +
                       std::to_string(g_pending_new_value));
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ScaledButton("Cancel", ImVec2(popup_button_width, action_btn_height))) {
                // Revert edit buffer
                if (g_pending_is_kp) {
                    g_kp_edit[g_pending_gain_joint] = g_pending_old_value;
                } else {
                    g_kd_edit[g_pending_gain_joint] = g_pending_old_value;
                }
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::End();

        // ---- 실제 화면 출력: 배경을 지우고, 위에서 구성한 UI 를 OpenGL 로 그린 뒤 버퍼 교체 ----
        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // ============================================================================
    //  종료 정리 (순서 중요: 스레드 종료 -> 구독자 해제 -> ImGui -> 창)
    // ============================================================================
    // Ensure background publish loop exits before joining.
    g_running = false;

    // Cleanup
    std::cout << "\nShutting down GUI..." << std::endl;

    // Wait for publish threads to finish
    if (publish_thread.joinable()) {
        publish_thread.join();
    }
    if (twist_thread.joinable()) {
        twist_thread.join();
    }

    // 전역 변수인 RobotState 구독자는 여기서 직접 해제합니다.
    // 프로그램 종료 시 전역 객체들의 소멸 순서는 정해져 있지 않아, 그냥 두면 통신 기반이 먼저 사라진 뒤
    // 구독자가 그것을 사용하려다 크래시가 날 수 있기 때문입니다.
    // Tear the global RobotState subscriber down explicitly so it does not
    // race with ChannelFactory's static-storage destruction (the undefined
    // order otherwise leads to use-after-free and SIGABRT).
    if (g_robotstate_sub) {
        g_robotstate_sub->stop();
        g_robotstate_sub.reset();
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    std::cout << "GUI Client terminated" << std::endl;
    return 0;
}
