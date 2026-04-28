#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>
#include <thread>
#include <csignal>

// SDK 헤더 (사용자 환경에 맞게 경로 확인 필요)
#include "igris_sdk/channel_factory.hpp"
#include "igris_sdk/subscriber.hpp"
#include "igris_sdk/types.hpp"

using namespace igris_sdk;

// 전역 플래그 (Ctrl+C 종료용)
bool g_running = true;
void signalHandler(int signum) {
    g_running = false;
}

/**
 * @brief LowState 콜백 함수
 * 실시간으로 로봇의 상태 정보를 받아 터미널에 출력합니다.
 */
void LowStateCallback(const LowState &state) {
    // 터미널 화면 초기화 (ANSI escape code)
    std::cout << "\033[2J\033[1;1H"; 
    
    std::cout << "==========================================================================================" << std::endl;
    std::cout << "                         IGRIS SDK Motor State Monitor (Real-time)" << std::endl;
    std::cout << "==========================================================================================" << std::endl;
    
    // 테이블 헤더 출력
    std::cout << std::setw(4) << "ID" 
              << std::setw(12) << "Pos(rad)" 
              << std::setw(12) << "Vel(rad/s)" 
              << std::setw(12) << "Torque(Nm)" 
              << std::setw(10) << "Temp(C)" 
              << std::setw(15) << "StatusBits" << std::endl;
    std::cout << "------------------------------------------------------------------------------------------" << std::endl;

    // 31개 모터 데이터 순회
    for (int i = 0; i < 31; ++i) {
        const auto& motor = state.motor_state()[i];
        
        std::cout << std::fixed << std::setprecision(3)
                  << std::setw(4)  << i
                  << std::setw(12) << motor.q()           // 현재 위치
                  << std::setw(12) << motor.dq()          // 현재 속도
                  << std::setw(12) << motor.tau_est()     // 추정 토크
                  << std::setw(10) << (int)motor.temperature() // 모터 온도
                  << "    0x" << std::hex << std::setw(8) << std::setfill('0') 
                  << motor.status_bits() << std::dec << std::setfill(' ') // 상태 비트
                  << std::endl;
    }
    
    std::cout << "------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Press Ctrl+C to exit..." << std::endl;
}

int main(int argc, char** argv) {
    // 1. 도메인 ID 설정 (기본값 0)
    int domain_id = 0;
    if (argc > 1) {
        domain_id = std::stoi(argv[1]);
    }

    // 2. 종료 시그널 등록
    signal(SIGINT, signalHandler);

    // 3. SDK 초기화
    ChannelFactory::Instance()->Init(domain_id);

    // 4. Subscriber 초기화 (rt/lowstate 토픽 구독)
    // LowState 내부의 motor_state()가 요청하신 q, dq, tau_est 등을 포함합니다.
    Subscriber<LowState> lowstate_sub("rt/lowstate");
    lowstate_sub.init(LowStateCallback);

    std::cout << "Starting Motor Monitor on Domain ID: " << domain_id << "..." << std::endl;

    // 5. 메인 루프 (콜백 대기)
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 화면 갱신 주기를 위해 대기
    }

    std::cout << "\nShutting down..." << std::endl;
    return 0;
}