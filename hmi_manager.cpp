#include <signal.h>
            
#include "hmi_manager/hmi_manager.hpp"             
#include "hmi_manager/hmi_manager_comm.hpp"


#include <memory>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <map>
#include <atomic>

using namespace std::chrono_literals;

void sig_handler(sig_atomic_t s){           
    RCLCPP_INFO(rclcpp::get_logger("Hmi_Manager"), "Caught Signal '%d' And Exit", s);       // RCLCPP에서 호출되는 종로 시그널을 받아 프로그램을 즉시 종료하는 함수
    exit(1);
}

int main(int argc, char* argv[])
{   
    signal (SIGINT,sig_handler);

    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;

    std::shared_ptr<hmi_manager::HmiComm> hminode = std::make_shared<hmi_manager::HmiComm>("hmi_manager");
    
    hminode->on_configure("/dev/ttyUSB0", 115200, 2000);

    executor.add_node(hminode);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
