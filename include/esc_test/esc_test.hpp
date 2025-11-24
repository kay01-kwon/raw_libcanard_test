#ifndef ESC_TEST_HPP
#define ESC_TEST_HPP

#include "canard_impl/canard_interface/canard_interface.hpp"
#include "dsdl_generated/dronecan_msgs.h"

#include <iostream>
#include <vector>

#include <chrono>
#include <thread>

class ESCTest
{
    public:

    ESCTest(const char *interface_name, int num_esc);

    ~ESCTest();

    private:

    void process_canard();

    // 1. Declare CANARD publisher
    CanardInterface canard_interface_{0};

    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub_{canard_interface_};

    Canard::Publisher<uavcan_equipment_esc_RawCommand> esc_cmd_pub_{canard_interface_};


    // 2. ESC status
    void handle_esc_status(const CanardRxTransfer &transfer,
                           const uavcan_equipment_esc_Status &msg);
    
    Canard::ObjCallback<ESCTest, uavcan_equipment_esc_Status> 
    esc_status_cb_{this, &ESCTest::handle_esc_status};

    Canard::Subscriber<uavcan_equipment_esc_Status>
    esc_status_sub_{esc_status_cb_, 0};

    // 3. Node handler
    
    // 3.1 Handler for getNodeInfo
    void handle_getNodeInfo(const CanardRxTransfer& transfer,
                            const uavcan_protocol_GetNodeInfoResponse &rsp);

    // 3.2 Object callback for getNodeInfo
    Canard::ObjCallback<ESCTest, uavcan_protocol_GetNodeInfoResponse>
    get_node_info_cb_{this, &ESCTest::handle_getNodeInfo};

    // 3.3 Declare Client
    Canard::Client<uavcan_protocol_GetNodeInfoResponse>
    get_node_info_client_{canard_interface_, get_node_info_cb_};


    // 4. Restart Node handler
    void handle_restart_node(const CanardRxTransfer& transfer,
                             const uavcan_protocol_RestartNodeResponse &rsp);

    Canard::ObjCallback<ESCTest, uavcan_protocol_RestartNodeResponse>
    restart_node_cb_{this, &ESCTest::handle_restart_node};

    Canard::Client<uavcan_protocol_RestartNodeResponse>
    restart_node_client_{canard_interface_, restart_node_cb_}; 

    
    void send_ESC_Command();

    void send_NodeStatus();

    uavcan_equipment_esc_RawCommand esc_cmd_msg_{};
    uavcan_protocol_NodeStatus node_status_msg_{};

    int num_esc_{1};

    size_t esc_count_{0};

    float voltage_{0.0f};

    std::vector<double> esc_cmd_vec_;

    double acc_max_{15000.0}; // Maximum acceleration
    double vel_max_{8000.0}; // Maximum velocity
    double rpm_init_{2000.0};
    double rpm_cmd_{2000.0}; // Commanded RPM

    double t_start0_{2.0};
    double t_start1_{5.0};
    double t_start2_{10.0};
    double t_cruise_{1.0};


    std::thread process_canard_thread_;
    std::thread send_node_status_thread_;
    std::thread send_esc_command_thread_;


};

#endif  //ESC_TEST_HPP