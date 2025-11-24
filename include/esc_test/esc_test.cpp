#include "esc_test.hpp"

ESCTest::ESCTest(const char *interface_name, int num_esc)
: num_esc_{num_esc}
{

    printf("Initializing ESC Test Node...\n");

    canard_interface_.init(interface_name);
    uint8_t node_id = canard_interface_.get_node_id();
    
    printf("ESC Test Node initialized with %d ESCs\n", num_esc_);

    // Display the number of ESC and Drone can info
    std::cout<<"Number of ESC from launch file: "<<num_esc_<<std::endl;
    printf("Drone Can started on %s, node ID: %d\n",
    interface_name,
    canard_interface_.get_node_id());

    process_canard_thread_ = std::thread(&ESCTest::process_canard, this);
    send_node_status_thread_ = std::thread(&ESCTest::send_NodeStatus, this);


    for(size_t i = 0; i < num_esc_; i++) 
    {
        printf("Requesting restart for node %ld\n", i+2);
        uavcan_protocol_RestartNodeRequest restart_req{};
        restart_req.magic_number = UAVCAN_PROTOCOL_RESTARTNODE_REQUEST_MAGIC_NUMBER;
        while(!restart_node_client_.request(i+2, restart_req))
        {
            printf("Retrying RestartNode request for node %ld\n", i+2);
        };
    }

    std::this_thread::sleep_for(std::chrono::seconds(10));

    uavcan_protocol_GetNodeInfoRequest req;

    for(size_t i = 0; i < num_esc_; i++) 
    {
        printf("Requesting node info for node %ld\n", i+2);
        req = {};
        while(!get_node_info_client_.request(i+2,req))
        {
            printf("Retrying GetNodeInfo request for node %ld\n", i+2);
        };
    }

    send_esc_command_thread_ = std::thread(&ESCTest::send_ESC_Command, this);


    esc_cmd_vec_.resize(num_esc_);

    // uavcan_equipment_esc_RawCommand esc_cmd_msg{};
    // esc_cmd_msg.cmd.len = num_esc_;

    // for(size_t i = 0; i < num_esc_; i++)
    // {
    //     esc_cmd_vec_[i] = 10.0;
    //     esc_cmd_msg.cmd.data[i] = static_cast<int16_t>(esc_cmd_vec_[i]*8191.0/9800.0);
    // }

    // esc_cmd_pub_.broadcast(esc_cmd_msg);
    // printf("Sent initial ESC Commands\n");

}

ESCTest::~ESCTest()
{
    process_canard_thread_.join();
}

void ESCTest::process_canard()
{
    // Process incoming CAN messages
    while(1)
    {
        canard_interface_.process(1);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void ESCTest::handle_esc_status(const CanardRxTransfer &transfer,
                               const uavcan_equipment_esc_Status &msg)
{
    // Process ESC status message
    // printf("Received ESC Status from ESC Index: %d\n", msg.esc_index);
    // printf("  Voltage: %.2f V\n", msg.voltage);
    // printf("  Current: %.2f A\n", msg.current);
    // printf("  Temperature: %.2f C\n", msg.temperature);
    // printf("  RPM: %d\n", msg.rpm);
    // printf("  Error Count: %u\n", msg.error_count);

}

void ESCTest::handle_getNodeInfo(const CanardRxTransfer& transfer,
                            const uavcan_protocol_GetNodeInfoResponse &rsp)
{
    printf("Got GetNodeInfo response\n");
    printf("ESC Name: ");
    for(int i = 0; i < rsp.name.len; i++)
    {
        printf("%c", rsp.name.data[i]);
    }
    printf("\n");
}

void ESCTest::handle_restart_node(const CanardRxTransfer& transfer,
                             const uavcan_protocol_RestartNodeResponse &rsp)
{

    printf("Got RestartNode response\n");

    if(rsp.ok)
    {
        printf("Node restarted successfully\n");
    }
    else
    {
        printf("Node restart failed\n");
    }
}

void ESCTest::send_ESC_Command()
{
    double t_acc = (vel_max_ - rpm_init_) / acc_max_;

    auto time_now = std::chrono::steady_clock::now();
    while(1)
    {
        auto time_elapsed = std::chrono::steady_clock::now() - time_now;
        double t_sec = std::chrono::duration<double>(time_elapsed).count();



        uavcan_equipment_esc_RawCommand cmd_msg;
        cmd_msg.cmd.len = num_esc_;


        if(t_sec < t_start0_)
        {
            for(size_t i = 0; i < num_esc_; i++)
            {
                esc_cmd_vec_[i] = 10;
                cmd_msg.cmd.data[i] = static_cast<int16_t>(esc_cmd_vec_[i]*8191.0/9800.0);
            }
        }
        else if(t_sec < t_start1_)
        {
            for(size_t i = 0; i < num_esc_; i++)
            {
                cmd_msg.cmd.data[i] = 0;
            }
        }
        else if(t_sec < t_start2_)
        {
            for(size_t i = 0; i < num_esc_; i++)
            {
                esc_cmd_vec_.at(i) = rpm_init_;
                cmd_msg.cmd.data[i] = static_cast<int16_t>(esc_cmd_vec_.at(i)*8191.0/9800.0);
            }
        }
        else if(t_sec < t_start2_ + t_acc)
        {
            double t_in_acc = t_sec - t_start2_;
            for(size_t i = 0; i < num_esc_; i++)
            {
                esc_cmd_vec_.at(i) = rpm_init_ + acc_max_ * t_in_acc;
                if(esc_cmd_vec_.at(i) > vel_max_)
                {
                    esc_cmd_vec_.at(i) = vel_max_;
                }
                cmd_msg.cmd.data[i] = static_cast<int16_t>(esc_cmd_vec_.at(i)*8191.0/9800.0);
            }
        }
        else if (t_sec < t_start2_ + t_acc + t_cruise_)
        {
            for(size_t i = 0; i < num_esc_; i++)
            {
                esc_cmd_vec_.at(i) = vel_max_;
                cmd_msg.cmd.data[i] = static_cast<int16_t>(esc_cmd_vec_.at(i)*8191.0/9800.0);
            }
        }
        else if(t_sec < t_start2_ + 2 * t_acc + t_cruise_)
        {
            double t_in_dec = t_sec - (t_start2_ + t_acc + t_cruise_);
            for(size_t i = 0; i < num_esc_; i++)
            {
                esc_cmd_vec_.at(i) = vel_max_ - acc_max_ * t_in_dec;
                if(esc_cmd_vec_.at(i) < rpm_init_)
                {
                    esc_cmd_vec_.at(i) = rpm_init_;
                }
                cmd_msg.cmd.data[i] = static_cast<int16_t>(esc_cmd_vec_.at(i)*8191.0/9800.0);
            }
        }
        else if(t_sec < t_start2_ + 2 * t_acc + t_cruise_ + 5.0)
        {
            for(size_t i = 0; i < num_esc_; i++)
            {
                esc_cmd_vec_.at(i) = rpm_init_;
                cmd_msg.cmd.data[i] = static_cast<int16_t>(esc_cmd_vec_.at(i)*8191.0/9800.0);
            }
        }
        else
        {
            for(size_t i = 0; i < num_esc_; i++)
            {
                cmd_msg.cmd.data[i] = 0;
            }
        }


        esc_cmd_pub_.broadcast(cmd_msg);
        printf("Time elapsed: %.2f seconds\n", t_sec);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}

void ESCTest::send_NodeStatus()
{
    while(1)
    {
        uavcan_protocol_NodeStatus uavcan_node_status_msg{};
        uavcan_node_status_msg.vendor_specific_status_code = 0;
        uavcan_node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
        uavcan_node_status_msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
        uavcan_node_status_msg.sub_mode = 0;
        uavcan_node_status_msg.uptime_sec = millis32() / 1000UL;
        node_status_pub_.broadcast(uavcan_node_status_msg);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}