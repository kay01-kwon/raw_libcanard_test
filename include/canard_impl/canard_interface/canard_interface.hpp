#ifndef CANARD_INTERFACE_HPP
#define CANARD_INTERFACE_HPP

// system includes
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <chrono>

// include the canard C++ APIs
#include "canard/publisher.h"
#include "canard/subscriber.h"
#include "canard/service_client.h"
#include "canard/service_server.h"
#include "canard/handler_list.h"
#include "canard/transfer_object.h"

// include the base canard API
#include "canard_internals/canard.h"

// include the interface
#include "driver/socketcan.h"


static uint64_t micros64()
{
    static uint64_t first_us;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t tus = (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
    if(first_us == 0)
    {
        first_us = tus;
    }

    return tus - first_us;

}

static uint32_t millis32()
{
    return micros64() / 1000ULL;
}

class CanardInterface : public Canard::Interface{


    public:

        CanardInterface(uint8_t iface_index)
        : Interface(iface_index)
        {}

        // Implement the Canard::Interface pure virtual functions
        void init(const char *interface_name);

        bool broadcast(const Canard::Transfer &transfer) override;
        
        bool request(uint8_t dest_node_id,
                    const Canard::Transfer &transfer) override;

        bool respond(uint8_t dest_node_id,
                    const Canard::Transfer &transfer) override;

        uint8_t get_node_id() const override
        {
            return canard_.node_id;
        }

        void process(uint32_t duration_ms);

        static void onTransferReceived(CanardInstance* ins,
                                    CanardRxTransfer* transfer);
        
        static bool shouldAcceptTransfer(const CanardInstance* ins,
                                        uint64_t* out_data_type_signature,
                                        uint16_t data_type_id,
                                        CanardTransferType transfer_type,
                                        uint8_t source_node_id);
    

    private:

        uint8_t memory_pool_[2048];
        CanardInstance canard_;
        CanardTxTransfer tx_transfer_;

        SocketCANInstance socketcan_;

};

#endif // CANARD_INTERFACE_HPP