#pragma once

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/kernel.h>

class WifiManager {
public:
    static WifiManager& instance();

    bool connect(const char* ssid, const char* password);
    bool is_connected();
    
    // Wait until connected with timeout
    bool wait_for_connection(k_timeout_t timeout);

private:
    WifiManager();
    static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                       uint64_t mgmt_event, struct net_if *iface);

    struct net_if *iface_;
    struct net_mgmt_event_callback wifi_mgmt_cb_;
    bool connected_ = false;
    struct k_sem wifi_connected_sem_;
};
