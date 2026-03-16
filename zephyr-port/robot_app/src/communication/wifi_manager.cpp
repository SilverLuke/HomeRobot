#include "wifi_manager.h"

#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(WifiManager, LOG_LEVEL_DBG);

WifiManager& WifiManager::instance() {
    static WifiManager instance;
    return instance;
}

WifiManager::WifiManager() {
    iface_ = net_if_get_default();
    k_sem_init(&wifi_connected_sem_, 0, 1);

    net_mgmt_init_event_callback(&wifi_mgmt_cb_, wifi_mgmt_event_handler,
                                 NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_mgmt_cb_);
}

void WifiManager::wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                          uint64_t mgmt_event, struct net_if *iface) {
    WifiManager& mgr = instance();
    if (mgmt_event == NET_EVENT_WIFI_CONNECT_RESULT) {
        const struct wifi_status *status = (const struct wifi_status *)cb->info;
        if (status->status == 0) {
            LOG_INF("Wi-Fi connected successfully");
            mgr.connected_ = true;
            k_sem_give(&mgr.wifi_connected_sem_);
        } else {
            LOG_ERR("Wi-Fi connection failed: %d", status->status);
        }
    } else if (mgmt_event == NET_EVENT_WIFI_DISCONNECT_RESULT) {
        LOG_INF("Wi-Fi disconnected");
        mgr.connected_ = false;
        k_sem_reset(&mgr.wifi_connected_sem_);
    }
}

bool WifiManager::connect(const char* ssid, const char* password) {
    struct wifi_connect_req_params params = {0};

    params.ssid = reinterpret_cast<const uint8_t*>(ssid);
    params.ssid_length = strlen(ssid);
    params.psk = reinterpret_cast<const uint8_t*>(password);
    params.psk_length = strlen(password);
    params.security = WIFI_SECURITY_TYPE_PSK;
    params.band = WIFI_FREQ_BAND_UNKNOWN;
    params.channel = WIFI_CHANNEL_ANY;

    LOG_INF("Connecting to Wi-Fi SSID: %s...", ssid);

    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface_, &params,
                       sizeof(struct wifi_connect_req_params));
    if (ret) {
        LOG_ERR("Wi-Fi connection request failed: %d", ret);
        return false;
    }

    return true;
}

bool WifiManager::is_connected() {
    return connected_;
}

bool WifiManager::wait_for_connection(k_timeout_t timeout) {
    if (connected_) {
        return true;
    }
    return k_sem_take(&wifi_connected_sem_, timeout) == 0;
}
