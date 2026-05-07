#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

void SetIpCallback(livox_status status, uint8_t handle, void* client_data) {
    if (status == kStatusSuccess) {
        printf("IP set successfully! LiDAR rebooting...\n");
    } else {
        printf("Failed to set IP.\n");
    }
}

void OnDeviceInfoChange(const LivoxLidarInfo* info, void* client_data) {
    printf("LiDAR found! Handle: %d, IP: %s\n", info->handle, info->ip);

    LivoxLidarIpInfo ip_info;
    strncpy(ip_info.ip_addr,  "192.168.1.125", sizeof(ip_info.ip_addr));
    strncpy(ip_info.net_mask, "255.255.255.0", sizeof(ip_info.net_mask));
    strncpy(ip_info.gw_addr,  "192.168.1.1",   sizeof(ip_info.gw_addr));

    LivoxLidarSetIp(info->handle, &ip_info, SetIpCallback, nullptr);
}

int main() {
    LivoxLidarSdkInit("config.json");
    SetLivoxLidarInfoChangeCallback(OnDeviceInfoChange, nullptr);

    printf("Waiting for LiDAR...\n");
    sleep(10);  // wait for discovery + IP set + reboot

    LivoxLidarSdkUninit();
    return 0;
}
