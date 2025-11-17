// 灵巧手和机械臂CAN接口自动检测程序
// 参考 linker_master_arm 的 can_ping_result.cpp
// 编译: g++ detect_hand_config.cpp -std=c++17 -o detect_hand_config

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <ifaddrs.h>
#include <filesystem>

namespace fs = std::filesystem;

static constexpr timeval RX_TIMEOUT{0, 100000}; // 100 ms

struct Job {
    const char* name;
    uint32_t id;
    bool is_ext;
    uint8_t data0;
};

bool ping_once(const std::string& ifname, const Job& job) {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) return false;

    ifreq ifr{};
    strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        close(sock);
        return false;
    }

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(sock);
        return false;
    }

    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &RX_TIMEOUT, sizeof(RX_TIMEOUT));

    can_frame tx{};
    tx.can_id = job.id | (job.is_ext ? CAN_EFF_FLAG : 0);
    tx.can_dlc = (tx.can_id == 0x27 || tx.can_id == 0x28) ? 1 : 8;
    memset(tx.data, 0, 8);
    tx.data[0] = job.data0;

    if (write(sock, &tx, sizeof(tx)) != sizeof(tx)) {
        close(sock);
        return false;
    }

    // 等待响应，可能会收到多个包
    can_frame rx{};
    for (int i = 0; i < 3; i++) {  // 尝试读取多次
        ssize_t n = read(sock, &rx, sizeof(rx));
        if (n > 0) {
            // 检查灵巧手响应
            if (tx.can_id == 0x28 && rx.can_id == 0x28) {
                close(sock);
                return true;
            }
            if (tx.can_id == 0x27 && rx.can_id == 0x27) {
                close(sock);
                return true;
            }
            // 检查Linker机械臂响应（参考can_ping_result.cpp）
            if ((tx.can_id | CAN_EFF_FLAG) == 0x8000FD3D && (rx.can_id | CAN_EFF_FLAG) == 0x80003DFE) {
                close(sock);
                return true;
            }
            if ((tx.can_id | CAN_EFF_FLAG) == 0x8000FD33 && (rx.can_id | CAN_EFF_FLAG) == 0x800033FE) {
                close(sock);
                return true;
            }
        }
    }
    
    close(sock);
    return false;
}

// 检查CAN接口是否UP
static bool is_interface_up(const std::string& ifname) {
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return false;

    ifreq ifr{};
    strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);

    bool up = (ioctl(fd, SIOCGIFFLAGS, &ifr) == 0) && (ifr.ifr_flags & IFF_UP);
    close(fd);
    return up;
}

std::vector<std::string> detect_can_devices() {
    std::vector<std::string> cans;
    ifaddrs *ifa = nullptr;
    if (getifaddrs(&ifa) == -1) {
        return cans;
    }
    for (ifaddrs *p = ifa; p; p = p->ifa_next) {
        if (!p->ifa_name) continue;
        std::string name(p->ifa_name);
        
        // 只检测物理CAN接口（can0, can1等）
        if (name.rfind("can", 0) == 0 && name.length() <= 5) {
            cans.emplace_back(name);
        }
    }
    freeifaddrs(ifa);
    return cans;
}

void yaml_writes(std::ofstream& yaml, const std::string& key, const std::string& value) {
    if (value.empty()) return;
    yaml << "    " << key << ": \"" << value << "\"\n";
}

int main(int argc, char* argv[]) {
    std::vector<std::string> buses = detect_can_devices();
    
    if (buses.empty()) {
        fprintf(stderr, "未检测到CAN接口\n");
        return 1;
    }

    printf("检测到 %zu 个CAN接口: ", buses.size());
    for (auto& b : buses) printf("%s ", b.c_str());
    printf("\n");

    // 自动启动CAN接口
    printf("检查并启动CAN接口...\n");
    for (auto& b : buses) {
        bool is_up = is_interface_up(b);
        printf("  %s: %s\n", b.c_str(), is_up ? "UP" : "DOWN");
        
        if (!is_up) {
            printf("  正在启动 %s...\n", b.c_str());
            // 已经用sudo运行了，不需要再加sudo
            system(std::string("ip link set " + b + " down 2>/dev/null").c_str());
            system(std::string("ip link set " + b + " up type can bitrate 1000000").c_str());
            system(std::string("chmod 0666 /sys/class/net/" + b + "/tx_queue_len 2>/dev/null").c_str());
            system(std::string("echo 1024 > /sys/class/net/" + b + "/tx_queue_len 2>/dev/null").c_str());
            
            // 等待接口启动
            usleep(100000);  // 100ms
            
            // 验证是否启动成功
            if (is_interface_up(b)) {
                printf("  %s 启动成功 ✓\n", b.c_str());
            } else {
                printf("  %s 启动失败 ✗\n", b.c_str());
            }
        }
    }
    printf("\n");

    // 定义检测任务（参考 can_ping_result.cpp，检测手和臂）
    const std::vector<Job> jobs = {
        {"left_hand",  0x28, false, 0x64},
        {"right_hand", 0x27, false, 0x64},
        {"left_hand",  0x28, false, 0xC1},
        {"right_hand", 0x27, false, 0xC1},
        {"left_arm",   0x8000FD3D, true, 0x00},   // Linker左臂
        {"right_arm",  0x8000FD33, true, 0x00},   // Linker右臂
    };

    // 检测结果
    std::map<std::string, std::string> hit;  // device_name -> can_interface

    printf("开始检测设备类型...\n");
    for (auto& bus : buses) {
        printf("  检测 %s:\n", bus.c_str());
        bool found = false;
        for (auto& j : jobs) {
            // 如果这个设备已经找到了，跳过
            if (hit.find(j.name) != hit.end() && hit[j.name] == bus) {
                continue;
            }
            
            printf("    尝试 %s (ID: 0x%02x, data: 0x%02x)... ", j.name, j.id, j.data0);
            fflush(stdout);
            
            if (ping_once(bus, j)) {
                printf("✓ 成功\n");
                printf("\n==> 发现设备: %s -> %s\n\n", j.name, bus.c_str());
                hit[j.name] = bus;
                found = true;
                break;  // 一个CAN口只能是一个设备
            } else {
                printf("✗ 无响应\n");
            }
        }
        if (!found) {
            printf("  ⚠ %s: 未识别到灵巧手\n", bus.c_str());
        }
    }
    printf("\n");

    // 判断模式
    std::string mode;
    std::string left_hand, right_hand, left_arm, right_arm;

    if (buses.size() == 4) {
        // 双手双臂模式
        mode = "double_linkerhand_grasp";
        left_hand = hit["left_hand"];
        right_hand = hit["right_hand"];
        left_arm = hit["left_arm"];
        right_arm = hit["right_arm"];
        
        printf("\n模式: 双手双臂 (4个CAN)\n");
        
    } else if (buses.size() == 2) {
        // Piper单手模式
        mode = "linkerhand_piper_grasp";
        
        if (!hit["left_hand"].empty()) {
            left_hand = hit["left_hand"];
            // 另一个是Piper左臂
            for (auto& bus : buses) {
                if (bus != left_hand) {
                    left_arm = bus;
                    break;
                }
            }
            printf("\n模式: Piper + 左手 (2个CAN)\n");
            
        } else if (!hit["right_hand"].empty()) {
            right_hand = hit["right_hand"];
            // 另一个是Piper右臂
            for (auto& bus : buses) {
                if (bus != right_hand) {
                    right_arm = bus;
                    break;
                }
            }
            printf("\n模式: Piper + 右手 (2个CAN)\n");
            
        } else {
            fprintf(stderr, "错误: 未检测到灵巧手\n");
            return 1;
        }
        
    } else {
        fprintf(stderr, "错误: CAN接口数量不符合要求 (需要2或4个，当前%zu个)\n", buses.size());
        return 1;
    }

    // 生成YAML配置文件
    fs::path scriptDir = fs::path(argv[0]).parent_path();
    if (scriptDir.empty()) scriptDir = ".";
    fs::path yamlPath = scriptDir / "hand_can_config.yaml";

    std::ofstream yaml(yamlPath);
    yaml << "# LinkerHand CAN配置文件\n";
    yaml << "# 自动生成，请勿手动编辑\n";
    yaml << "/**:\n";
    yaml << "  ros__parameters:\n";
    yaml << "    mode: \"" << mode << "\"\n";
    yaml_writes(yaml, "left_hand", left_hand);
    yaml_writes(yaml, "right_hand", right_hand);
    yaml_writes(yaml, "left_arm", left_arm);
    yaml_writes(yaml, "right_arm", right_arm);
    yaml.close();

    printf("\n配置已保存到: %s\n", yamlPath.c_str());
    printf("  模式: %s\n", mode.c_str());
    if (!left_hand.empty())  printf("  左手: %s\n", left_hand.c_str());
    if (!right_hand.empty()) printf("  右手: %s\n", right_hand.c_str());
    if (!left_arm.empty())   printf("  左臂: %s\n", left_arm.c_str());
    if (!right_arm.empty())  printf("  右臂: %s\n", right_arm.c_str());

    return 0;
}

