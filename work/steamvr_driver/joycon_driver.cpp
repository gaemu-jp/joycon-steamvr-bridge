#include <openvr_driver.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cctype>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

namespace {

constexpr uint16_t kSteamVrUdpPort = 39772;

static bool FindSection(const char* text, const char* key, const char** start, const char** end) {
    std::string needle = std::string("\"") + key + "\"";
    const char* pos = std::strstr(text, needle.c_str());
    if (!pos) return false;
    pos = std::strchr(pos, ':');
    if (!pos) return false;
    while (*pos && (std::isspace(static_cast<unsigned char>(*pos)) || *pos == ':')) ++pos;
    if (*pos == '[') {
        const char* close = std::strchr(pos, ']');
        if (!close) return false;
        *start = pos + 1;
        *end = close;
        return true;
    }
    return false;
}

static bool ParseQuaternion(const char* text, double& w, double& x, double& y, double& z) {
    const char* start = nullptr;
    const char* end = nullptr;
    if (!FindSection(text, "quat", &start, &end) && !FindSection(text, "quaternion", &start, &end)) {
        return false;
    }
    double values[4] = {};
    int count = 0;
    const char* p = start;
    while (p < end && count < 4) {
        while (p < end && (std::isspace(static_cast<unsigned char>(*p)) || *p == ',')) ++p;
        char* next = nullptr;
        values[count] = std::strtod(p, &next);
        if (next == p) break;
        p = next;
        ++count;
    }
    if (count != 4) return false;
    x = values[0];
    y = values[1];
    z = values[2];
    w = values[3];
    return true;
}

static bool ParseQuaternionFromPose(const char* text, double& w, double& x, double& y, double& z) {
    const char* pose = std::strstr(text, "\"pose\"");
    if (!pose) {
        return false;
    }
    const char* quat = std::strstr(pose, "\"quat\"");
    if (!quat) {
        quat = std::strstr(pose, "\"quaternion\"");
    }
    if (!quat) {
        return false;
    }
    const char* start = std::strchr(quat, '[');
    if (!start) {
        return false;
    }
    const char* end = std::strchr(start, ']');
    if (!end) {
        return false;
    }
    double values[4] = {};
    int count = 0;
    const char* p = start + 1;
    while (p < end && count < 4) {
        while (p < end && (std::isspace(static_cast<unsigned char>(*p)) || *p == ',')) ++p;
        char* next = nullptr;
        values[count] = std::strtod(p, &next);
        if (next == p) break;
        p = next;
        ++count;
    }
    if (count != 4) return false;
    x = values[0];
    y = values[1];
    z = values[2];
    w = values[3];
    return true;
}

static bool ParsePosition(const char* text, double& x, double& y, double& z) {
    const char* pose = std::strstr(text, "\"pose\"");
    if (!pose) return false;
    const char* position = std::strstr(pose, "\"position\"");
    if (!position) return false;
    const char* start = std::strchr(position, '[');
    if (!start) return false;
    const char* end = std::strchr(start, ']');
    if (!end) return false;
    double values[3] = {};
    int count = 0;
    const char* p = start + 1;
    while (p < end && count < 3) {
        while (p < end && (std::isspace(static_cast<unsigned char>(*p)) || *p == ',')) ++p;
        char* next = nullptr;
        values[count++] = std::strtod(p, &next);
        if (next == p) return false;
        p = next;
    }
    if (count != 3) return false;
    x = values[0];
    y = values[1];
    z = values[2];
    return true;
}

static std::string ParseDevice(const char* text) {
    const char* pos = std::strstr(text, "\"device\"");
    if (!pos) return "right";
    pos = std::strchr(pos, ':');
    if (!pos) return "right";
    while (*pos && (std::isspace(static_cast<unsigned char>(*pos)) || *pos == ':')) ++pos;
    if (*pos != '\"') return "right";
    ++pos;
    const char* end = std::strchr(pos, '\"');
    if (!end) return "right";
    return std::string(pos, end);
}

static bool ParseNumberAfter(const char* text, const char* key, double& value) {
    const char* pos = std::strstr(text, key);
    if (!pos) return false;
    pos = std::strchr(pos, ':');
    if (!pos) return false;
    char* end = nullptr;
    value = std::strtod(pos + 1, &end);
    return end != pos + 1;
}

static bool ParseInputValue(const char* text, const char* inputName, double& value) {
    std::string key = std::string("\"inputs\"");
    const char* inputs = std::strstr(text, key.c_str());
    if (!inputs) return false;
    std::string name = std::string("\"") + inputName + "\"";
    const char* field = std::strstr(inputs, name.c_str());
    if (!field) return false;
    return ParseNumberAfter(field, name.c_str(), value);
}

class JoyconDevice final : public vr::ITrackedDeviceServerDriver {
public:
    explicit JoyconDevice(std::string serial, vr::ETrackedControllerRole role)
        : serial_(std::move(serial)), role_(role) {}

    vr::EVRInitError Activate(uint32_t objectId) override {
        object_id_ = objectId;
        property_container_ = vr::VRProperties()->TrackedDeviceToPropertyContainer(object_id_);
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_ModelNumber_String, "Joy-Con VR");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_SerialNumber_String, serial_.c_str());
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_ManufacturerName_String, "Joy-Con Bridge");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_ControllerType_String, "joycon_controller");
        vr::VRProperties()->SetStringProperty(property_container_, vr::Prop_InputProfilePath_String, "{joycon}/input/joycon_controller_profile.json");
        vr::VRProperties()->SetBoolProperty(property_container_, vr::Prop_DeviceIsWireless_Bool, true);
        vr::VRProperties()->SetInt32Property(property_container_, vr::Prop_ControllerRoleHint_Int32, static_cast<int32_t>(role_));
        SetPose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/trigger/click", &trigger_click_);
        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/grip/click", &grip_click_);
        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/menu/click", &menu_click_);
        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/application_menu/click", &application_menu_click_);
        vr::VRDriverInput()->CreateScalarComponent(property_container_, "/input/thumbstick/x", &stick_x_, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateScalarComponent(property_container_, "/input/thumbstick/y", &stick_y_, vr::VRScalarType_Absolute, vr::VRScalarUnits_NormalizedTwoSided);
        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/thumbstick/click", &stick_click_);
        vr::VRDriverInput()->CreateBooleanComponent(property_container_, "/input/thumbstick/touch", &stick_touch_);
        PublishPose();
        return vr::VRInitError_None;
    }

    void Deactivate() override { object_id_ = vr::k_unTrackedDeviceIndexInvalid; }
    void EnterStandby() override {}
    void* GetComponent(const char* nameAndVersion) override { (void)nameAndVersion; return nullptr; }
    void DebugRequest(const char* request, char* responseBuffer, uint32_t responseBufferSize) override {
        (void)request;
        if (responseBufferSize > 0) responseBuffer[0] = '\0';
    }

    vr::DriverPose_t GetPose() override {
        std::lock_guard<std::mutex> lock(mu_);
        return pose_;
    }

    void SetPose(double w, double x, double y, double z, double px, double py, double pz) {
        std::lock_guard<std::mutex> lock(mu_);
        pose_ = {};
        pose_.qWorldFromDriverRotation = {1.0, 0.0, 0.0, 0.0};
        pose_.qDriverFromHeadRotation = {1.0, 0.0, 0.0, 0.0};
        pose_.qRotation = {w, x, y, z};
        pose_.poseIsValid = true;
        pose_.willDriftInYaw = false;
        pose_.shouldApplyHeadModel = false;
        pose_.deviceIsConnected = true;
        // VMC supplies the hand position; report a fully tracked pose so
        // SteamVR does not discard vecPosition as rotation-only fallback.
        pose_.result = vr::TrackingResult_Running_OK;
        pose_.vecPosition[0] = px;
        pose_.vecPosition[1] = py;
        pose_.vecPosition[2] = pz;
        pose_.vecVelocity[0] = 0.0;
        pose_.vecVelocity[1] = 0.0;
        pose_.vecVelocity[2] = 0.0;
        pose_.vecAcceleration[0] = 0.0;
        pose_.vecAcceleration[1] = 0.0;
        pose_.vecAcceleration[2] = 0.0;
        pose_.poseTimeOffset = 0.0;
        last_quat_[0] = w;
        last_quat_[1] = x;
        last_quat_[2] = y;
        last_quat_[3] = z;
        dirty_ = true;
    }

    void PublishPose() {
        if (object_id_ == vr::k_unTrackedDeviceIndexInvalid) {
            return;
        }
        vr::DriverPose_t pose;
        {
            std::lock_guard<std::mutex> lock(mu_);
            pose = pose_;
            if (dirty_) {
                dirty_ = false;
                if (auto* log = vr::VRDriverLog()) {
                    char buf[256];
                    std::snprintf(buf, sizeof(buf),
                                  "joycon %s pose q=(%.3f, %.3f, %.3f, %.3f) p=(%.3f, %.3f, %.3f) valid=%d connected=%d result=%d",
                                  serial_.c_str(),
                                  pose.qRotation.w, pose.qRotation.x, pose.qRotation.y, pose.qRotation.z,
                                  pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2],
                                  pose.poseIsValid ? 1 : 0,
                                  pose.deviceIsConnected ? 1 : 0,
                                  static_cast<int>(pose.result));
                    log->Log(buf);
                }
            }
        }
        vr::VRServerDriverHost()->TrackedDevicePoseUpdated(object_id_, pose, sizeof(vr::DriverPose_t));
    }

    void UpdateInputs(bool trigger, bool grip, bool menu, bool application_menu,
                      bool stick_click, bool stick_touch, double stick_x, double stick_y) {
        if (trigger_click_) vr::VRDriverInput()->UpdateBooleanComponent(trigger_click_, trigger, 0.0);
        if (grip_click_) vr::VRDriverInput()->UpdateBooleanComponent(grip_click_, grip, 0.0);
        if (menu_click_) vr::VRDriverInput()->UpdateBooleanComponent(menu_click_, menu, 0.0);
        if (application_menu_click_) vr::VRDriverInput()->UpdateBooleanComponent(application_menu_click_, application_menu, 0.0);
        if (stick_click_) vr::VRDriverInput()->UpdateBooleanComponent(stick_click_, stick_click, 0.0);
        if (stick_touch_) vr::VRDriverInput()->UpdateBooleanComponent(stick_touch_, stick_touch, 0.0);
        if (stick_x_) vr::VRDriverInput()->UpdateScalarComponent(stick_x_, static_cast<float>(stick_x), 0.0);
        if (stick_y_) vr::VRDriverInput()->UpdateScalarComponent(stick_y_, static_cast<float>(stick_y), 0.0);
    }

    uint32_t ObjectId() const {
        return object_id_;
    }

private:
    std::string serial_;
    vr::ETrackedControllerRole role_;
    uint32_t object_id_ = vr::k_unTrackedDeviceIndexInvalid;
    vr::PropertyContainerHandle_t property_container_ = vr::k_ulInvalidPropertyContainer;
    vr::DriverPose_t pose_{};
    bool dirty_ = false;
    vr::VRInputComponentHandle_t trigger_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t grip_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t menu_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t application_menu_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t stick_x_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t stick_y_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t stick_click_ = vr::k_ulInvalidInputComponentHandle;
    vr::VRInputComponentHandle_t stick_touch_ = vr::k_ulInvalidInputComponentHandle;
    double last_quat_[4] = {1.0, 0.0, 0.0, 0.0};
    std::mutex mu_;
};

class JoyconProvider final : public vr::IServerTrackedDeviceProvider {
public:
    vr::EVRInitError Init(vr::IVRDriverContext* pDriverContext) override {
        VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
        left_ = std::make_unique<JoyconDevice>("joycon-left", vr::TrackedControllerRole_LeftHand);
        right_ = std::make_unique<JoyconDevice>("joycon-right", vr::TrackedControllerRole_RightHand);

        vr::VRServerDriverHost()->TrackedDeviceAdded("joycon-left", vr::TrackedDeviceClass_Controller, left_.get());
        vr::VRServerDriverHost()->TrackedDeviceAdded("joycon-right", vr::TrackedDeviceClass_Controller, right_.get());

        running_.store(true);
        worker_ = std::thread(&JoyconProvider::WorkerLoop, this);
        return vr::VRInitError_None;
    }

    void Cleanup() override {
        running_.store(false);
        if (worker_.joinable()) {
            worker_.join();
        }
        right_.reset();
        left_.reset();
        VR_CLEANUP_SERVER_DRIVER_CONTEXT();
    }

    const char* const* GetInterfaceVersions() override { return vr::k_InterfaceVersions; }
    void RunFrame() override {
        PushPoses();
    }
    bool ShouldBlockStandbyMode() override { return false; }
    void EnterStandby() override {}
    void LeaveStandby() override {}

private:
    void WorkerLoop() {
        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            return;
        }
        SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock == INVALID_SOCKET) {
            WSACleanup();
            return;
        }
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(kSteamVrUdpPort);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        if (bind(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            closesocket(sock);
            WSACleanup();
            return;
        }

        while (running_.load()) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(sock, &readfds);
            timeval timeout{};
            timeout.tv_sec = 0;
            timeout.tv_usec = 50000;
            int ready = select(0, &readfds, nullptr, nullptr, &timeout);
            if (ready <= 0) {
                PushPoses();
                continue;
            }

            char buffer[2048];
            sockaddr_in from{};
            int from_len = sizeof(from);
            int received = recvfrom(sock, buffer, sizeof(buffer) - 1, 0, reinterpret_cast<sockaddr*>(&from), &from_len);
            if (received <= 0) {
                PushPoses();
                continue;
            }
            buffer[received] = '\0';
            ApplyPacket(buffer);
            PushPoses();
        }
        closesocket(sock);
        WSACleanup();
    }

    void ApplyPacket(const char* text) {
        double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
        double px = 0.0, py = 0.0, pz = 0.0;
        if (!ParseQuaternionFromPose(text, w, x, y, z) && !ParseQuaternion(text, w, x, y, z)) {
            return;
        }
        ParsePosition(text, px, py, pz);
        double trigger = 0.0, grip = 0.0, menu = 0.0, application_menu = 0.0;
        double stick_click = 0.0, stick_touch = 0.0, stick_x = 0.0, stick_y = 0.0;
        ParseInputValue(text, "trigger", trigger);
        ParseInputValue(text, "grip", grip);
        ParseInputValue(text, "menu", menu);
        ParseInputValue(text, "application_menu", application_menu);
        ParseInputValue(text, "stick_click", stick_click);
        ParseInputValue(text, "stick_touch", stick_touch);
        const char* stick = std::strstr(text, "\"stick\"");
        if (stick) {
            ParseNumberAfter(stick, "\"x\"", stick_x);
            ParseNumberAfter(stick, "\"y\"", stick_y);
        }

        vr::TrackedDevicePose_t tracked_poses[vr::k_unMaxTrackedDeviceCount]{};
        vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.0f, tracked_poses, vr::k_unMaxTrackedDeviceCount);
        const auto& hmd_pose = tracked_poses[vr::k_unTrackedDeviceIndex_Hmd];
        if (hmd_pose.bPoseIsValid && hmd_pose.bDeviceIsConnected) {
            const auto& matrix = hmd_pose.mDeviceToAbsoluteTracking;
            // Rotate VMC's head-relative offset by HMD yaw only. Full HMD
            // pitch/roll would pull the hands into the user's face.
            double right_x = matrix.m[0][0];
            double right_z = matrix.m[2][0];
            const double right_len = std::sqrt(right_x * right_x + right_z * right_z);
            if (right_len > 1e-6) {
                right_x /= right_len;
                right_z /= right_len;
            } else {
                right_x = 1.0;
                right_z = 0.0;
            }
            const double local_x = px;
            const double local_z = pz;
            px = matrix.m[0][3] + right_x * local_x - right_z * local_z;
            py += matrix.m[1][3];
            pz = matrix.m[2][3] + right_z * local_x + right_x * local_z;
        }
        std::string device = ParseDevice(text);
        if (device == "left") {
            if (left_) {
                left_->SetPose(w, x, y, z, px, py, pz);
                left_->UpdateInputs(trigger > 0.5, grip > 0.5, menu > 0.5, application_menu > 0.5,
                                    stick_click > 0.5, stick_touch > 0.5, stick_x, stick_y);
                left_->PublishPose();
            }
        } else {
            if (right_) {
                right_->SetPose(w, x, y, z, px, py, pz);
                right_->UpdateInputs(trigger > 0.5, grip > 0.5, menu > 0.5, application_menu > 0.5,
                                     stick_click > 0.5, stick_touch > 0.5, stick_x, stick_y);
                right_->PublishPose();
            }
        }
    }

    void PushPoses() {
        if (left_) {
            left_->PublishPose();
        }
        if (right_) {
            right_->PublishPose();
        }
    }

    std::atomic<bool> running_{false};
    std::thread worker_;
    std::unique_ptr<JoyconDevice> left_;
    std::unique_ptr<JoyconDevice> right_;
};

JoyconProvider g_provider;

}  // namespace

extern "C" __declspec(dllexport) void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode) {
    if (std::strcmp(pInterfaceName, vr::IServerTrackedDeviceProvider_Version) == 0) {
        if (pReturnCode) *pReturnCode = vr::VRInitError_None;
        return &g_provider;
    }
    if (pReturnCode) *pReturnCode = vr::VRInitError_Init_InterfaceNotFound;
    return nullptr;
}
