#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

#define SL_RESULT_OK                (0)
#define SL_RESULT_FAIL_BIT          (0x80000000)
#define SL_RESULT_ALREADY_DONE      (0x20)
#define SL_RESULT_INVALID_DATA      (0x8000 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_FAIL    (0x8001 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_TIMEOUT (0x8002 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_STOP    (0x8003 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_OPERATION_NOT_SUPPORT (0x8004 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_FORMAT_NOT_SUPPORT    (0x8005 | SL_RESULT_FAIL_BIT)
#define SL_RESULT_INSUFFICIENT_MEMORY   (0x8006 | SL_RESULT_FAIL_BIT)

typedef uint32_t sl_result;
#define SL_IS_OK(x)    (((x) & SL_RESULT_FAIL_BIT) == 0)
#define SL_IS_FAIL(x)  (((x) & SL_RESULT_FAIL_BIT) != 0)

#pragma pack(push, 1)

// Slamtec standard High Quality scan measurement node
typedef struct sl_lidar_response_measurement_node_hq_t {
    uint16_t   angle_z_q14; // angle in degrees * 16384 / 90 (q14 format)
    uint32_t   dist_mm_q2;  // distance in mm * 4 (q2 format)
    uint8_t    quality;     // signal quality
    uint8_t    flag;        // sync flag: bit 0 indicates start of scan (1 = new 360-deg scan)
} sl_lidar_response_measurement_node_hq_t;

typedef struct sl_lidar_response_device_info_t {
    uint8_t   model;
    uint16_t  firmware_version;
    uint8_t   hardware_version;
    uint8_t   serialnum[16];
} sl_lidar_response_device_info_t;

typedef struct sl_lidar_response_device_health_t {
    uint8_t   status;
    uint16_t  error_code;
} sl_lidar_response_device_health_t;

#define SL_LIDAR_STATUS_OK      0x0
#define SL_LIDAR_STATUS_WARNING 0x1
#define SL_LIDAR_STATUS_ERROR   0x2

#pragma pack(pop)

namespace sl {

class IChannel {
public:
    virtual ~IChannel() {}
    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() = 0;
};

class ILidarDriver {
public:
    static ILidarDriver* create();
    virtual ~ILidarDriver() {}

    virtual sl_result connect(IChannel* channel) = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() = 0;

    virtual sl_result getDeviceInfo(sl_lidar_response_device_info_t& info, uint32_t timeoutMs = 1000) = 0;
    virtual sl_result getHealth(sl_lidar_response_device_health_t& health, uint32_t timeoutMs = 1000) = 0;

    virtual sl_result setMotorSpeed(uint16_t speed = 600) = 0;
    virtual sl_result startScan(bool force, bool useTypicalScan, uint32_t options = 0, void* outScanMode = nullptr) = 0;
    virtual sl_result stop() = 0;

    // Primary scan grabbing function required by NIDAR challenge
    virtual sl_result grabScanDataHq(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t& count, uint32_t timeout = 1000) = 0;
    virtual sl_result ascendScanData(sl_lidar_response_measurement_node_hq_t* nodebuffer, size_t count) = 0;
};

IChannel* createSerialPortChannel(const char* port, uint32_t baudrate);

} // namespace sl
