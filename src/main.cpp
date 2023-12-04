
#include <driver/gpio.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <cstring>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <iomanip>

// ************************************** DEFINE **********************************************

#define LIDAR_TX GPIO_NUM_17
#define LIDAR_RX GPIO_NUM_16
#define LIDAR_UART UART_NUM_2
#define LIDAR_MOTOR GPIO_NUM_4

#define LIDAR_HIGH 1
#define LIDAR_LOW 0

#define RD_SIZE 5
#define TIMEOUT 10

#define RPLIDAR_RESP_MEASUREMENT_SYNCBIT (0x1 << 0)
#define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT 2

#define RPLIDAR_RESP_MEASUREMENT_CHECKBIT (0x1 << 0)
#define RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT 1

// #define RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT 2
// #define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_1 0xA
// #define RPLIDAR_RESP_MEASUREMENT_EXP_SYNC_2 0x5

#if !defined(_countof)
#if !defined(__cplusplus)
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#else
extern "C++"
{
    template <typename _CountofType, size_t _SizeOfArray>
    char (*__countof_helper(_CountofType (&_Array)[_SizeOfArray]))[_SizeOfArray];
#define _countof(_Array) sizeof(*__countof_helper(_Array))
}
#endif
#endif

#ifdef __cplusplus
extern "C"
{
    void app_main();
}
#endif

// ************************************** DEFINE **********************************************

// ******************************** CONSTANT VARIABLES ****************************************

const uint8_t GET_INFO[] = {0XA5, 0X50};
const uint8_t GET_INFO_RD[] = {0X14, 0X00, 0X00, 0X00, 0X04};

const uint8_t GET_HEALTH[] = {0XA5, 0X52};
const uint8_t GET_HEALTH_RD[] = {0x3, 0X00, 0X00, 0X00, 0X06};

const uint8_t STOP[] = {0XA5, 0x25};

const uint8_t FORCE_SCAN[] = {0xA5, 0x21};
const uint8_t SCAN[] = {0xA5, 0x20};
const uint8_t SCAN_RD[] = {0x05, 0x00, 0x00, 0x40, 0x81};

const uint8_t EXPRESS_SCAN_DATA[] = {0XA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22};
const uint8_t EXPRESS_SCAN_DATA_RD[] = {0X54, 0X00, 0X00, 0X40, 0X82};

// ******************************** CONSTANT VARIABLES ****************************************

// ********************************* ENUM AND STRUCT ******************************************

enum RPLidar_Request
{
    getInfo,
    getHealth,
    scan,
    expressScan,
};

typedef struct _response_get_info_t
{
    uint8_t model;
    uint16_t firmware_version;
    uint8_t hardware_version;
    uint8_t serialnum[16];
} __attribute__((packed)) response_get_info_t;

typedef struct _response_get_health_t
{
    uint8_t status;
    uint16_t error_code;
} __attribute__((packed)) response_get_health_t;

typedef struct _rplidar_response_measurement_node_hq_t
{
    uint16_t angle_z_q14;
    uint32_t dist_mm_q2;
    uint8_t quality;
    uint8_t flag;
} __attribute__((packed)) rplidar_response_measurement_node_hq_t;

typedef struct _rplidar_response_measurement_node_t
{
    uint8_t sync_quality;       // syncbit:1; syncbit_inverse:1; quality:6;
    uint16_t angle_q6_checkbit; // checkbit:1; angle_q6:15;
    uint16_t distance_q2;       // checkbit:1; angle_q6:15;
} __attribute__((packed)) rplidar_response_measurement_node_t;

// ********************************* ENUM AND STRUCT ******************************************

// ************************************** INSTANCE ********************************************

rplidar_response_measurement_node_hq_t _cached_scan_node_hq_buf[10];
size_t _cached_scan_node_hq_count;

// DTijCmU5aS98c6gihFDmkSUmKgTCXBGHrXrHXJv61aXf - 43 asterisks

// ************************************** INSTANCE ********************************************

// ***************************** RPLIDAR GENERAL FUNCTIONS ************************************

void connectLidar()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(LIDAR_UART, &uart_config);
    uart_set_pin(LIDAR_UART, LIDAR_TX, LIDAR_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(LIDAR_UART, 1024, 0, 0, NULL, 0);
}

void sendCommand(const uint8_t *command)
{
    uart_write_bytes(LIDAR_UART, (const char *)command, sizeof(command));
}

bool getResponseDescriptor(RPLidar_Request request)
{
    uint8_t byte1 = 0, byte2 = 0;
    while (byte1 != 0xA5 && byte2 != 0x5A)
    {
        printf("check response descriptor ");
        uart_read_bytes(LIDAR_UART, &byte1, 1, 10);
        if (byte1 == 0xA5)
            uart_read_bytes(LIDAR_UART, &byte2, 1, portMAX_DELAY);
        printf("0x%02x 0x%02x\n", byte1, byte2);
        vTaskDelay(100);
    }
    uint8_t rd[5];
    uart_read_bytes(LIDAR_UART, rd, sizeof(rd), portMAX_DELAY);

    switch (request)
    {
    case getInfo:
        return std::equal(GET_INFO_RD, GET_INFO_RD + RD_SIZE, rd);

    case getHealth:
        return std::equal(GET_HEALTH_RD, GET_HEALTH_RD + RD_SIZE, rd);

    case scan:
        return std::equal(SCAN_RD, SCAN_RD + RD_SIZE, rd);

        // case expressScan:
        //     return std::equal(EXPRESS_SCAN_DATA_RD, EXPRESS_SCAN_DATA_RD + RD_SIZE, rd);

    default:
        printf("eh! : incorrect response descriptor\n");
        return false;
    }
}

// ******************************* RPLIDAR GENERAL FUNCTIONS **********************************

// ******************************* RPLIDAR PRIVATE FUNCTIONS **********************************

void _waitNode(rplidar_response_measurement_node_t *node)
{
    uint8_t recvPos = 0;
    uint8_t *nodebuf = (uint8_t *)node;

    while (recvPos < 2)
    {
        uint8_t currentByte;
        uart_read_bytes(LIDAR_UART, &currentByte, 1, portMAX_DELAY);
        uint8_t tmp;
        switch (recvPos)
        {
        case 0: // expect sync bit and its reverse in this byte
            tmp = (currentByte >> 1);
            if ((tmp ^ currentByte) & 0x1)
            {
                // pass
            }
            else
            {
                // printf("current byte: 0x%02x\n", currentByte);
                continue;
            }
            break;
        case 1: // expect MSB to be 1
            if (currentByte & RPLIDAR_RESP_MEASUREMENT_CHECKBIT)
            {
                // pass
            }
            else
            {
                // printf("current byte: 0x%02x\n", currentByte);
                continue;
            }
            break;
        }
        nodebuf[recvPos++] = currentByte;
    }
    int nBytes = uart_read_bytes(LIDAR_UART, &nodebuf[recvPos], sizeof(rplidar_response_measurement_node_t) - recvPos, portMAX_DELAY);
    // ESP_LOGI("WAIT NODE", "%d bytes received", nBytes + 2);

    // printf("quality: 0x%02x\n",node->sync_quality);
    // printf("angle and checkbit: 0x%04x\n",node->angle_q6_checkbit);
    // printf("distanceq2: 0x%04x\n",node->distance_q2);
}

static void convert(const rplidar_response_measurement_node_t &from, rplidar_response_measurement_node_hq_t &to)
{
    to.angle_z_q14 = (((from.angle_q6_checkbit) >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) << 8) / 90; // transfer to q14 Z-angle
    to.dist_mm_q2 = from.distance_q2;
    to.flag = (from.sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT);                                                     // trasfer syncbit to HQ flag field
    to.quality = (from.sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT) << RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT; // remove the last two bits and then make quality from 0-63 to 0-255
}

// ******************************* RPLIDAR PRIVATE FUNCTIONS **********************************

// ******************************* RPLIDAR SPECIFIC FUNCTIONS *********************************

void getInfoResponse(response_get_info_t &info)
{
    uart_read_bytes(LIDAR_UART, &info, sizeof(info), portMAX_DELAY);
    printf("Model ID: %d\n", info.model);
    printf("Firmware version: %.2f\n", info.firmware_version / 256.0);
    printf("Hardware version: %d\n", info.hardware_version);

    std::stringstream ss;
    for (int i = 0; i < 16; ++i)
    {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(info.serialnum[i]);
    }
    std::cout << "Serial Number: " << ss.str() << std::endl;
}

void getHealthResponse(response_get_health_t &health)
{
    uart_read_bytes(LIDAR_UART, &health, sizeof(health), portMAX_DELAY);
    printf("Status:%d\n", health.status);
    printf("error code: %d\n", health.error_code);
}

void loopScanData()
{
    // static uint16_t recvNodeCount = 0;
    rplidar_response_measurement_node_t node;
    rplidar_response_measurement_node_hq_t nodeHq;
    _waitNode(&node);
    convert(node, nodeHq);

    _cached_scan_node_hq_buf[_cached_scan_node_hq_count++] = nodeHq;

    if (_cached_scan_node_hq_count >= _countof(_cached_scan_node_hq_buf))
    {
        _cached_scan_node_hq_count = 0;
    }
}
void grabScanData(rplidar_response_measurement_node_hq_t *nodebuffer, size_t &count)
{
    if (_cached_scan_node_hq_count == 0)
        return; // considered timeout
    size_t size_to_copy = std::min(count, _cached_scan_node_hq_count);
    std::memcpy(nodebuffer, _cached_scan_node_hq_buf, _cached_scan_node_hq_count * sizeof(rplidar_response_measurement_node_hq_t));
    count = size_to_copy;
    _cached_scan_node_hq_count = 0;
}

// ******************************* RPLIDAR SPECIFIC FUNCTIONS *********************************

// *********************************** MAIN FUNCTION ******************************************

void app_main()
{
    connectLidar();
    printf("%d\n", _countof(_cached_scan_node_hq_buf));
    gpio_set_direction(LIDAR_MOTOR, GPIO_MODE_OUTPUT);
    uart_write_bytes(LIDAR_UART, STOP, sizeof(STOP));
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    uart_flush(LIDAR_UART);

    sendCommand(GET_INFO);
    if (getResponseDescriptor(getInfo))
        printf("getInfo request should be ok, so proceed\n");

    response_get_info_t info;
    getInfoResponse(info);
    printf("crack brain more!!!!!!!!!!!!!!!!!!\n");
    printf("\n\n");

    sendCommand(GET_HEALTH);
    if (getResponseDescriptor(getHealth))
        printf("get health should be ok, so proceed\n");
    else
        printf("not good for the health!!!!!!!!!!!!!!!!!!\n");

    response_get_health_t health;
    getHealthResponse(health);
    printf("ti's not good for one's health\n");
    printf("\n\n");

    gpio_set_level(LIDAR_MOTOR, LIDAR_HIGH);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    uart_write_bytes(LIDAR_UART, FORCE_SCAN, sizeof(FORCE_SCAN));
    // uart_write_bytes(LIDAR_UART, SCAN, sizeof(SCAN));
    getResponseDescriptor(scan);

    rplidar_response_measurement_node_hq_t *nodes = new rplidar_response_measurement_node_hq_t[10];
    size_t nodeCount = 10;
    while (1)
    {
        
        loopScanData();
        grabScanData(nodes, nodeCount);

        for (size_t i = 0; i < nodeCount; ++i)
        {
            // convert to standard units
            float angle_in_degrees = nodes[i].angle_z_q14 * 90.f / (1 << 14);
            float distance_in_meters = nodes[i].dist_mm_q2 / 1000.f / (1 << 2);
            char report[50];
            snprintf(report, sizeof(report), "D:%.2f A:%.2f Q:%d", distance_in_meters, angle_in_degrees, nodes[i].quality);
            printf(report);
            printf("\n\n");
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    delete[] nodes;

    // _waitNode(&node);
    // printf("quality: 0x%02x\n", node.sync_quality);
    // printf("angle and checkbit: 0x%04x\n", node.angle_q6_checkbit);
    // printf("distanceq2: 0x%04x\n", node.distance_q2);
    // printf("\n");
}

// *********************************** MAIN FUNCTION ******************************************