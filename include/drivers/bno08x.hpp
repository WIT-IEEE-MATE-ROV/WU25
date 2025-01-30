#pragma once

#include <string>
#include <chrono>

// extern "C" {
// #include "sh2.h"
// #include "sh2_SensorValue.h"
// #include "sh2_err.h"
// }

#include "spi.hpp"

using namespace std::chrono_literals;

typedef enum {
    BNO_CHANNEL_SHTP_COMMAND = 0,
    BNO_CHANNEL_EXE = 1, 
    BNO_CHANNEL_SH_CONTROL = 2,
    BNO_CHANNEL_INPUT_SENSOR_REPORTS = 3,
    BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = 4,
    BNO_CHANNEL_GYRO_ROTATION_VECTOR = 5
} bno_channel;

typedef struct {
    uint16_t data_length;
    bno_channel channel;
    uint8_t seq_num;
} packet_header;

class Packet {
public:
    static const uint32_t MAX_PACKET_LEN = 32766 + 4;

    packet_header header;
    uint8_t packet_data[MAX_PACKET_LEN];

    Packet();
    Packet(packet_header hdr);
    Packet(uint8_t *packet_data, uint16_t packet_len);
    Packet(packet_header hdr, uint8_t *packet_data, uint16_t packet_len);
};

class BNO08x {
public: 
    BNO08x(std::string dev_file="/dev/spidev0.0");
    ~BNO08x();
    
    void initialize();

    void service();

private:
    sh2_Hal_t *psh2;
    static std::string spi_path;
    static spi_t *spi_dev;
    static const uint32_t SPI_BUFLEN = 32766 + 4;
    static uint8_t spi_txbuf[SPI_BUFLEN];
    static uint8_t spi_rxbuf[SPI_BUFLEN];
    static uint32_t spi_recv_len;
    static uint8_t sequence_numbers[6];

    static const uint32_t spi_mode = SPI_MODE_3;
    static const uint32_t spi_speed = 100000;
    static const uint8_t spi_bits = 8;
    static char *input_file;
    static char *output_file;
    static uint16_t spi_delay;
    static int spi_verbose;

    static uint32_t bno_interrupts;
    static uint32_t bno_int_timestamp_us;

    static const int32_t BNO_RST_PIN = 25; 
    static const int32_t BNO_INT_PIN = 19; // $ gpio edge 113 falling $ gpio mode 113 up
    static const int32_t BNO_WAK_PIN = 27;
    static const uint8_t BNO_PRODUCT_ID_REQ = 0xF9;

    static int32_t reportProdIds(sh2_ProductIds_t *ids);

    static int32_t wait_for_h_intn() 
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        uint32_t start_inc = bno_interrupts;

        while (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count() < 3.) { // TODO: Lower than 3 
            if (bno_interrupts != start_inc) {
                return 0;
            }
        }

        return 1;
    }

    static void update_sequence_number(packet_header *hdr) {
        sequence_numbers[hdr->channel] = hdr->seq_num;
    }

    static Packet read_packet_type(bno_channel channel, float timeout_ms = 5.0)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
        auto current_time = start_time;
        Packet pkt; 

        while(std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() >= timeout_ms) {
            pkt = read_packet();    

            if (pkt.header.channel == channel) {
                return pkt;
            }
            if (pkt.header.channel != BNO_CHANNEL_EXE || pkt.header.channel != BNO_CHANNEL_SHTP_COMMAND) {

            }

            // TODO: Handle packet()


            current_time = std::chrono::high_resolution_clock::now();
        }

        printf("Timed out waiting for packet type\n");
        return Packet();
    }
    
    static uint32_t read_header(packet_header *hdr_out, bool wait_for_int=true)
    {
        if (wait_for_int && wait_for_h_intn()) {
            printf("Timed out reading header");
            return 1;
        }

        int32_t read_ret;
        if ((read_ret = spi_read(spi_dev, spi_rxbuf, 4)) < 0) {
            printf("SPI read failed: %s (%d)\n", strerror(errno), read_ret);
            return 1;
        }

        hdr_out->channel = (bno_channel) spi_rxbuf[2];
        hdr_out->seq_num = spi_rxbuf[3];
        hdr_out->data_length = (spi_rxbuf[1] << 8) | spi_rxbuf[0];

        printf("Read header: len: %u, channel: %u, seqnum: %u\n", hdr_out->data_length, hdr_out->channel, hdr_out->seq_num);
        return 0;
    }

    static Packet read_packet(bool wait_for_int=true)
    {
        packet_header hdr;
        if (read_header(&hdr, wait_for_int)) {
            printf("Read header failed\n");
            return Packet();
        }

        bool half_packet = false;

        if (hdr.channel & 0x80) {
            half_packet = true;
        }

        sequence_numbers[hdr.channel] = hdr.seq_num;

        if (hdr.data_length == 0) {
            printf("Read packet length == 0\n");
            return Packet();
        }

        if (hdr.data_length > SPI_BUFLEN) {
            printf("Can't read everything %u > SPI_BUFLEN, channel = %u, seqnum = %u\n", hdr.data_length, hdr.channel, hdr.seq_num);
            return Packet();
        }

        spi_read(spi_dev, spi_rxbuf, hdr.data_length);

        packet_header new_hdr;
        new_hdr.channel = (bno_channel) spi_rxbuf[2];
        new_hdr.seq_num = spi_rxbuf[3];

        new_hdr.data_length = (spi_rxbuf[1] << 8) & spi_rxbuf[0];

        if (half_packet) {
            printf("Error: Read partial packet\n");
            return Packet();
        }

        sequence_numbers[new_hdr.channel] = new_hdr.seq_num;

        // Set recv len to use in sh2 read callback
        spi_recv_len = new_hdr.data_length;

        return Packet(new_hdr, spi_rxbuf, new_hdr.data_length);
    }

    static uint8_t send_packet(uint8_t channel, uint8_t *data, uint16_t data_len)
    {
        uint32_t write_len = data_len + 4;

        // Fill in 
        spi_txbuf[0] = write_len & 0xFF;
        spi_txbuf[1] = (write_len >> 8) & 0xFF;

        spi_txbuf[2] = channel;

        spi_txbuf[3] = sequence_numbers[channel];

        memcpy(spi_txbuf + 4, data, data_len);

        if (wait_for_h_intn()) {
            printf("BNO08x send timed out waiting for interrupt\n");
            return 1;
        }

        if (spi_write(spi_dev, spi_txbuf, write_len) < 0) {
            printf("SPI write failed: %s\n", strerror(errno));
            return 1;
        }

        ++sequence_numbers[channel];

        return 0;
    }

    static void event_sh2_cb(void *cookie, sh2_AsyncEvent_t *pEvent)
    {
        BNO08x *bno = (BNO08x *) cookie;

        // If we see a reset, set a flag so that sensors will be reconfigured.
        if (pEvent->eventId == SH2_RESET) {
            bno->reset_sh2_cb(pEvent);
        }
        else if (pEvent->eventId == SH2_SHTP_EVENT) {
            printf("EventHandler  id:SHTP, %d\n", pEvent->shtpEvent);
        }
        else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
            // printf("EventHandler Sensor Config, %d\n", pEvent->sh2SensorConfigResp.sensorId);
        }
        else {
            printf("EventHandler, unknown event Id: %d\n", pEvent->eventId);
        }
    }

    static void sensor_event_sh2_cb(void * cookie, sh2_SensorEvent_t *pEvent)
    {
        int rc;
        sh2_SensorValue_t value;
        float scaleRadToDeg = 180.0 / 3.14159265358;
        float r, i, j, k, acc_deg, x, y, z;
        float t;
        static int skip = 0;

        rc = sh2_decodeSensorEvent(&value, pEvent);
        if (rc != SH2_OK) {
            printf("Error decoding sensor event: %d\n", rc);
            return;
        }

        t = value.timestamp / 1000000.0;  // time in seconds.
        switch (value.sensorId) {
            case SH2_RAW_ACCELEROMETER:
                printf("%8.4f Raw acc: %d %d %d time_us:%d\n",
                    (double)t,
                    value.un.rawAccelerometer.x,
                    value.un.rawAccelerometer.y,
                    value.un.rawAccelerometer.z,
                    value.un.rawAccelerometer.timestamp);
                break;

            case SH2_ACCELEROMETER:
                printf("%8.4f Acc: %f %f %f\n",
                    (double)t,
                    (double)value.un.accelerometer.x,
                    (double)value.un.accelerometer.y,
                    (double)value.un.accelerometer.z);
                break;
                
            case SH2_RAW_GYROSCOPE:
                printf("%8.4f Raw gyro: x:%d y:%d z:%d temp:%d time_us:%d\n",
                    (double)t,
                    value.un.rawGyroscope.x,
                    value.un.rawGyroscope.y,
                    value.un.rawGyroscope.z,
                    value.un.rawGyroscope.temperature,
                    value.un.rawGyroscope.timestamp);
                break;
                
            case SH2_ROTATION_VECTOR:
                r = value.un.rotationVector.real;
                i = value.un.rotationVector.i;
                j = value.un.rotationVector.j;
                k = value.un.rotationVector.k;
                acc_deg = scaleRadToDeg * 
                    value.un.rotationVector.accuracy;
                printf("%8.4f Rotation Vector: "
                    "r:%0.6f i:%0.6f j:%0.6f k:%0.6f (acc: %0.6f deg)\n",
                    (double)t,
                    (double)r, (double)i, (double)j, (double)k, (double)acc_deg);
                break;
            case SH2_GAME_ROTATION_VECTOR:
                r = value.un.gameRotationVector.real;
                i = value.un.gameRotationVector.i;
                j = value.un.gameRotationVector.j;
                k = value.un.gameRotationVector.k;
                printf("%8.4f GRV: "
                    "r:%0.6f i:%0.6f j:%0.6f k:%0.6f\n",
                    (double)t,
                    (double)r, (double)i, (double)j, (double)k);
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                x = value.un.gyroscope.x;
                y = value.un.gyroscope.y;
                z = value.un.gyroscope.z;
                printf("%8.4f GYRO: "
                    "x:%0.6f y:%0.6f z:%0.6f\n",
                    (double)t,
                    (double)x, (double)y, (double)z);
                break;
            case SH2_GYROSCOPE_UNCALIBRATED:
                x = value.un.gyroscopeUncal.x;
                y = value.un.gyroscopeUncal.y;
                z = value.un.gyroscopeUncal.z;
                printf("%8.4f GYRO_UNCAL: "
                    "x:%0.6f y:%0.6f z:%0.6f\n",
                    (double)t,
                    (double)x, (double)y, (double)z);
                break;
            case SH2_GYRO_INTEGRATED_RV:
                // These come at 1kHz, too fast to print all of them.
                // So only print every 10th one
                skip++;
                if (skip == 10) {
                    skip = 0;
                    r = value.un.gyroIntegratedRV.real;
                    i = value.un.gyroIntegratedRV.i;
                    j = value.un.gyroIntegratedRV.j;
                    k = value.un.gyroIntegratedRV.k;
                    x = value.un.gyroIntegratedRV.angVelX;
                    y = value.un.gyroIntegratedRV.angVelY;
                    z = value.un.gyroIntegratedRV.angVelZ;
                    printf("%8.4f Gyro Integrated RV: "
                        "r:%0.6f i:%0.6f j:%0.6f k:%0.6f x:%0.6f y:%0.6f z:%0.6f\n",
                        (double)t,
                        (double)r, (double)i, (double)j, (double)k,
                        (double)x, (double)y, (double)z);
                }
                break;
            case SH2_IZRO_MOTION_REQUEST:
                printf("IZRO Request: intent:%d, request:%d\n",
                    value.un.izroRequest.intent,
                    value.un.izroRequest.request);
                break;
            case SH2_SHAKE_DETECTOR:
                printf("Shake Axis: %c%c%c\n", 
                    (value.un.shakeDetector.shake & SHAKE_X) ? 'X' : '.',
                    (value.un.shakeDetector.shake & SHAKE_Y) ? 'Y' : '.',
                    (value.un.shakeDetector.shake & SHAKE_Z) ? 'Z' : '.');

                break;
            case SH2_STABILITY_CLASSIFIER:
                printf("Stability Classification: %d\n",
                    value.un.stabilityClassifier.classification);
                break;
            case SH2_STABILITY_DETECTOR:
                printf("Stability Detector: %d\n",
                    value.un.stabilityDetector.stability);
                break;
            default:
                printf("Unknown sensor: %d\n", value.sensorId);
                break;
        }
    }

    static int32_t open_sh2_cb(sh2_Hal_t *self)
    {

        digitalWrite(BNO_RST_PIN, LOW);
        std::this_thread::sleep_for(10ms);
        digitalWrite(BNO_RST_PIN, HIGH);
        digitalWrite(BNO_WAK_PIN, LOW);

        return 0;
    }

    static void close_sh2_cb(sh2_Hal_t *self)
    {
        digitalWrite(BNO_RST_PIN, LOW);
    }

    static int32_t read_sh2_cb(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us)
    {
        int32_t ret = 0;

        if (spi_recv_len > 0) {
            if (len >= spi_recv_len) {
                memcpy(pBuffer, spi_rxbuf, spi_recv_len);
                ret = spi_recv_len;

                *t_us = bno_int_timestamp_us;

                spi_recv_len = 0;
            } else {
                ret = SH2_ERR_BAD_PARAM;
                spi_recv_len = 0;
            }
        }

        return ret;
    }

    static int32_t write_sh2_cb(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
    {
        if ((pBuffer == 0) || (len == 0))
        {
            return SH2_ERR_BAD_PARAM;
        }

        int32_t write_ret = spi_write(spi_dev, pBuffer, len);
        if (write_ret < 0) {
            printf("spi_write returned %d. (%s)\n", write_ret, strerror(errno));
        }

    
        return len;
    }

    static uint32_t getTimeUs(sh2_Hal_t *self) 
    {
        auto dur = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(dur.time_since_epoch()).count();
    }

    static void h_intn_cb() 
    {
        bno_int_timestamp_us = getTimeUs(NULL);
        ++bno_interrupts;
        read_packet(false);
        // printf("Interrupted %fms/\n", (float) bno_int_timestamp_us / 1000.f);
    }

    void reset_sh2_cb(sh2_AsyncEvent *pEvent);
};