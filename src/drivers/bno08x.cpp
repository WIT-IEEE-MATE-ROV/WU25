#include <thread>
#include <mutex>
#include <cerrno>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <wiringPi.h>

extern "C" {
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
}

#include "drivers/bno08x.hpp"


spi_t *BNO08x::spi_dev;
std::string BNO08x::spi_path;
uint32_t BNO08x::bno_interrupts;
uint8_t BNO08x::spi_txbuf[];
uint8_t BNO08x::spi_rxbuf[];
uint32_t BNO08x::spi_recv_len;
uint8_t BNO08x::sequence_numbers[];
volatile uint32_t BNO08x::bno_int_timestamp_us;
bool BNO08x::spi_busy;

int32_t BNO08x::rx_buflen;
int32_t BNO08x::tx_buflen;

bool BNO08x::in_reset;
bool BNO08x::rx_ready;
bool BNO08x::rx_data_ready;
SpiState_t BNO08x::spi_state;

//TODO - find settings for BNO085 and BNO086
Packet::Packet() : header() {

}

Packet::Packet(PacketHeader hdr) {
    this->header = hdr;
}

Packet::Packet(uint8_t *packet_data, uint16_t packet_len) {
    if (packet_data == nullptr) {
        return;
    }

    memcpy(this->packet_data, packet_data, packet_len);
}

Packet::Packet(PacketHeader hdr, uint8_t *packet_data, uint16_t packet_len) {
    if (packet_data == nullptr) {
        return;
    }

    memcpy(this->packet_data, packet_data, packet_len);

    this->header = hdr;
}

BNO08x::BNO08x(const std::string &dev_file) {
    BNO08x::spi_path = dev_file;
    spi_dev = (spi_t *) malloc(sizeof(spi_t));
    spi_state = SPI_INIT;
    spi_busy = false;
    rx_buflen = 0;


    memset(BNO08x::sequence_numbers, 0, 6);

    BNO08x::bno_interrupts = 0;

    BNO08x::psh2 = (sh2_Hal_t *) malloc(sizeof(sh2_Hal_t));

    BNO08x::psh2->open = open_sh2_cb;
    BNO08x::psh2->close = close_sh2_cb;
    BNO08x::psh2->read = read_sh2_cb;
    BNO08x::psh2->write = write_sh2_cb;
    BNO08x::psh2->getTimeUs = getTimeUs;

}

BNO08x::~BNO08x() {
    printf("Destructing BNO08x\n");
    sh2_close();
    free(psh2);
    spi_free(spi_dev);
}

const char *strerror_sh2(int32_t error) {
    switch (error) {
        case SH2_OK:
            return "Success";
        case SH2_ERR:
            return "General Error";
        case SH2_ERR_BAD_PARAM:
            return "Bad parameter to an API call";
        case SH2_ERR_OP_IN_PROGRESS:
            return "Operation in progress";
        case SH2_ERR_IO:
            return "Error communicating with hub";
        case SH2_ERR_HUB:
            return "Error reported by hub";
        case SH2_ERR_TIMEOUT:
            return "Operation timed out";
        default:
            return "Undefined SH2 Error";
    }
}

const char *spi_state_str(SpiState_t state) {
#ifndef STR
#define STR(x) #x
#endif

    switch (state) {
        case SPI_INIT:
            return STR(SPI_INIT);
        case SPI_DUMMY:
            return STR(SPI_DUMMY);
        case SPI_DFU:
            return STR(SPI_DFU);
        case SPI_IDLE:
            return STR(SPI_IDLE);
        case SPI_RD_HDR:
            return STR(SPI_RD_HDR);
        case SPI_RD_BODY:
            return STR(SPI_RD_BODY);
        case SPI_WRITE:
            return STR(SPI_WRITE);
        default:
            return "Undefined SPI State";
    }

#undef STR
}

void BNO08x::initialize() {
    wiringPiSetup();

    while (false) {
        printf("HIGH\n");
        digitalWrite(BNO_RST_PIN, HIGH);
        digitalWrite(BNO_WAK_PIN, HIGH);
        std::this_thread::sleep_for(2s);
        printf("LOW\n");
        digitalWrite(BNO_RST_PIN, LOW);
        digitalWrite(BNO_WAK_PIN, LOW);
        std::this_thread::sleep_for(2s);
    }

    // printf("Initializing SPI\n");
    spi_init(spi_dev, spi_path.c_str(), spi_mode, spi_bits, spi_speed);
    // printf("SPI initialized\n");

    wiringPiISR(BNO_INT_PIN, INT_EDGE_FALLING, &h_intn_cb);
    // printf("Interrupt created\n");

    int32_t open_ret;
    // printf("Opening sh2\n");
    if ((open_ret = sh2_open(BNO08x::psh2, event_sh2_cb, this)) != SH2_OK) {
        printf("Couldn't open SH2: %d\n", open_ret);
    }

    sh2_setSensorCallback(sensor_event_sh2_cb, this);

    // printf("Getting product id\n");
    sh2_ProductIds_t ids;
    if (reportProdIds(&ids)) {
        printf("Couldn't get product ids\n");
    } else {
        printf("SH2 Product ID count: %u, id[", ids.numEntries);
    }

    sh2_SensorId_t id = SH2_GAME_ROTATION_VECTOR;
    sh2_SensorConfig_t config;
    config.reportInterval_us = 10000;
    // printf("Setting sensor config\n");
    int32_t sensor_config_ret;
    printf("Setting sensor_config\n");
    if ((sensor_config_ret = sh2_setSensorConfig(id, &config)) < 0) {
        printf("Error setting sensor config: %s\n", strerror_sh2(sensor_config_ret));
    }
}

void BNO08x::service() {
    sh2_service();
}


int32_t BNO08x::reportProdIds(sh2_ProductIds_t *ids) {
    int status;

    // memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(ids);

    if (status < 0) {
        printf("Error from sh2_getProdIds: %s\n", strerror_sh2(status));
        return status;
    }

    // Report the results
    for (int n = 0; n < ids->numEntries; n++) {
        printf("Part %d : Version %d.%d.%d Build %d\n",
               ids->entry[n].swPartNumber,
               ids->entry[n].swVersionMajor, ids->entry[n].swVersionMinor,
               ids->entry[n].swVersionPatch, ids->entry[n].swBuildNumber);

        // Wait a bit so we don't overflow the console output.
        // delayUs(10000);
    }

    return SH2_OK;
}

int32_t BNO08x::wait_for_h_intn() {
    auto start_time = std::chrono::high_resolution_clock::now();
    uint32_t start_inc = bno_interrupts;

    while (std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start_time).count() < 3) { // TODO: Lower than 3
        if (bno_interrupts != start_inc) {
            return 0;
        }
    }

    printf("Timed out waiting for interrupt\n");
    return 1;
}

void BNO08x::h_intn_cb() {
    bno_int_timestamp_us = getTimeUs(nullptr);
    ++bno_interrupts;

    in_reset = false;
    rx_ready = true;

    spi_activate();
}

void BNO08x::update_sequence_number(PacketHeader *hdr) {
    sequence_numbers[hdr->channel] = hdr->seq_num;
}

Packet BNO08x::read_packet_type(BNOChannel channel, uint32_t timeout_ms) {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto current_time = start_time;
    Packet pkt;

    while (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() >= timeout_ms) {
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
    return {};
}

uint32_t BNO08x::read_header(PacketHeader *hdr_out, bool wait_for_int) {
    if (wait_for_int && wait_for_h_intn()) {
        printf("Timed out reading header");
        return 1;
    }

    int32_t read_ret;
    spi_busy = true;
    if ((read_ret = spi_read(spi_dev, spi_rxbuf, 4)) < 0) {
        printf("SPI read failed: %s (%d)\n", strerror(errno), read_ret);
        spi_busy = false;
        return 1;
    }

    spi_busy = false;

    hdr_out->channel = (BNOChannel) spi_rxbuf[2];
    hdr_out->seq_num = spi_rxbuf[3];
    hdr_out->data_length = (spi_rxbuf[1] << 8) | spi_rxbuf[0];

    // printf("Read header: len: %u, channel: %u, seqnum: %u\n", hdr_out->data_length, hdr_out->channel, hdr_out->seq_num);
    return 0;
}

Packet BNO08x::read_packet(bool wait_for_int) {
    PacketHeader hdr;
    if (read_header(&hdr, wait_for_int)) {
        printf("Read header failed\n");
        return {};
    }

    bool half_packet = false;

    if (hdr.channel & 0x80) {
        half_packet = true;
    }

    sequence_numbers[hdr.channel] = hdr.seq_num;

    if (hdr.data_length == 0) {
        printf("Read packet length == 0\n");
        return {};
    }

    if (hdr.data_length > SPI_BUFLEN) {
        printf("Can't read everything %u > SPI_BUFLEN, channel = %u, seqnum = %u\n", hdr.data_length, hdr.channel,
               hdr.seq_num);
        return {};
    }

    spi_busy = true;

    spi_read(spi_dev, spi_rxbuf, hdr.data_length);

    spi_busy = false;

    printf("Read: {0x%X, 0x%X, 0x%X}\n", spi_rxbuf[0], spi_rxbuf[1], spi_rxbuf[2]);

    PacketHeader new_hdr;
    new_hdr.channel = (BNOChannel) spi_rxbuf[2];
    new_hdr.seq_num = spi_rxbuf[3];

    new_hdr.data_length = (spi_rxbuf[1] << 8) | spi_rxbuf[0];

    if (half_packet) {
        printf("Error: Read partial packet\n");
        return {};
    }

    sequence_numbers[new_hdr.channel] = new_hdr.seq_num;

    // Set recv len to use in sh2 read callback
    spi_recv_len = new_hdr.data_length;

    return {new_hdr, spi_rxbuf, new_hdr.data_length};
}

uint8_t BNO08x::send_packet(uint8_t channel, uint8_t *data, uint16_t data_len) {
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

    spi_busy = true;
    if (spi_write(spi_dev, spi_txbuf, write_len) < 0) {
        printf("SPI write failed: %s\n", strerror(errno));
        return 1;
    }
    spi_busy = false;

    ++sequence_numbers[channel];

    return 0;
}

int32_t BNO08x::spi_dummy_op() {
    uint8_t dummyTx[2];
    uint8_t dummyRx[2];

    memset(dummyTx, 0xAA, sizeof(dummyTx));

    int32_t ret = 0;
    ret = spi_write(spi_dev, dummyTx, 2);
    if (ret < 0) {
        return ret;
    }
    ret = spi_read(spi_dev, dummyRx, 2);

    return ret;
}

void BNO08x::spi_activate() {
    // printf("Active: State = %s\n", spi_state_str(spi_state));
    if ((spi_state == SPI_IDLE) && (rx_buflen == 0)) {
        if (rx_ready) {
            rx_ready = false;

            if (tx_buflen > 0) {
                spi_state = SPI_WRITE;

                spi_write(spi_dev, spi_txbuf, tx_buflen);
                printf("Write header: len: %u, channel: %u, seqnum: %u\n",
                       (spi_txbuf[0] + (spi_txbuf[1] << 8)) & ~0x8000, spi_txbuf[2], spi_txbuf[3]);
                digitalWrite(BNO_WAK_PIN, LOW);
            } else {
                spi_state = SPI_RD_HDR;

                // Read header
                spi_read(spi_dev, spi_rxbuf, 4);

                // printf("Read header: len: %u, channel: %u, seqnum: %u\n",(spi_rxbuf[0] + (spi_rxbuf[1] << 8)) & ~0x8000, spi_rxbuf[2], spi_rxbuf[3]);
//                printf("hdr: 0x%X 0x%X 0x%X 0x%X\n", spi_rxbuf[0], spi_rxbuf[1], spi_rxbuf[2], spi_rxbuf[3]);
            }
        }
    }

    spi_completed();
}

void BNO08x::spi_completed() {
    uint16_t rx_len = (spi_rxbuf[0] + (spi_rxbuf[1] << 8)) & ~0x8000;

    if (rx_len > SPI_BUFLEN) {
        rx_len = SPI_BUFLEN;
    }

    if (spi_state == SPI_DUMMY) {
        spi_state = SPI_IDLE;
    } else if (spi_state == SPI_RD_HDR) {
        if (rx_len > HEADER_LEN) {
            spi_state = SPI_RD_BODY;

            spi_read(spi_dev, spi_rxbuf + 4, rx_len - 4);
        } else {
            rx_buflen = 0;
            spi_state = SPI_IDLE;
            spi_activate();
        }
    } else if (spi_state == SPI_RD_BODY) {
        rx_buflen = rx_len;

        spi_state = SPI_IDLE;

        spi_activate();
    } else if (spi_state == SPI_WRITE) {
        rx_buflen = (tx_buflen < rx_len) ? tx_buflen : rx_len;

        tx_buflen = 0;

        spi_state = SPI_IDLE;

        spi_activate();
    }
}

///////////////////////////////////////////////////////////////////////////////
// SH2 Callbacks
///////////////////////////////////////////////////////////////////////////////

void BNO08x::reset_sh2_cb(sh2_AsyncEvent *pEvent) {
    printf("BNO Reset!!");
}

std::string shtpevent_to_str(sh2_ShtpEvent_t event) {
#ifndef STR
#define STR(x) #x
#endif

    switch (event) {
        case SH2_SHTP_TX_DISCARD:
            return STR(SH2_SHTP_TX_DISCARD);
        case SH2_SHTP_SHORT_FRAGMENT:
            return STR(SH2_SHTP_SHORT_FRAGMENT);
        case SH2_SHTP_TOO_LARGE_PAYLOADS:
            return STR(SH2_SHTP_TOO_LARGE_PAYLOADS);
        case SH2_SHTP_BAD_RX_CHAN:
            return STR(SH2_SHTP_BAD_RX_CHAN);
        case SH2_SHTP_BAD_TX_CHAN:
            return STR(SH2_SHTP_BAD_TX_CHAN);
        case SH2_SHTP_BAD_FRAGMENT:
            return STR(SH2_SHTP_BAD_FRAGMENT);
        case SH2_SHTP_BAD_SN:
            return STR(SH2_SHTP_BAD_SN);
        case SH2_SHTP_INTERRUPTED_PAYLOAD:
            return STR(SH2_SHTP_INTERRUPTED_PAYLOAD);
        default:
            return "Unknown sh2_ShtpEvent_t";
    }

#undef STR
}

void BNO08x::event_sh2_cb(void *cookie, sh2_AsyncEvent_t *pEvent) {
//    BNO08x *bno = (BNO08x *) cookie;
    auto *bno = reinterpret_cast<BNO08x *>(cookie);

    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET) {
        printf("EventHandler: SH2_RESET\n");
        bno->reset_sh2_cb(pEvent);
    } else if (pEvent->eventId == SH2_SHTP_EVENT) {
        printf("EventHandler  id:SHTP, %s\n", shtpevent_to_str(pEvent->shtpEvent).c_str());

    } else if (pEvent->eventId == SH2_GET_FEATURE_RESP) {
        printf("EventHandler Sensor Config, %d\n", pEvent->sh2SensorConfigResp.sensorId);
    } else {
        printf("EventHandler, unknown event Id: %d\n", pEvent->eventId);
    }
}

void BNO08x::sensor_event_sh2_cb(void *cookie, sh2_SensorEvent_t *pEvent) {
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
                   (double) t,
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y,
                   value.un.rawAccelerometer.z,
                   value.un.rawAccelerometer.timestamp);
            break;

        case SH2_ACCELEROMETER:
            printf("%8.4f Acc: %f %f %f\n",
                   (double) t,
                   (double) value.un.accelerometer.x,
                   (double) value.un.accelerometer.y,
                   (double) value.un.accelerometer.z);
            break;

        case SH2_RAW_GYROSCOPE:
            printf("%8.4f Raw gyro: x:%d y:%d z:%d temp:%d time_us:%d\n",
                   (double) t,
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
                   (double) t,
                   (double) r, (double) i, (double) j, (double) k, (double) acc_deg);
            break;
        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.gameRotationVector.real;
            i = value.un.gameRotationVector.i;
            j = value.un.gameRotationVector.j;
            k = value.un.gameRotationVector.k;
            printf("%8.4f GRV: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f\n",
                   (double) t,
                   (double) r, (double) i, (double) j, (double) k);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            x = value.un.gyroscope.x;
            y = value.un.gyroscope.y;
            z = value.un.gyroscope.z;
            printf("%8.4f GYRO: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   (double) t,
                   (double) x, (double) y, (double) z);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            x = value.un.gyroscopeUncal.x;
            y = value.un.gyroscopeUncal.y;
            z = value.un.gyroscopeUncal.z;
            printf("%8.4f GYRO_UNCAL: "
                   "x:%0.6f y:%0.6f z:%0.6f\n",
                   (double) t,
                   (double) x, (double) y, (double) z);
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
                       (double) t,
                       (double) r, (double) i, (double) j, (double) k,
                       (double) x, (double) y, (double) z);
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

int32_t BNO08x::open_sh2_cb(sh2_Hal_t *self) {
    digitalWrite(BNO_RST_PIN, LOW);

    // spi_dummy_op();
    spi_state = SPI_IDLE;

    rx_buflen = 0;
    tx_buflen = 0;
    rx_data_ready = false;
    rx_ready = false;

    in_reset = true;  // will change back to false when INTN serviced

    std::this_thread::sleep_for(10ms);
    digitalWrite(BNO_RST_PIN, HIGH);
    digitalWrite(BNO_WAK_PIN, LOW);

    return 0;
}

void BNO08x::close_sh2_cb(sh2_Hal_t *self) {
    printf("SH2 Closed\n");
    spi_state = SPI_INIT;
    digitalWrite(BNO_RST_PIN, LOW);
}

int32_t BNO08x::read_sh2_cb(sh2_Hal_t *self, uint8_t *pBuffer, uint32_t len, uint32_t *t_us) {
    // printf("Read cb\n");

    int32_t ret = 0;

    if (rx_buflen > 0) {
        if (len >= rx_buflen) {
            printf("Read SH2 cb\n");
            memcpy(pBuffer, spi_rxbuf, rx_buflen);

            ret = rx_buflen;

            *t_us = bno_int_timestamp_us;

            rx_buflen = 0;
        } else {
            ret = SH2_ERR_BAD_PARAM;
            rx_buflen = 0;
        }

        spi_activate();
    }

    return ret;
}

int32_t BNO08x::write_sh2_cb(sh2_Hal_t *self, uint8_t *pBuffer, uint32_t len) {
    // printf("Write cb\n");
    uint32_t ret = SH2_OK;

    if ((self == nullptr) || (len > SPI_BUFLEN) || ((len > 0) && (pBuffer == nullptr))) {
        return SH2_ERR_BAD_PARAM;
    }

    if (tx_buflen != 0) {
        return 0;
    }

    memcpy(spi_txbuf, pBuffer, len);

    if (pBuffer != nullptr) {
        printf("Write: {0x%X, 0x%X, 0x%X}\n", pBuffer[0], pBuffer[1], pBuffer[2]);
    } else {
        std::cerr << "write_sh2_cb: pBuffer = null" << std::endl;
    }

    tx_buflen = static_cast<int32_t>(len);
    ret = len;

    digitalWrite(BNO_WAK_PIN, LOW);

    return static_cast<int32_t>(ret);
}

uint32_t BNO08x::getTimeUs(sh2_Hal_t *self) {
    auto dur = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(dur.time_since_epoch()).count();
}
