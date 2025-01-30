#include <stdint.h>
#include <functional>
#include <thread>
#include <errno.h>
#include <cstring>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

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
uint32_t BNO08x::bno_int_timestamp_us;

//TODO - find settings for BNO085 and BNO086
Packet::Packet() 
{
    
}

Packet::Packet(packet_header hdr) 
{
    Packet(hdr, NULL, 0);
}

Packet::Packet(uint8_t *packet_data, uint16_t packet_len) 
{
    if (packet_data == NULL) {
        return; 
    }

    memcpy(this->packet_data, packet_data, packet_len);
}

Packet::Packet(packet_header hdr, uint8_t *packet_data, uint16_t packet_len) 
{
    if (packet_data == NULL) {
        return; 
    }

    memcpy(this->packet_data, packet_data, packet_len);

    this->header = hdr;
}

BNO08x::BNO08x(std::string dev_file) 
{
    BNO08x::spi_path = dev_file;
    spi_dev = (spi_t *) malloc(sizeof(spi_t));

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

void BNO08x::initialize()
{
    wiringPiSetup();

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
    }

    sh2_SensorId_t id = SH2_GAME_ROTATION_VECTOR;
    sh2_SensorConfig_t config;
    config.reportInterval_us = 10000;
    // printf("Setting sensor config\n");
    sh2_setSensorConfig(id, &config);
}

void BNO08x::service() 
{
    sh2_service();
}

void BNO08x::reset_sh2_cb(sh2_AsyncEvent *pEvent) 
{
    printf("BNO Reset!!");
}

int32_t BNO08x::reportProdIds(sh2_ProductIds_t *ids)
{
    int status;

    // memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(ids);

    if (status < 0) {
        printf("Error from sh2_getProdIds: %d\n", status);
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
