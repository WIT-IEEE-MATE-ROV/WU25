//
// Created by ubuntu on 2/1/25.
//

#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>
#include <pthread.h>

#include <wiringPi.h>
#include <gpiod.h>
#include "drivers/spi.hpp"
#include "drivers/BNO08x_ada.hpp"

using namespace std::chrono_literals;
typedef struct {
    uint32_t value;
    pthread_mutex_t lock;
} counter;

counter interrupt_counter;

static volatile uint32_t int_count = 0;

const int32_t BNO_RST_PIN = 25; // 25
const int32_t BNO_INT_PIN = 19; // $ gpio edge 113 falling $ gpio mode 113 up
const int32_t BNO_WAK_PIN = 27;

spi_t *spi_dev = nullptr;

//static Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface
static int8_t _int_pin, _reset_pin;
volatile uint64_t interrupts = 0;

//static Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
//static HardwareSerial *uart_dev = NULL;

static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

static bool spihal_wait_for_int(void);

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us);

static void spihal_close(sh2_Hal_t *self);

static int spihal_open(sh2_Hal_t *self);

static uint32_t hal_getTimeUs(sh2_Hal_t *self);

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);

static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);

static void hal_hardwareReset(void);

uint32_t increment_counter(counter *c) {
    pthread_mutex_lock(&c->lock);
    uint32_t new_value = ++c->value;
    pthread_mutex_unlock(&c->lock);

    return new_value;
}

uint32_t counter_get(counter *c) {
    pthread_mutex_lock(&c->lock);
    uint32_t rc = c->value;
    pthread_mutex_unlock(&c->lock);
    return rc;
}

/**
 * @brief Construct a new Adafruit_BNO08x::Adafruit_BNO08x object
 *
 * @param reset_pin The arduino pin # connected to the BNO Reset pin
 */
Adafruit_BNO08x::Adafruit_BNO08x() = default;

/**
 * @brief Destroy the Adafruit_BNO08x::Adafruit_BNO08x object
 *
 */
Adafruit_BNO08x::~Adafruit_BNO08x(void) {
    // if (temp_sensor)
    //   delete temp_sensor;
    digitalWrite(BNO_RST_PIN, LOW);
    digitalWrite(BNO_WAK_PIN, HIGH);
    spi_free(spi_dev);
    sh2_close();
}


void h_intn_cb() {
//    uint32_t v = increment_counter(&interrupt_counter);
    int_count = int_count + 1;
//    printf("INT: %u\n", int_count);
}

/*!  @brief Initializer for post i2c/spi init
 *   @returns True if chip identified and initialized
 */
bool Adafruit_BNO08x::init() {
    interrupt_counter.value = 0;
    pthread_mutex_init(&interrupt_counter.lock, NULL);

    printf("Setting up wiringPi\n");
    wiringPiSetup();

    printf("Setting up spi\n");
    if (spi_dev == nullptr) {
        spi_dev = (spi_t *) malloc(sizeof(spi_t));
        spi_init(spi_dev, "/dev/spidev0.0", SPI_MODE_3, 8, 100000);
    }

    wiringPiISR(BNO_INT_PIN, INT_EDGE_FALLING, &h_intn_cb);


    _HAL.open = spihal_open;
    _HAL.close = spihal_close;
    _HAL.read = spihal_read;
    _HAL.write = spihal_write;
    _HAL.getTimeUs = hal_getTimeUs;


    int status;

    printf("Resetting sensor\n");
    hardwareReset();

    // Open SH2 interface (also registers non-sensor event handler.)
    printf("Starting SH2\n");
    status = sh2_open(&_HAL, hal_callback, NULL);
    if (status != SH2_OK) {
        return false;
    }

    // Check connection partially by getting the product id's
    printf("Retrieving product ids\n");
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
    if (status != SH2_OK) {
        return false;
    }

    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

    return true;
}

/**
 * @brief Reset the device using the Reset pin
 *
 */
void Adafruit_BNO08x::hardwareReset(void) { hal_hardwareReset(); }

/**
 * @brief Check if a reset has occured
 *
 * @return true: a reset has occured false: no reset has occoured
 */
bool Adafruit_BNO08x::wasReset(void) {
    bool x = _reset_occurred;
    _reset_occurred = false;

    return x;
}

/**
 * @brief Fill the given sensor value object with a new report
 *
 * @param value Pointer to an sh2_SensorValue_t struct to fil
 * @return true: The report object was filled with a new report
 * @return false: No new report available to fill
 */
bool Adafruit_BNO08x::getSensorEvent(sh2_SensorValue_t *value) {
    _sensor_value = value;

    value->timestamp = 0;

    sh2_service();

    if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        // no new events
        return false;
    }

    return true;
}

/**
 * @brief Enable the given report type
 *
 * @param sensorId The report ID to enable
 * @param interval_us The update interval for reports to be generated, in
 * microseconds
 * @return true: success false: failure
 */
bool Adafruit_BNO08x::enableReport(sh2_SensorId_t sensorId,
                                   uint32_t interval_us) {
    static sh2_SensorConfig_t config;

    // These sensor options are disabled or not used in most cases
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    config.reportInterval_us = interval_us;
    int status = sh2_setSensorConfig(sensorId, &config);

    if (status != SH2_OK) {
        return false;
    }

    return true;
}

/**************************************** UART interface
 * ***********************************************************/

static int spihal_open(sh2_Hal_t *self) {
    // Serial.println("SPI HAL open");
    digitalWrite(BNO_RST_PIN, HIGH);
    digitalWrite(BNO_WAK_PIN, LOW);

    spihal_wait_for_int();

    return 0;
}

static bool spihal_wait_for_int() {
//    auto start_time = std::chrono::high_resolution_clock::now();
//    uint32_t start_count = counter_get(&interrupt_counter);
//
//    while (start_count == counter_get(&interrupt_counter)) {
//        auto now = std::chrono::high_resolution_clock::now();
//        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
//        if (elapsed_ms > 2000) {
//            printf("Timed out waiting for int: %u\n", counter_get(&interrupt_counter));
//            return false;
//        }
//    }
//    return true;
//    printf("Waiting\n");
//
//    int success = waitForInterrupt(BNO_INT_PIN, 2000) > 0;
//    if (success) {
//        printf("Done\n");
//    } else {
//        printf("Timed out\n");
//    }
//    return success;
    uint64_t start_int = int_count;

    auto start_time = std::chrono::high_resolution_clock::now();
    while (start_int == int_count) {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed_ms > 2000) {
            printf("Timed out waiting for int: %u\n", int_count);
            hal_hardwareReset();
            return false;
        }
    }
    return true;

//    for (int i = 0; i < 500000000; i++) {
//        if (!digitalRead(BNO_INT_PIN)) {
//            printf("\n");
//            return true;
//        }
//         printf(".");
//
//    }
//    printf("Timed out!\n");
//    hal_hardwareReset();

    return false;
}

static void spihal_close(sh2_Hal_t *self) {
    // Serial.println("SPI HAL close");
}

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
//     printf("SPI HAL read\n");

    uint16_t packet_size = 0;

    if (!spihal_wait_for_int()) {
        return 0;
    }

//    if (!spi_dev->read(pBuffer, 4, 0x00)) {
//        return 0;
//    }

    int read_ret;
    if ((read_ret = spi_read(spi_dev, pBuffer, 4)) < 0) {
        printf("SPI read failed: %d\n", read_ret);
        return 0;
    }

    // Determine amount to read
    packet_size = (uint16_t) pBuffer[0] | (uint16_t) pBuffer[1] << 8;
    // Unset the "continue" bit
    packet_size &= ~0x8000;

//    printf("Read SHTP header. Packet size: %u, buffer size: %u\n", packet_size, len);
//    printf("Packet size: ");
//    printf(packet_size);
//    printf(" & buffer size: ");
//    printf(len);

    if (packet_size > len) {
        return 0;
    }

    if (!spihal_wait_for_int()) {
        return 0;
    }

//    if (!spi_dev->read(pBuffer, packet_size, 0x00)) {
//        return 0;
//    }

    if ((read_ret = spi_read(spi_dev, pBuffer, packet_size)) < 0) {
        printf("SPI read failed: %d\n", read_ret);
        return 0;
    }


    return packet_size;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    // Serial.print("SPI HAL write packet size: ");
    // Serial.println(len);

    if (!spihal_wait_for_int()) {
        return 0;
    }

//    spi_dev->write(pBuffer, len);
    int write_ret;
    if ((write_ret = spi_write(spi_dev, pBuffer, static_cast<int>(len))) < 0) {
        printf("SPI write failed: %d\n", write_ret);
    }

    return static_cast<int>(len);
}

/**************************************** HAL interface
 * ***********************************************************/

static void hal_hardwareReset(void) {
//    if (_reset_pin != -1) {
    printf("BNO08x Hardware reset\n");
    digitalWrite(BNO_RST_PIN, LOW);
    digitalWrite(BNO_WAK_PIN, HIGH);
    std::this_thread::sleep_for(1000ms);
    digitalWrite(BNO_RST_PIN, HIGH);
    digitalWrite(BNO_WAK_PIN, LOW);
//    }
}

static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    auto dur = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(dur.time_since_epoch()).count();
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET) {
        // Serial.println("Reset!");
        _reset_occurred = true;
    }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
    int rc;

    // Serial.println("Got an event!");

    rc = sh2_decodeSensorEvent(_sensor_value, event);
    if (rc != SH2_OK) {
        printf("BNO08x - Error decoding sensor event\n");
        _sensor_value->timestamp = 0;
        return;
    }
}