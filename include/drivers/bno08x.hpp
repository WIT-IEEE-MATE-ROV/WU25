//#pragma once
//
//#include <string>
//#include <chrono>
//
// extern "C" {
// #include "sh2.h"
// #include "sh2_SensorValue.h"
// #include "sh2_err.h"
// }
//
//#include "spi.hpp"
//
//using namespace std::chrono_literals;
//
//typedef enum SpiState_e {
//    SPI_INIT,
//    SPI_DUMMY,
//    SPI_DFU,
//    SPI_IDLE,
//    SPI_RD_HDR,
//    SPI_RD_BODY,
//    SPI_WRITE
//} SpiState_t;
//
//typedef enum {
//    BNO_CHANNEL_SHTP_COMMAND = 0,
//    BNO_CHANNEL_EXE = 1,
//    BNO_CHANNEL_SH_CONTROL = 2,
//    BNO_CHANNEL_INPUT_SENSOR_REPORTS = 3,
//    BNO_CHANNEL_WAKE_INPUT_SENSOR_REPORTS = 4,
//    BNO_CHANNEL_GYRO_ROTATION_VECTOR = 5
//} BNOChannel;
//
//typedef struct {
//    uint16_t data_length;
//    BNOChannel channel;
//    uint8_t seq_num;
//} PacketHeader;
//
//struct Packet {
//public:
//    static const uint32_t MAX_PACKET_LEN = 32766 + 4;
//
//    PacketHeader header{.data_length=0, .channel=BNO_CHANNEL_SHTP_COMMAND, .seq_num=0};
//    uint8_t packet_data[MAX_PACKET_LEN] = {};
//
//    Packet();
//
//    explicit Packet(PacketHeader hdr);
//
//    Packet(uint8_t *packet_data, uint16_t packet_len);
//
//    Packet(PacketHeader hdr, uint8_t *packet_data, uint16_t packet_len);
//};
//
//class BNO08x {
//public:
//    explicit BNO08x(const std::string &dev_file = "/dev/spidev0.0");
//
//    ~BNO08x();
//
//    void initialize();
//
//    void service();
//
//private:
//    sh2_Hal_t *psh2;
//    static std::string spi_path;
//    static bool spi_busy;
//    static spi_t *spi_dev;
//    static const uint32_t HEADER_LEN = 4;
//    static const uint32_t SPI_BUFLEN = 32766 + 4;
//    static uint8_t spi_txbuf[SPI_BUFLEN];
//    static uint8_t spi_rxbuf[SPI_BUFLEN];
//    static uint32_t spi_recv_len;
//    static uint8_t sequence_numbers[6];
//
//    static const uint32_t spi_mode = SPI_MODE_3;
//    static const uint32_t spi_speed = 100000;
//    static const uint8_t spi_bits = 8;
//    static char *input_file;
//    static char *output_file;
//    static uint16_t spi_delay;
//    static int spi_verbose;
//
//    static uint32_t bno_interrupts;
//    static volatile uint32_t bno_int_timestamp_us;
//    static bool in_reset;
//    static bool rx_ready;
//    static bool rx_data_ready;
//
//    static uint32_t rx_buflen;
//    static uint32_t tx_buflen;
//
//    static SpiState_t spi_state;
//
//    static const int32_t BNO_RST_PIN = 25;
//    static const int32_t BNO_INT_PIN = 19; // $ gpio edge 113 falling $ gpio mode 113 up
//    static const int32_t BNO_WAK_PIN = 27;
//    static const uint8_t BNO_PRODUCT_ID_REQ = 0xF9;
//
//    static int32_t reportProdIds(sh2_ProductIds_t *ids);
//
//    static int32_t wait_for_h_intn();
//
//    static void update_sequence_number(PacketHeader *hdr);
//
//    static Packet read_packet_type(BNOChannel channel, uint32_t timeout_ms = 5.0);
//
//    static uint32_t read_header(PacketHeader *hdr_out, bool wait_for_int = true);
//
//    static Packet read_packet(bool wait_for_int = true);
//
//    static uint8_t send_packet(uint8_t channel, uint8_t *data, uint16_t data_len);
//
//    static int32_t spi_dummy_op();
//
//    static void spi_activate();
//
//    static void spi_completed();
//
//    static void event_sh2_cb(void *cookie, sh2_AsyncEvent_t *pEvent);
//
//    static void sensor_event_sh2_cb(void *cookie, sh2_SensorEvent_t *pEvent);
//
//    static int32_t open_sh2_cb(sh2_Hal_t *self);
//
//    static void close_sh2_cb(sh2_Hal_t *self);
//
//    static int32_t read_sh2_cb(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
//
//    static int32_t write_sh2_cb(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
//
//    static uint32_t getTimeUs(sh2_Hal_t *self);
//
//    static void h_intn_cb();
//
//    void reset_sh2_cb(sh2_AsyncEvent *pEvent);
//};