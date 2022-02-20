/*
* ESPAsyncE131.h
*
* Project: ESPAsyncE131 - Asynchronous E.131 (sACN) library for Arduino ESP8266 and ESP32
* Copyright (c) 2019 Shelby Merrick
* http://www.forkineye.com
*
*  This program is provided free for you to use in any way that you wish,
*  subject to the laws and regulations where you are using it.  Due diligence
*  is strongly suggested before using this code.  Please give credit where due.
*
*  The Author makes no warranty of any kind, express or implied, with regard
*  to this program or the documentation contained in this document.  The
*  Author shall not be liable in any event for incidental or consequential
*  damages in connection with, or arising out of, the furnishing, performance
*  or use of these programs.
*
*/

#ifndef ESPASYNCE131_H_
#define ESPASYNCE131_H_

#ifdef ESP32
#include <WiFi.h>
#include <AsyncUDP.h>
#elif defined (ESP8266)
#include <ESPAsyncUDP.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#else
#error Platform not supported
#endif

#include <lwip/ip_addr.h>
#include <lwip/igmp.h>
#include <Arduino.h>
#include "RingBuf.h"

#if LWIP_VERSION_MAJOR == 1
typedef struct ip_addr ip4_addr_t;
#endif

// Defaults
#define E131_DEFAULT_PORT 5568

// E1.31 Packet Offsets
#define E131_ROOT_PREAMBLE_SIZE 0
#define E131_ROOT_POSTAMBLE_SIZE 2
#define E131_ROOT_ID 4
#define E131_ROOT_FLENGTH 16
#define E131_ROOT_VECTOR 18
#define E131_ROOT_CID 22

#define E131_FRAME_FLENGTH 38
#define E131_FRAME_VECTOR 40
#define E131_FRAME_SOURCE 44
#define E131_FRAME_PRIORITY 108
#define E131_FRAME_RESERVED 109
#define E131_FRAME_SEQ 111
#define E131_FRAME_OPT 112
#define E131_FRAME_UNIVERSE 113

#define E131_DMP_FLENGTH 115
#define E131_DMP_VECTOR 117
#define E131_DMP_TYPE 118
#define E131_DMP_ADDR_FIRST 119
#define E131_DMP_ADDR_INC 121
#define E131_DMP_COUNT 123
#define E131_DMP_DATA 125

/* E1.31 Private Constants */
const uint16_t 	_E131_PREAMBLE_SIZE = 0x0010;
const uint16_t 	_E131_POSTAMBLE_SIZE = 0x0000;
const uint8_t 	_E131_ACN_PID[] = {0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00};
const uint32_t 	_E131_ROOT_VECTOR = 0x00000004;
const uint32_t 	_E131_FRAME_VECTOR = 0x00000002;
const uint8_t 	_E131_FRAME_OPTIONS = 0x40;
const uint8_t 	_E131_DMP_VECTOR = 0x02;
const uint8_t 	_E131_DMP_TYPE = 0xa1;
const uint16_t 	_E131_DMP_FIRST_ADDR = 0x0000;
const uint16_t 	_E131_DMP_ADDR_INC = 0x0001;

/* E1.31 Public Constants */
const uint16_t 	E131_DEFAULT_PORT_srv = 5568;
const uint8_t 	E131_DEFAULT_PRIORITY = 0x64;

// E1.31 Packet Structure
typedef union {
    struct {
        // Root Layer
        uint16_t preamble_size;
        uint16_t postamble_size;
        uint8_t  acn_id[12];
        uint16_t root_flength;
        uint32_t root_vector;
        uint8_t  cid[16];

        // Frame Layer
        uint16_t frame_flength;
        uint32_t frame_vector;
        uint8_t  source_name[64];
        uint8_t  priority;
        uint16_t reserved;
        uint8_t  sequence_number;
        uint8_t  options;
        uint16_t universe;

        // DMP Layer
        uint16_t dmp_flength;
        uint8_t  dmp_vector;
        uint8_t  type;
        uint16_t first_address;
        uint16_t address_increment;
        uint16_t property_value_count;
        uint8_t  property_values[513];
    } __attribute__((packed));

    uint8_t raw[638];
} e131_packet_t;

/* E1.31 Packet Type */
/* All packet contents shall be transmitted in network byte order (big endian) */
typedef union {
  struct {
    struct { /* ACN Root Layer: 38 bytes */
      uint16_t preamble_size;    /* Preamble Size */
      uint16_t postamble_size;   /* Post-amble Size */
      uint8_t  acn_pid[12];      /* ACN Packet Identifier */
      uint16_t flength;          /* Flags (high 4 bits) & Length (low 12 bits) */
      uint32_t vector;           /* Layer Vector */
      uint8_t  cid[16];          /* Component Identifier (UUID) */
    } __attribute__((packed)) root;

    struct { /* Framing Layer: 77 bytes */
      uint16_t flength;          /* Flags (high 4 bits) & Length (low 12 bits) */
      uint32_t vector;           /* Layer Vector */
      uint8_t  source_name[64];  /* User Assigned Name of Source (UTF-8) */
      uint8_t  priority;         /* Packet Priority (0-200, default 100) */
      uint16_t reserved;         /* Reserved (should be always 0) */
      uint8_t  seq_number;       /* Sequence Number (detect duplicates or out of order packets) */
      uint8_t  options;          /* Options Flags (bit 7: preview data, bit 6: stream terminated) */
      uint16_t universe;         /* DMX Universe Number */
    } __attribute__((packed)) frame;

    struct { /* Device Management Protocol (DMP) Layer: 523 bytes */
      uint16_t flength;          /* Flags (high 4 bits) / Length (low 12 bits) */
      uint8_t  vector;           /* Layer Vector */
      uint8_t  type;             /* Address Type & Data Type */
      uint16_t first_addr;       /* First Property Address */
      uint16_t addr_inc;         /* Address Increment */
      uint16_t prop_val_cnt;     /* Property Value Count (1 + number of slots) */
      uint8_t  prop_val[513];    /* Property Values (DMX start code + slots data) */
    } __attribute__((packed)) dmp;
  } __attribute__((packed));

  uint8_t raw[638]; /* raw buffer view: 638 bytes */
} e131_packet_tx_t;

// Error Types
typedef enum {
    ERROR_NONE,
    ERROR_IGNORE,
    ERROR_ACN_ID,
    ERROR_PACKET_SIZE,
    ERROR_VECTOR_ROOT,
    ERROR_VECTOR_FRAME,
    ERROR_VECTOR_DMP
} e131_error_t;

// E1.31 Listener Types
typedef enum {
    E131_UNICAST,
    E131_MULTICAST
} e131_listen_t;

// Status structure
typedef struct {
    uint32_t    num_packets;
    uint32_t    packet_errors;
    IPAddress   last_clientIP;
    uint16_t    last_clientPort;
    unsigned long    last_seen;
} e131_stats_t;

typedef uint16_t ESPAsyncE131PortId;

class ESPAsyncE131 {
 private:
    // Constants for packet validation
    static const uint8_t ACN_ID[];
    static const uint32_t VECTOR_ROOT = 4;
    static const uint32_t VECTOR_FRAME = 2;
    static const uint8_t VECTOR_DMP = 2;

    e131_packet_t   *sbuff;     // Pointer to scratch packet buffer
    AsyncUDP        udp;        // AsyncUDP
    RingBuf         *pbuff;     // Ring Buffer of universe packet buffers
    e131_packet_tx_t pbuff3;  /* Packet buffer */
    e131_packet_tx_t pbuff4;   /* Double buffer */
    e131_packet_tx_t *pwbuffTX; /* Pointer to working packet TX buffer */
    void            * UserInfo = nullptr;

    // Internal Initializers
    bool initUnicast();
    bool initMulticast(uint16_t universe, uint8_t n = 1);

    // Packet parser callback
    void parsePacket(AsyncUDPPacket _packet);

    void (*PacketCallback)(e131_packet_t* ReceivedData, void* UserInfo) = nullptr;
    ESPAsyncE131PortId E131_ListenPort = E131_DEFAULT_PORT;

    uint16_t swapf_uint16(uint16_t x);
    uint32_t swapf_uint32(uint32_t x);

 public:
    e131_stats_t  stats;    // Statistics tracker
    e131_packet_tx_t *packetTX;         /* Pointer to last valid TX packet */
    ESPAsyncE131(uint8_t buffers = 1);

    // Generic UDP listener, no physical or IP configuration
    bool begin (e131_listen_t type, uint16_t universe = 1, uint8_t n = 1);
    bool begin (e131_listen_t type, ESPAsyncE131PortId UdpPortId, uint16_t universe, uint8_t n);

    // Ring buffer access
    inline bool isEmpty() { return pbuff->isEmpty(pbuff); }
    inline void *pull(e131_packet_t *packet) { return pbuff->pull(pbuff, packet); }

    // Callback support
    void registerCallback(void* _UserInfo, void (*cbFunction)(e131_packet_t*, void*)) { PacketCallback = cbFunction; UserInfo = _UserInfo; }

    // Diag functions
    void dumpError(e131_error_t error);

    // void stopUdp(void);
    // void connectMulticast(uint16_t universe);
    // void dumpPacket(int packetNo);
    void setRGB(const uint8_t channel,const uint8_t dRed,const uint8_t dGreen,const uint8_t dBlue);
    void setSourceName(const char *source_name); //
    void setSequenceNumber(const int  seq_number);
    void setData(const int channel, const int dmxVal );
    int  setPacketHeader(const uint16_t universe, const uint16_t num_channels);
    void fillSendBuffer(uint8_t fillData);
    void clearSendBuffer(void );
    void sendPacket(uint16_t universe);
};

#endif  // ESPASYNCE131_H_
