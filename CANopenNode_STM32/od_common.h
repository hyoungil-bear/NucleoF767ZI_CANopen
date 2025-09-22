// Common OD types and helpers
// Generated on 2025-09-19T04:12:59
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    ODT_BOOLEAN = 0x0001,
    ODT_INTEGER8 = 0x0002,
    ODT_INTEGER16 = 0x0003,
    ODT_INTEGER32 = 0x0004,
    ODT_UNSIGNED8 = 0x0005,
    ODT_UNSIGNED16 = 0x0006,
    ODT_UNSIGNED32 = 0x0007,
    ODT_REAL32 = 0x0008,
    ODT_VISIBLE_STRING = 0x0009,
    ODT_OCTET_STRING = 0x000A,
    ODT_UNICODE_STRING = 0x000B,
    ODT_TIME_OF_DAY = 0x000C,
    ODT_TIME_DIFFERENCE = 0x000D,
    ODT_DOMAIN = 0x0010,
    ODT_INTEGER24 = 0x0011,
    ODT_REAL64 = 0x0012,
    ODT_INTEGER40 = 0x0013,
    ODT_INTEGER48 = 0x0014,
    ODT_INTEGER56 = 0x0015,
    ODT_INTEGER64 = 0x0016,
    ODT_UNSIGNED24 = 0x0017,
    ODT_UNSIGNED40 = 0x0018,
    ODT_UNSIGNED48 = 0x0019,
    ODT_UNSIGNED56 = 0x001A,
    ODT_UNSIGNED64 = 0x001B,
} OD_DataType;

typedef enum { 
    OD_ACCESS_RO=0, 
    OD_ACCESS_WO=1, 
    OD_ACCESS_RW=2 
} OD_Access;

typedef struct {
    uint16_t index;
    uint8_t subindex;
    OD_DataType datatype;
    OD_Access access;
    const char* name;
} OD_Entry;

typedef struct {
    uint16_t index;    // 0x1400..0x15FF (RPDO) or 0x1800..0x19FF (TPDO)
    uint8_t  is_tx;    // 0=RPDO, 1=TPDO
    uint8_t  pdo_num;  // sequential number (index-base)
    uint32_t cob_id;   // sub1
    uint8_t  transmission_type; // sub2
    uint16_t inhibit_time;      // sub3 (us or 100us depending on device)
    uint16_t event_timer;       // sub5 (ms)
    uint16_t sync_start;        // sub6
} OD_PDOComm;

typedef struct {
    uint16_t idx;
    uint8_t  sub;
    uint8_t  len_bits;
} OD_PDOMappingEntry;

typedef struct {
    uint16_t index;   // 0x1600..0x17FF (RPDO) or 0x1A00..0x1BFF (TPDO)
    uint8_t  is_tx;   // 0=RPDO, 1=TPDO
    uint8_t  pdo_num; // sequential number (index-base)
    uint8_t  mapped_count;
    OD_PDOMappingEntry map[8];  // up to 8 entries
} OD_PDOMapping;

typedef struct {
    uint16_t index;   // 0x1200..0x127F
    uint8_t  server;  // server number (index-0x1200)
    uint32_t cobid_cs; // sub1: COB-ID SDO Client->Server (Rx)
    uint32_t cobid_sc; // sub2: COB-ID SDO Server->Client (Tx)
} OD_SDOServerParam;

typedef struct {
    uint16_t index;   // 0x1280..0x12FF
    uint8_t  client;  // client number (index-0x1280)
    uint32_t cobid_sc; // sub1: COB-ID SDO Client->Server (Rx) from client perspective
    uint32_t cobid_cs; // sub2: COB-ID SDO Server->Client (Tx) from client perspective
} OD_SDOClientParam;


#if 0
void debug_pdo_tpdo(void) {
    for (uint32_t i = 0; i < ISV2_CAN_TPDO_COMM_COUNT; ++i) {
        const OD_PDOComm* c = &ISV2_CAN_TPDO_COMM[i];
        printf("TPDO%u COB-ID=0x%08X, TT=0x%02X, ET=%u\n",
               c->pdo_num+1, c->cob_id, c->transmission_type, c->event_timer);
    }
}

void print_tpdo1_map(void) {
    for (uint32_t i = 0; i < ISV2_CAN_TPDO_MAP_COUNT; ++i) {
        if (ISV2_CAN_TPDO_MAP[i].pdo_num == 0) { // TPDO1
            const OD_PDOMapping* m = &ISV2_CAN_TPDO_MAP[i];
            for (uint8_t k = 0; k < m->mapped_count; ++k) {
                const OD_PDOMappingEntry* e = &m->map[k];
                printf("Map %u: 0x%04X/%u (%u bits)\n", k+1, e->idx, e->sub, e->len_bits);
            }
        }
    }
}
#endif