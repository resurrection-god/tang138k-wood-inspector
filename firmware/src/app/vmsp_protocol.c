/**
 * @file vmsp_protocol.c
 * @brief VMSP packet framing: encoding, parsing and CRC-32
 *
 * No dynamic memory, safe for unaligned memory access.
 */

#include "vmsp_protocol.h"
#include <string.h>

/* ========================================================================= */
/* SAFE UNALIGNED BIG-ENDIAN ACCESS FUNCTIONS                                */
/* ========================================================================= */

static inline uint16_t read_be16(const uint8_t *p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static inline uint32_t read_be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  |
           ((uint32_t)p[3]);
}

static inline void write_be16(uint8_t *p, uint16_t val) {
    p[0] = (uint8_t)((val >> 8) & 0xFF);
    p[1] = (uint8_t)(val & 0xFF);
}

static inline void write_be32(uint8_t *p, uint32_t val) {
    p[0] = (uint8_t)((val >> 24) & 0xFF);
    p[1] = (uint8_t)((val >> 16) & 0xFF);
    p[2] = (uint8_t)((val >> 8) & 0xFF);
    p[3] = (uint8_t)(val & 0xFF);
}

/* ========================================================================= */
/* CRC-32 TABLE (IEEE 802.3 ETHERNET POLYNOMIAL 0xEDB88320)                   */
/* ========================================================================= */

static uint32_t s_crc32_table[4][256];
static bool s_crc32_table_inited = false;

static void init_crc32_table(void) {
    if (s_crc32_table_inited) return;
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int j = 0; j < 8; j++) {
            c = (c & 1) ? (0xEDB88320UL ^ (c >> 1)) : (c >> 1);
        }
        s_crc32_table[0][i] = c;
    }
    for (uint32_t i = 0; i < 256; i++) {
        s_crc32_table[1][i] = (s_crc32_table[0][i] >> 8) ^ s_crc32_table[0][s_crc32_table[0][i] & 0xFF];
        s_crc32_table[2][i] = (s_crc32_table[1][i] >> 8) ^ s_crc32_table[0][s_crc32_table[1][i] & 0xFF];
        s_crc32_table[3][i] = (s_crc32_table[2][i] >> 8) ^ s_crc32_table[0][s_crc32_table[2][i] & 0xFF];
    }
    s_crc32_table_inited = true;
}

/* Streaming CRC-32 (slicing-by-4): 4 bytes per step for 32-bit RISC-V */
uint32_t vmsp_crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    if (!s_crc32_table_inited) init_crc32_table();
    if (!data || len == 0) return crc;

    const uint8_t *p = data;
    while (len > 0 && ((uintptr_t)p & 3)) {
        crc = s_crc32_table[0][(crc ^ *p++) & 0xFF] ^ (crc >> 8);
        len--;
    }

    const uint32_t *p32 = (const uint32_t *)p;
    while (len >= 4) {
        uint32_t x = crc ^ *p32++;
        crc = s_crc32_table[3][x & 0xFF] ^
              s_crc32_table[2][(x >> 8) & 0xFF] ^
              s_crc32_table[1][(x >> 16) & 0xFF] ^
              s_crc32_table[0][(x >> 24) & 0xFF];
        len -= 4;
    }

    p = (const uint8_t *)p32;
    while (len > 0) {
        crc = s_crc32_table[0][(crc ^ *p++) & 0xFF] ^ (crc >> 8);
        len--;
    }
    return crc;
}

uint32_t vmsp_crc32(const uint8_t *data, size_t len) {
    if (!data || len == 0) return 0;
    return vmsp_crc32_final(vmsp_crc32_update(VMSP_CRC32_INIT, data, len));
}

/* ========================================================================= */
/* PACKET ENCODING                                                           */
/* ========================================================================= */

static size_t vmsp_encode_header(const vmsp_header_t *hdr, uint8_t *out_buf, size_t max_len) {
    if (!hdr || !out_buf || max_len < VMSP_HEADER_SIZE) return 0;

    out_buf[0] = VMSP_MAGIC_BYTE0;
    out_buf[1] = VMSP_MAGIC_BYTE1;
    out_buf[2] = VMSP_MAGIC_BYTE2;
    out_buf[3] = VMSP_MAGIC_BYTE3;

    write_be16(out_buf + 4, hdr->protocol_ver ? hdr->protocol_ver : VMSP_PROTOCOL_VERSION);
    write_be16(out_buf + 6, hdr->opcode);
    write_be32(out_buf + 8, hdr->seq_num);
    write_be32(out_buf + 12, hdr->sender_id);
    write_be32(out_buf + 16, hdr->target_id);
    write_be32(out_buf + 20, hdr->payload_len);
    write_be32(out_buf + 24, hdr->payload_crc32);

    return VMSP_HEADER_SIZE;
}

size_t vmsp_encode_packet(uint16_t opcode, uint32_t seq_num, uint32_t sender_id, uint32_t target_id,
                          const uint8_t *payload, uint32_t plen, uint8_t *out_buf, size_t max_len) {
    if (!out_buf || max_len < VMSP_HEADER_SIZE + plen) return 0;
    if (plen > 0 && !payload) return 0;

    vmsp_header_t hdr;
    hdr.magic[0] = VMSP_MAGIC_BYTE0;
    hdr.magic[1] = VMSP_MAGIC_BYTE1;
    hdr.magic[2] = VMSP_MAGIC_BYTE2;
    hdr.magic[3] = VMSP_MAGIC_BYTE3;
    hdr.protocol_ver = VMSP_PROTOCOL_VERSION;
    hdr.opcode = opcode;
    hdr.seq_num = seq_num;
    hdr.sender_id = sender_id;
    hdr.target_id = target_id;
    hdr.payload_len = plen;
    hdr.payload_crc32 = (payload && plen > 0) ? vmsp_crc32(payload, plen) : 0;

    size_t hdr_len = vmsp_encode_header(&hdr, out_buf, max_len);
    if (hdr_len != VMSP_HEADER_SIZE) return 0;

    if (payload && plen > 0) {
        memcpy(out_buf + VMSP_HEADER_SIZE, payload, plen);
    }

    return VMSP_HEADER_SIZE + plen;
}

/* ========================================================================= */
/* DECODING AND VALIDATION OF INCOMING PACKETS                               */
/* ========================================================================= */

vmsp_decode_status_t vmsp_decode_packet(const uint8_t *buf, size_t buf_len, uint32_t my_node_id, vmsp_packet_t *out_pkt) {
    if (!buf || !out_pkt) return VMSP_DEC_ERR_NULL;
    if (buf_len < VMSP_HEADER_SIZE) return VMSP_DEC_ERR_SHORT_HEADER;

    if (buf[0] != VMSP_MAGIC_BYTE0 || buf[1] != VMSP_MAGIC_BYTE1 ||
        buf[2] != VMSP_MAGIC_BYTE2 || buf[3] != VMSP_MAGIC_BYTE3) {
        return VMSP_DEC_ERR_BAD_MAGIC;
    }

    out_pkt->header.magic[0] = buf[0];
    out_pkt->header.magic[1] = buf[1];
    out_pkt->header.magic[2] = buf[2];
    out_pkt->header.magic[3] = buf[3];

    out_pkt->header.protocol_ver = read_be16(buf + 4);
    out_pkt->header.opcode = read_be16(buf + 6);
    out_pkt->header.seq_num = read_be32(buf + 8);
    out_pkt->header.sender_id = read_be32(buf + 12);
    out_pkt->header.target_id = read_be32(buf + 16);
    out_pkt->header.payload_len = read_be32(buf + 20);
    out_pkt->header.payload_crc32 = read_be32(buf + 24);

    if (out_pkt->header.protocol_ver != VMSP_PROTOCOL_VERSION) {
        return VMSP_DEC_ERR_BAD_VERSION;
    }

    if (buf_len < VMSP_HEADER_SIZE + (size_t)out_pkt->header.payload_len) {
        return VMSP_DEC_ERR_SHORT_PAYLOAD;
    }

    /* Addressing check (broadcast 0 or a match with node_id) */
    if (my_node_id != 0 && out_pkt->header.target_id != 0 && out_pkt->header.target_id != my_node_id) {
        return VMSP_DEC_ERR_WRONG_TARGET;
    }

    out_pkt->payload = (out_pkt->header.payload_len > 0) ? (buf + VMSP_HEADER_SIZE) : NULL;
    out_pkt->payload_len = out_pkt->header.payload_len;

    /* Strict CRC-32 check of the body when a payload is present */
    if (out_pkt->payload_len > 0) {
        uint32_t calc_crc = vmsp_crc32(out_pkt->payload, out_pkt->payload_len);
        if (calc_crc != out_pkt->header.payload_crc32) {
            return VMSP_DEC_ERR_BAD_CRC;
        }
    }

    return VMSP_DEC_OK;
}
