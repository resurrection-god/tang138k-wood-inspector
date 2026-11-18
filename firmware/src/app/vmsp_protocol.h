/**
 * @file vmsp_protocol.h
 * @brief VMSP packet framing: 28-byte Big-Endian header and CRC-32
 *
 * Packet format only: encoding, parsing with integrity checks, and CRC-32
 * (IEEE 802.3). Opcodes, status codes and handlers live in vmsp_service.c;
 * the normative description is docs/protocol/VMSP_PROTOCOL_SPECIFICATION.md.
 */

#ifndef VMSP_PROTOCOL_H
#define VMSP_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ========================================================================= */
/* CONSTANTS                                                                 */
/* ========================================================================= */

#define VMSP_MAGIC_BYTE0        0x56 /* 'V' */
#define VMSP_MAGIC_BYTE1        0x4D /* 'M' */
#define VMSP_MAGIC_BYTE2        0x53 /* 'S' */
#define VMSP_MAGIC_BYTE3        0x50 /* 'P' */

#define VMSP_PROTOCOL_VERSION   0x0002 /* Stack version v0.2 */
#define VMSP_HEADER_SIZE        28     /* Fixed VMSP header size */

#define VMSP_PORT_DISCOVERY     39150  /* UDP discovery */

/* ========================================================================= */
/* HEADER AND PARSE RESULT                                                   */
/* ========================================================================= */

#pragma pack(push, 1)

/**
 * @brief Normative 28-byte VMSP header (Big-Endian on the wire)
 */
typedef struct {
    uint8_t  magic[4];       /**< 'VMSP' (0x56, 0x4D, 0x53, 0x50) [0x00..0x03] */
    uint16_t protocol_ver;   /**< Protocol version (0x0002) [0x04..0x05] */
    uint16_t opcode;         /**< Command / response code [0x06..0x07] */
    uint32_t seq_num;        /**< Transaction number [0x08..0x0B] */
    uint32_t sender_id;      /**< Sender ID (0=PC, 1..8=module) [0x0C..0x0F] */
    uint32_t target_id;      /**< Receiver ID (0=broadcast, 1..8=module) [0x10..0x13] */
    uint32_t payload_len;    /**< Payload size in bytes [0x14..0x17] */
    uint32_t payload_crc32;  /**< CRC-32 IEEE 802.3 of the payload [0x18..0x1B] */
} vmsp_header_t;

#pragma pack(pop)

/**
 * @brief Result of decoding an incoming packet
 */
typedef enum {
    VMSP_DEC_OK = 0,                /**< Packet is valid and ready for processing */
    VMSP_DEC_ERR_NULL,              /**< A null pointer was passed */
    VMSP_DEC_ERR_SHORT_HEADER,      /**< Buffer length is less than 28 bytes */
    VMSP_DEC_ERR_BAD_MAGIC,         /**< The 'VMSP' signature did not match */
    VMSP_DEC_ERR_BAD_VERSION,       /**< Protocol version is not 0x0002 */
    VMSP_DEC_ERR_SHORT_PAYLOAD,     /**< Buffer is shorter than the declared payload_len */
    VMSP_DEC_ERR_BAD_CRC,           /**< Payload CRC-32 did not match */
    VMSP_DEC_ERR_WRONG_TARGET       /**< Packet is addressed to another node */
} vmsp_decode_status_t;

/**
 * @brief Parsed VMSP packet
 */
typedef struct {
    vmsp_header_t  header;          /**< Header with fields in host byte order */
    const uint8_t *payload;         /**< Pointer to the payload */
    uint32_t       payload_len;     /**< Payload length */
} vmsp_packet_t;

/* ========================================================================= */
/* API                                                                       */
/* ========================================================================= */

/**
 * @brief Polynomial CRC-32 checksum (IEEE 802.3 Ethernet)
 */
uint32_t vmsp_crc32(const uint8_t *data, size_t len);

/**
 * @brief Streaming CRC-32 for a body made of several pieces
 * crc = VMSP_CRC32_INIT; crc = vmsp_crc32_update(crc, a, na); ... ; vmsp_crc32_final(crc)
 */
#define VMSP_CRC32_INIT         0xFFFFFFFFUL
#define vmsp_crc32_final(crc)   ((crc) ^ 0xFFFFFFFFUL)
uint32_t vmsp_crc32_update(uint32_t crc, const uint8_t *data, size_t len);

/**
 * @brief Encode a full VMSP packet (28 B header + body + CRC32) into a buffer (unaligned safe)
 */
size_t vmsp_encode_packet(uint16_t opcode, uint32_t seq_num, uint32_t sender_id, uint32_t target_id,
                          const uint8_t *payload, uint32_t plen, uint8_t *out_buf, size_t max_len);

/**
 * @brief Decode and integrity-check an incoming VMSP packet (unaligned safe)
 */
vmsp_decode_status_t vmsp_decode_packet(const uint8_t *buf, size_t buf_len, uint32_t my_node_id, vmsp_packet_t *out_pkt);

#ifdef __cplusplus
}
#endif

#endif /* VMSP_PROTOCOL_H */
