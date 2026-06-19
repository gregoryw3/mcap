#pragma once

// mcap/embedded_writer.hpp
//
// Minimal, header-only MCAP writer for embedded targets (FreeRTOS, bare-metal).
//
// Design constraints:
//   - No STL (no std::vector, std::string, std::unordered_map)
//   - No exceptions (compile with -fno-exceptions)
//   - No RTTI
//   - No dynamic memory allocation
//   - No compression (write-only, flat data section)
//   - Fixed compile-time channel / schema counts via template parameters
//   - C++17 minimum
//
// I/O abstraction:
//   The IO template parameter must be a type with:
//     void write(const uint8_t* data, size_t len);   // write len bytes
//     uint64_t size() const;                          // total bytes written so far
//
// Example (FreeRTOS + FatFS):
//
//   struct FatFsIO {
//       FIL file;
//       uint64_t bytesWritten = 0;
//
//       void write(const uint8_t* data, size_t len) {
//           UINT written;
//           f_write(&file, data, len, &written);
//           bytesWritten += written;
//       }
//       uint64_t size() const { return bytesWritten; }
//   };
//
//   FatFsIO io;
//   f_open(&io.file, "flight.mcap", FA_CREATE_ALWAYS | FA_WRITE);
//
//   mcap::embedded::McapWriter<FatFsIO> writer(io);
//   writer.open("helios");  // profile name
//
//   auto sid = writer.addSchema("helios.Imu", "jsonschema", schemaJson, schemaLen);
//   auto cid = writer.addChannel(sid, "/imu", "json");
//   writer.addMessage(cid, timestamp_ns, timestamp_ns, seq, msgData, msgLen);
//   writer.close();
//
//   f_close(&io.file);

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace mcap {
namespace embedded {

// ─── CRC-32 (ISO 3309) ───────────────────────────────────────────────────────

namespace detail {
    static constexpr uint32_t kCRCPolynomial = 0xEDB88320u;

    inline uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) noexcept {
        crc = ~crc;
        while (len--) {
            crc ^= *data++;
            for (int i = 0; i < 8; ++i)
                crc = (crc >> 1) ^ (kCRCPolynomial & -(crc & 1));
        }
        return ~crc;
    }
} // namespace detail

// ─── Little-endian write helpers ─────────────────────────────────────────────

namespace detail {
    inline void writeU8(uint8_t* buf, size_t& off, uint8_t v) noexcept {
        buf[off++] = v;
    }
    inline void writeU16(uint8_t* buf, size_t& off, uint16_t v) noexcept {
        buf[off++] = (uint8_t)(v);
        buf[off++] = (uint8_t)(v >> 8);
    }
    inline void writeU32(uint8_t* buf, size_t& off, uint32_t v) noexcept {
        buf[off++] = (uint8_t)(v);
        buf[off++] = (uint8_t)(v >> 8);
        buf[off++] = (uint8_t)(v >> 16);
        buf[off++] = (uint8_t)(v >> 24);
    }
    inline void writeU64(uint8_t* buf, size_t& off, uint64_t v) noexcept {
        for (int i = 0; i < 8; ++i) buf[off++] = (uint8_t)(v >> (i * 8));
    }
} // namespace detail

// ─── MCAP opcodes ────────────────────────────────────────────────────────────

namespace opcodes {
    static constexpr uint8_t Header      = 0x01;
    static constexpr uint8_t Footer      = 0x02;
    static constexpr uint8_t Schema      = 0x03;
    static constexpr uint8_t Channel     = 0x04;
    static constexpr uint8_t Message     = 0x05;
    static constexpr uint8_t DataEnd     = 0x0F;
} // namespace opcodes

// ─── MCAP magic bytes ────────────────────────────────────────────────────────

static constexpr uint8_t kMagic[] = {0x89, 0x4D, 0x43, 0x41, 0x50, 0x30, 0x0D, 0x0A};

// ─── McapWriter ──────────────────────────────────────────────────────────────

/**
 * @brief Minimal MCAP file writer for embedded / bare-metal targets.
 *
 * @tparam IO              I/O backend (see file-level docs for required interface)
 * @tparam MaxSchemas      Compile-time limit on registered schemas (default 8)
 * @tparam MaxChannels     Compile-time limit on registered channels (default 16)
 * @tparam ScratchBytes    Size of the stack-allocated record-assembly buffer.
 *                         Must be >= the largest single record you will write.
 *                         Increase if you have very long schema JSON or topic names.
 */
template<typename IO,
         size_t MaxSchemas   = 8,
         size_t MaxChannels  = 16,
         size_t ScratchBytes = 2048>
class McapWriter {
public:
    explicit McapWriter(IO& io) noexcept : io_(io) {}

    McapWriter(const McapWriter&)            = delete;
    McapWriter& operator=(const McapWriter&) = delete;

    /**
     * @brief Write the MCAP magic and Header record. Must be called first.
     *
     * @param profile  MCAP profile string (e.g. "helios", "ros2").
     * @param library  Library identification string.
     */
    void open(const char* profile  = "",
              const char* library  = "mcap-embedded") noexcept {
        io_.write(kMagic, 8);

        const size_t profileLen = strlen(profile);
        const size_t libraryLen = strlen(library);
        const uint64_t bodyLen  = 4 + profileLen + 4 + libraryLen;

        size_t off = 0;
        detail::writeU8 (scratch_, off, opcodes::Header);
        detail::writeU64(scratch_, off, bodyLen);
        detail::writeU32(scratch_, off, (uint32_t)profileLen);
        io_.write(scratch_, off);
        io_.write((const uint8_t*)profile, profileLen);
        off = 0;
        detail::writeU32(scratch_, off, (uint32_t)libraryLen);
        io_.write(scratch_, off);
        io_.write((const uint8_t*)library, libraryLen);
    }

    /**
     * @brief Register a schema. Returns a non-zero schema ID on success,
     *        0 if the schema table is full.
     *
     * @param name      Schema name (e.g. "helios.Imu").
     * @param encoding  Schema encoding (e.g. "jsonschema", "flatbuffer").
     * @param data      Schema data bytes (JSON schema text, etc.).
     * @param dataLen   Length of schema data.
     */
    uint16_t addSchema(const char*    name,
                       const char*    encoding,
                       const uint8_t* data,
                       size_t         dataLen) noexcept {
        if (schemaCount_ >= MaxSchemas) return 0;

        const uint16_t id = nextSchemaID_++;
        schemaIDs_[schemaCount_++] = id;

        const size_t nameLen     = strlen(name);
        const size_t encodingLen = strlen(encoding);
        // body: id(2) + name(4+n) + encoding(4+e) + dataLen(4) + data
        const uint64_t bodyLen = 2 + 4 + nameLen + 4 + encodingLen + 4 + dataLen;

        size_t off = 0;
        detail::writeU8 (scratch_, off, opcodes::Schema);
        detail::writeU64(scratch_, off, bodyLen);
        detail::writeU16(scratch_, off, id);
        detail::writeU32(scratch_, off, (uint32_t)nameLen);
        io_.write(scratch_, off);
        io_.write((const uint8_t*)name, nameLen);
        off = 0;
        detail::writeU32(scratch_, off, (uint32_t)encodingLen);
        io_.write(scratch_, off);
        io_.write((const uint8_t*)encoding, encodingLen);
        off = 0;
        detail::writeU32(scratch_, off, (uint32_t)dataLen);
        io_.write(scratch_, off);
        io_.write(data, dataLen);

        return id;
    }

    /**
     * @brief Register a channel. Returns a channel ID.
     *
     * @param schemaID        Schema ID returned by addSchema().
     * @param topic           Topic string (e.g. "/imu/accel_raw").
     * @param messageEncoding Message encoding (e.g. "json", "flatbuffer").
     * @param metadataKeys    Optional metadata key array (null-terminated strings).
     * @param metadataValues  Optional metadata value array (parallel to keys).
     * @param metadataCount   Number of key/value pairs.
     */
    uint16_t addChannel(uint16_t       schemaID,
                        const char*    topic,
                        const char*    messageEncoding,
                        const char* const* metadataKeys   = nullptr,
                        const char* const* metadataValues = nullptr,
                        size_t             metadataCount  = 0) noexcept {
        if (channelCount_ >= MaxChannels) return 0;

        const uint16_t id = nextChannelID_++;
        channelIDs_[channelCount_++] = id;

        const size_t topicLen   = strlen(topic);
        const size_t encodingLen = strlen(messageEncoding);

        // Compute metadata map byte length
        uint64_t metaLen = 4;  // uint32 count
        for (size_t i = 0; i < metadataCount; ++i) {
            metaLen += 4 + strlen(metadataKeys[i]);
            metaLen += 4 + strlen(metadataValues[i]);
        }

        // body: id(2) + schemaID(2) + topic(4+n) + encoding(4+e) + meta
        const uint64_t bodyLen = 2 + 2 + 4 + topicLen + 4 + encodingLen + metaLen;

        size_t off = 0;
        detail::writeU8 (scratch_, off, opcodes::Channel);
        detail::writeU64(scratch_, off, bodyLen);
        detail::writeU16(scratch_, off, id);
        detail::writeU16(scratch_, off, schemaID);
        detail::writeU32(scratch_, off, (uint32_t)topicLen);
        io_.write(scratch_, off);
        io_.write((const uint8_t*)topic, topicLen);
        off = 0;
        detail::writeU32(scratch_, off, (uint32_t)encodingLen);
        io_.write(scratch_, off);
        io_.write((const uint8_t*)messageEncoding, encodingLen);
        off = 0;
        detail::writeU32(scratch_, off, (uint32_t)metadataCount);
        io_.write(scratch_, off);
        for (size_t i = 0; i < metadataCount; ++i) {
            off = 0;
            const size_t kl = strlen(metadataKeys[i]);
            const size_t vl = strlen(metadataValues[i]);
            detail::writeU32(scratch_, off, (uint32_t)kl);
            io_.write(scratch_, off);
            io_.write((const uint8_t*)metadataKeys[i], kl);
            off = 0;
            detail::writeU32(scratch_, off, (uint32_t)vl);
            io_.write(scratch_, off);
            io_.write((const uint8_t*)metadataValues[i], vl);
        }

        return id;
    }

    /**
     * @brief Write a message record.
     *
     * @param channelID   Channel ID returned by addChannel().
     * @param logTime     Timestamp in nanoseconds (e.g. UNIX epoch or monotonic).
     * @param publishTime Timestamp when the data was produced (often == logTime).
     * @param sequence    Per-channel sequence number (wraps naturally).
     * @param data        Message payload bytes.
     * @param dataLen     Payload length.
     */
    void addMessage(uint16_t       channelID,
                    uint64_t       logTime,
                    uint64_t       publishTime,
                    uint32_t       sequence,
                    const uint8_t* data,
                    size_t         dataLen) noexcept {
        // body: channelID(2) + sequence(4) + logTime(8) + publishTime(8) + data
        const uint64_t bodyLen = 2 + 4 + 8 + 8 + dataLen;

        size_t off = 0;
        detail::writeU8 (scratch_, off, opcodes::Message);
        detail::writeU64(scratch_, off, bodyLen);
        detail::writeU16(scratch_, off, channelID);
        detail::writeU32(scratch_, off, sequence);
        detail::writeU64(scratch_, off, logTime);
        detail::writeU64(scratch_, off, publishTime);
        io_.write(scratch_, off);
        io_.write(data, dataLen);
    }

    /**
     * @brief Finalise the file. Write DataEnd and Footer, then the closing magic.
     *        No further writes should be made after calling close().
     */
    void close() noexcept {
        // DataEnd (no data section CRC — set to 0 for simplicity)
        size_t off = 0;
        detail::writeU8 (scratch_, off, opcodes::DataEnd);
        detail::writeU64(scratch_, off, 4ULL);   // body length
        detail::writeU32(scratch_, off, 0U);     // dataSectionCRC = 0
        io_.write(scratch_, off);

        // Footer — no summary section, so summaryStart = 0
        const uint64_t footerBodyLen = 8 + 8 + 4;
        off = 0;
        detail::writeU8 (scratch_, off, opcodes::Footer);
        detail::writeU64(scratch_, off, footerBodyLen);
        detail::writeU64(scratch_, off, 0ULL);   // summaryStart = 0 (no summary)
        detail::writeU64(scratch_, off, 0ULL);   // summaryOffsetStart = 0
        detail::writeU32(scratch_, off, 0U);     // summaryCRC = 0
        io_.write(scratch_, off);

        io_.write(kMagic, 8);
    }

    // ── Introspection ─────────────────────────────────────────────────────────

    uint16_t schemaCount()  const noexcept { return schemaCount_; }
    uint16_t channelCount() const noexcept { return channelCount_; }

private:
    IO& io_;

    uint16_t nextSchemaID_  = 1;
    uint16_t nextChannelID_ = 0;
    uint16_t schemaCount_   = 0;
    uint16_t channelCount_  = 0;

    // We only track IDs for introspection; the actual schema/channel data is
    // written immediately to IO and not buffered in RAM.
    uint16_t schemaIDs_ [MaxSchemas]  = {};
    uint16_t channelIDs_[MaxChannels] = {};

    // Scratch buffer for assembling record headers (not message payloads).
    uint8_t scratch_[ScratchBytes] = {};
};

} // namespace embedded
} // namespace mcap
