#pragma once
#include <array>
#include <string>
#include <cstdint>

class UartRingBuffer {
public:
    static constexpr size_t BufferSize = 512;

    bool write(const uint8_t* data, size_t len);
    bool readLine(std::string& outLine);
    bool hasLine() const;
    void clear();

private:
    std::array<uint8_t, BufferSize> buffer{};
    size_t head = 0;
    size_t tail = 0;
};
