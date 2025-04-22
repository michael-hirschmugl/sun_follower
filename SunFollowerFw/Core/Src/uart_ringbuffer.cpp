#include "uart_ringbuffer.hpp"

bool UartRingBuffer::write(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        size_t next = (head + 1) % BufferSize;
        if (next == tail) return false; // buffer full
        buffer[head] = data[i];
        head = next;
    }
    return true;
}

bool UartRingBuffer::hasLine() const {
    size_t i = tail;
    while (i != head) {
        if (buffer[i] == '\n' || buffer[i] == '\r') return true;  // Akzeptiere beide
        i = (i + 1) % BufferSize;
    }
    return false;
}

bool UartRingBuffer::readLine(std::string& outLine) {
    if (!hasLine()) return false;

    outLine.clear();
    while (tail != head) {
        char c = buffer[tail];
        tail = (tail + 1) % BufferSize;
        if (c == '\n' || c == '\r') break;  // Beide als Zeilenende
        outLine += c;   // Alles andere übernehmen (kein zusätzliches Filter auf \r nötig)
    }
    return true;
}

void UartRingBuffer::clear() {
    head = tail = 0;
}
