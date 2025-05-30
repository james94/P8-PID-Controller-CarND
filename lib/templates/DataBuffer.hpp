#pragma once

template <typename T>
class DataBuffer {
    std::vector<T> buffer;
public:
    void add(const T& data) {
        buffer.push_back(data);
    }

    T getLast() const {
        return buffer.back();
    }
};
