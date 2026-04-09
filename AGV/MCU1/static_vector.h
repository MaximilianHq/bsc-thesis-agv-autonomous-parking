#pragma once
#include "types.h"
#include <Arduino.h>

template <typename T>

class StaticVector
{
public:
    StaticVector(size_t arr_size) : _arr_size(arr_size) {}

    StaticVector(const StaticVector &v)
        : _arr_size(v._arr_size), _count(v._count)
    {
        for (size_t i = 0; i < _count; i++)
        {
            _v[i] = v._v[i];
        }
    }

    void push_back(T data)
    {
        if (_count >= _arr_size)
            _raise_error("[FATAL ERROR] Array out of bounds");
        _v[_count++] = data;
    }

    void pop() { _count-- };

    void pop(size_t i)
    {
        for (i; i < _count - 1; i++)
            _v[i] = _v[i + 1];
        _count--;
    }

    T pop_front()
    {
        T tmp = _v[_count];
        pop(0);
        return tmp;
    }

    size_t size() { return _count };

private:
    size_t _arr_size, _count = 0;
    T _v[_arr_size];

    void _raise_error(const String &msg) const
    {
        Serial.println(msg);
        while (true)
        {
        };
    }
}