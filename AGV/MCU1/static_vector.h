#pragma once
#include "types.h"
#include <Arduino.h>

template <typename T, size_t N>

class StaticVector
{
public:
    StaticVector() {}

    StaticVector(const StaticVector &v) : _count(v._count)
    {
        for (size_t i = 0; i < _count; i++)
            _v[i] = v._v[i];
    }

    void push_back(const T &data)
    {
        if (_count >= N)
            _raise_error("[FATAL ERROR] Array out of bounds");
        _v[_count++] = data;
    }

    void pop()
    {
        if (_count > 0)
            _count--;
    }

    void pop(size_t i)
    {
        if (_count <= 1)
        {
            pop();
            return;
        }

        for (i; i < _count - 1; i++)
            _v[i] = _v[i + 1];
        _count--;
    }

    T pop_front()
    {
        if (_count == 0)
            _raise_error("[FATAL ERROR] Array out of bounds");

        T tmp = _v[_count - 1];
        pop(0);
        return tmp;
    }

    template <typename Predicate>
    void pop_if(Predicate pred)
    {
        for (size_t i = 0; i < _count; i++)
            if (pred(_v[i]))
                pop(i);
    }

    void clear() { _count = 0; }

    template <typename Predicate>
    T *find_if(Predicate pred)
    {
        for (size_t i = 0; i < _count; i++)
            if (pred(_v[i]))
                return &_v[i];

        return nullptr;
    }

    template <typename Predicate>
    const T *find_if(Predicate pred) const
    {
        for (size_t i = 0; i < _count; i++)
            if (pred(_v[i]))
                return &_v[i];

        return nullptr;
    }

    // Packet *p = vec.find_if([](const Packet &p)
    //                         { return p.seq == 1; });

    // if (p)
    // {
    //     Serial.println(p->seq);
    // }

    size_t size() { return _count; }

private:
    size_t _count = 0;
    T _v[N];

    void _raise_error(const String &msg) const
    {
        Serial.println(msg);
        while (true)
        {
        };
    }
};