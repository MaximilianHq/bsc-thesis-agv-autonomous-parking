#pragma once
#include <Arduino.h>
#include <types.h>
#include <initializer_list>

template <typename T, size_t N>

class StaticVector
{
public:
    StaticVector() {}

    StaticVector(std::initializer_list<T> list)
    {
        if (list.size() > N)
            _raise_error("[FATAL ERROR] StaticVector initializer list too large");

        for (const T &item : list)
            _v[_count++] = item;
    }

    StaticVector(const StaticVector &v) : _count(v._count)
    {
        for (size_t i = 0; i < _count; i++)
            _v[i] = v._v[i];
    }

    T &operator[](size_t i)
    {
        if (i >= _count)
            _raise_error("[FATAL ERROR] Index out of bounds");
        return _v[i];
    }

    const T &operator[](size_t i) const
    {
        if (i >= _count)
            _raise_error("[FATAL ERROR] Index out of bounds");
        return _v[i];
    }

    void push_back(const T &data)
    {
        if (_count >= N)
            pop(); // ta bort första/äldsta

        _v[_count++] = data; // lägg ny längst bak
    }

    void push_front(const T &data)
    {
        if (_count >= N)
            _count--;

        for (size_t i = _count; i > 0; i--)
            _v[i] = _v[i - 1];

        _v[0] = data;
        _count++;
    }

    void pop()
    {
        pop_front_discard();
    }

    void pop_back()
    {
        if (_count > 0)
            _count--;
    }

    void pop_front_discard()
    {
        if (_count == 0)
            return;

        for (size_t i = 0; i < _count - 1; i++)
            _v[i] = _v[i + 1];

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
            _raise_error("[FATAL ERROR] Array empty");

        T tmp = _v[0];
        pop_front_discard();
        return tmp;
    }

    template <typename Predicate>
    void pop_if(Predicate pred)
    {
        for (size_t i = 0; i < _count;)
        {
            if (pred(_v[i]))
                pop(i);
            else
                i++;
        }
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
