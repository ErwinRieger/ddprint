/*
* This file is part of ddprint - a 3D printer firmware.
* 
* Copyright 2020 erwin.rieger@ibrieger.de
* 
* ddprint is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* ddprint is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with ddprint.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "mdebug.h"

#pragma once

template <class ElementType, class IndexType, int BufferSize>
struct CircularBuffer {

    typedef IndexType _IndexType;

    ElementType _ringbuffer_array[BufferSize];

    IndexType _ringbuffer_head;
    IndexType _ringbuffer_tail;

    CircularBuffer() { ringBufferInit(); }

    void ringBufferInit() { _ringbuffer_head = _ringbuffer_tail = 0; }

    IndexType mask(IndexType val)  { return val & (BufferSize - 1); }

    bool empty()    { return _ringbuffer_head == _ringbuffer_tail; }
    bool full()     { return size() == (BufferSize - 1); }
    IndexType size()     { return _ringbuffer_head - _ringbuffer_tail; }

    void pushRef(ElementType& val)  {
        _ringbuffer_array[mask(_ringbuffer_head++)] = val;
    }

    void pushVal(ElementType val)  {
        _ringbuffer_array[mask(_ringbuffer_head++)] = val;
    }

    ElementType &pop() {
        return _ringbuffer_array[mask(_ringbuffer_tail++)];
    }
    ElementType &peek() { return _ringbuffer_array[mask(_ringbuffer_tail)]; }
    ElementType &peekN(IndexType index) { return _ringbuffer_array[mask(_ringbuffer_tail+index)]; }
};

template <class ElementType>
struct Buffer256 {

    ElementType _ringbuffer_array[256];
    uint8_t _ringbuffer_head;
    uint8_t _ringbuffer_tail;

    Buffer256() { ringBufferInit(); }
    FWINLINE void ringBufferInit() { _ringbuffer_head = _ringbuffer_tail = 0; }

    FWINLINE bool empty()   { return _ringbuffer_head == _ringbuffer_tail; }
    FWINLINE bool full()    { return size() == 255; }
    FWINLINE uint8_t size() { return _ringbuffer_head - _ringbuffer_tail; }

    FWINLINE void pushRef(ElementType &val)  {
       _ringbuffer_array[_ringbuffer_head++] = val;
    }

    FWINLINE void pushVal(ElementType val)  {
       _ringbuffer_array[_ringbuffer_head++] = val;
    }

    FWINLINE ElementType &pop() {
       return _ringbuffer_array[_ringbuffer_tail++];
    }

    // For uart ringbuffer
    FWINLINE ElementType &peek() { return _ringbuffer_array[_ringbuffer_tail]; }
    // For uart ringbuffer
    FWINLINE ElementType &peekN(uint8_t index) { return _ringbuffer_array[_ringbuffer_tail+index]; }

    // For uart ringbuffer/cobs
    FWINLINE void setVal(uint8_t i, ElementType val) {
        _ringbuffer_array[i] = val;
    }

    // For uart ringbuffer/cobs
    FWINLINE void inc(uint8_t i) {
        _ringbuffer_array[i] ++;
    }
};

















































