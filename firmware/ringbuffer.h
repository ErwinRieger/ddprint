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
#include "mdebug.h"

#pragma once

template <class ElementType, class IndexType, int BufferSize>
struct CircularBuffer {

    typedef IndexType _IndexType;

    ElementType _ringbuffer_array[BufferSize];
    IndexType _ringbuffer_head;
    IndexType _ringbuffer_tail;

    void ringBufferInit() { _ringbuffer_head = _ringbuffer_tail = 0; }

    IndexType mask(IndexType val)  { return val & (BufferSize - 1); }

    bool empty()    { return _ringbuffer_head == _ringbuffer_tail; }
    bool full()     { return size() == BufferSize; }
    IndexType size()     { return _ringbuffer_head - _ringbuffer_tail; }

    void push(ElementType& val)  {
       
       massert(!full());

        IndexType h = _ringbuffer_head;
        _ringbuffer_array[mask(h)] = val;
        _ringbuffer_head = h+1;
    }

    void pushVar(ElementType val)  {
        
       massert(!full());

        IndexType h = _ringbuffer_head;
        _ringbuffer_array[mask(h)] = val;
        _ringbuffer_head = h+1;
    }

    // 
    void pushWrap(ElementType& val)  {
        
        if (full())
            pop();
        push(val);
    }

    ElementType &pop() {

       massert(!empty());

        IndexType t = _ringbuffer_tail;
        ElementType &val = _ringbuffer_array[mask(t)];
        _ringbuffer_tail = t + 1;
        return val;
    }
    ElementType &peek() { massert(!empty()); return _ringbuffer_array[mask(_ringbuffer_tail)]; }
    ElementType &peekN(IndexType index) { return _ringbuffer_array[mask(_ringbuffer_tail+index)]; }

    void set(IndexType i, ElementType &val) {
        _ringbuffer_array[mask(i)] = val;
    }
    void setVar(IndexType i, ElementType val) {
        _ringbuffer_array[mask(i)] = val;
    }
    void mod(IndexType i, ElementType &val) {
        _ringbuffer_array[mask(i)] += val;
    }
    void inc(IndexType i) {
        _ringbuffer_array[mask(i)] ++;
    }
};

