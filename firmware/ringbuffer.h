// xxx header

#include "mdebug.h"
#include "mdebug.h"

#pragma once

// static uint32_t stepin = 0;
// static uint32_t stepout = 0;

template <class ElementType, class IndexType, int BufferSize>
struct CircularBuffer {

    typedef IndexType _IndexType;

    ElementType _ringbuffer_array[BufferSize];
    volatile IndexType _ringbuffer_head;
    volatile IndexType _ringbuffer_tail;

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

