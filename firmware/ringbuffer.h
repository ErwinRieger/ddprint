// xxx header

#pragma once

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
        
        IndexType h = _ringbuffer_head;
        _ringbuffer_array[mask(h)] = val;
        _ringbuffer_head = h+1;
    }

    void pushVar(ElementType val)  {
        
        IndexType h = _ringbuffer_head;
        _ringbuffer_array[mask(h)] = val;
        _ringbuffer_head = h+1;
    }

    ElementType &pop() {

        IndexType t = _ringbuffer_tail;
        ElementType &val = _ringbuffer_array[mask(t)];
        _ringbuffer_tail = t + 1;
        return val;
    }
    ElementType &peek() { return _ringbuffer_array[mask(_ringbuffer_tail)]; }
    ElementType &peekN(IndexType index) { return _ringbuffer_array[mask(_ringbuffer_tail+index)]; }
};

template <class ElementType, class IndexType, int BufferSize>
struct oldCircularBuffer {

    typedef IndexType _IndexType;

    ElementType array[BufferSize];
    volatile IndexType head;
    volatile IndexType tail;

    void ringBufferInit() { head = tail = 0; }

    IndexType mask(IndexType val)  { return val & (BufferSize - 1); }

    bool empty()    { return head == tail; }
    bool full()     { return size() == BufferSize; }
    IndexType size()     { return head - tail; }

    void push(ElementType& val)  {
        
        IndexType h = head;
        array[mask(h)] = val;
        head = h+1;
    }

    void pushVar(ElementType val)  {
        
        IndexType h = head;
        array[mask(h)] = val;
        head = h+1;
    }

    ElementType &pop() {

        IndexType t = tail;
        ElementType &val = array[mask(t)];
        tail = t + 1;
        return val;
    }
    ElementType &peek() { return array[mask(tail)]; }
    ElementType &peekN(IndexType index) { return array[mask(tail+index)]; }
};

