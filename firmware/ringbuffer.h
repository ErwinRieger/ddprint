// xxx header

#pragma once

template <class ElementType, class IndexType, int BufferSize>
struct CircularBuffer {
    ElementType array[BufferSize];
    volatile IndexType head;
    volatile IndexType tail;

    void stepBufferInit() { head = tail = 0; }

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

