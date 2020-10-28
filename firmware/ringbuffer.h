// xxx header

#pragma once

template <class ElementType, class IndexType, int BufferSize>
struct CircularBuffer {
    ElementType array[BufferSize];
    volatile IndexType head;
    volatile IndexType tail;

    void init() { head = tail = 0; }

    IndexType mask(IndexType val)  { return val & (BufferSize - 1); }

    bool empty()    { return head == tail; }
    bool full()     { return size() == BufferSize; }
    IndexType size()     { return head - tail; }

    // Called from application
    void push(ElementType& val)  {
        
        IndexType h = head;
                // CRITICAL_SECTION_START;
        array[mask(h)] = val;
                // CRITICAL_SECTION_END;
        head = h+1;
    }

    // Called from stepper ISR
    ElementType &pop() {

        IndexType t = tail;
        ElementType &val = array[mask(t)];
        tail = t + 1;
        return val;
    }
    ElementType &peek() { return array[mask(tail)]; }
};

