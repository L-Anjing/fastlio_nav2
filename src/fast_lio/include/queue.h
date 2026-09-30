#ifndef QUEUE_H_
#define QUEUE_H_

#include <iostream>

#define MAXCAPACITY 10

template <class T> class Queue
{
    private:
        T *data_;
        int head_, tail_;
        int capacity_;
        int size_;

    public:
        Queue(int capacity):capacity_(capacity), size_(0), head_(0), tail_(0)
        {
            data_ = new T[capacity];
        }


        Queue():capacity_(MAXCAPACITY), size_(0), head_(0), tail_(0)
        {
            data_ = new T[capacity_];
        }

        ~Queue()
        {
            delete[] data_;
        }


        bool IsEmpty() const
        {
            if(size_ == 0) return true;
            else return false;
        }


        bool IsFull() const
        {
            if(size_ == capacity_) return true;
            else return false;
        }
        
        bool Push(const T &value)
        {
            if(IsFull()) 
            {
                throw("Queue is Full!");
                return false;
            }

            data_[tail_] = value;
            tail_ = (tail_ + 1) % capacity_;
            size_++;
            return true;
        }

        const T& Front() const
        {
            if(IsEmpty())
            {
                throw("Queue is Empty!");
            }
            return data_[head_];
        }

        T Pop()
        {
            if(IsEmpty()) 
            {
                throw("Queue is Empty!");
            }

            T value = data_[head_];
            head_ = (head_ + 1) % capacity_;
            size_--;
            return value;
        }

        int Size() const
        {
            return size_;
        }
        
};

#endif