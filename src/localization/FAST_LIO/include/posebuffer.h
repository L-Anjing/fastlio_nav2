#ifndef BUFFER_H_
#define BUFFER_H_

#include "queue.h"
#include <mutex>
#include <condition_variable>

struct Pose
{
    double _x, _y, _z;
    double _qx, _qy, _qz, _qw;
    double _timestamp;

    Pose(double x, double y, double z, double qx, double qy, double qz, double qw, double timestamp)
        : _x(x), _y(y), _z(z), _qx(qx), _qy(qy), _qz(qz), _qw(qw), _timestamp(timestamp) {}

    Pose() : _timestamp(0.0), _x(0.0), _y(0.0), _z(0.0), _qx(0.0), _qy(0.0), _qz(0.0), _qw(0.0) {}
};

class PoseBuffer
{
    public:
        PoseBuffer(size_t capacity = 400) : queue_(capacity) {}

        void Push(const Pose &pose)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_full_.wait(lock, [this]() { return !queue_.IsFull(); });

            queue_.Push(pose);

            cond_empty_.notify_one();
        }

        Pose Pop()
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_empty_.wait(lock, [this]() { return !queue_.IsEmpty(); });

            Pose pose = queue_.Pop();

            cond_full_.notify_one();
            return pose;
        }

        bool TryPop(Pose &pose)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.IsEmpty()) return false;
            pose = queue_.Pop();
            cond_full_.notify_one();
            return true;
        }

        bool IsEmpty() const
        {
            std::lock_guard<std::mutex> lock(mtx_);
            return queue_.IsEmpty();
        }

        int Size() const
        {
            std::lock_guard<std::mutex> lock(mtx_);
            return queue_.Size();
        }

    private:
        Queue<Pose> queue_;   
        mutable std::mutex mtx_;
        std::condition_variable cond_full_;
        std::condition_variable cond_empty_;

};

#endif