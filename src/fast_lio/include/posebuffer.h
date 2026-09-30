#ifndef FAST_LIO_POSE_BUFFER_H
#define FAST_LIO_POSE_BUFFER_H

#include "queue.h"
#include <mutex>

struct ImuPoseSample
{
    double _x, _y, _z;
    double _qx, _qy, _qz, _qw;
    double _timestamp;

    ImuPoseSample(double x, double y, double z, double qx, double qy, double qz, double qw, double timestamp)
        : _x(x), _y(y), _z(z), _qx(qx), _qy(qy), _qz(qz), _qw(qw), _timestamp(timestamp) {}

    ImuPoseSample() : _x(0.0), _y(0.0), _z(0.0), _qx(0.0), _qy(0.0), _qz(0.0),
                      _qw(1.0), _timestamp(0.0) {}
};

class PoseBuffer
{
    public:
        PoseBuffer(size_t capacity = 400) : queue_(capacity) {}

        void Push(const ImuPoseSample &pose)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            if (queue_.IsFull()) {
                // Keep the newest poses to avoid blocking producer threads.
                queue_.Pop();
            }

            queue_.Push(pose);
        }

        bool TryPopLatest(ImuPoseSample &pose)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.IsEmpty()) return false;
            while (!queue_.IsEmpty()) {
                pose = queue_.Pop();
            }
            return true;
        }

        void Clear()
        {
            std::lock_guard<std::mutex> lock(mtx_);
            while (!queue_.IsEmpty()) {
                queue_.Pop();
            }
        }

    private:
        Queue<ImuPoseSample> queue_;
        std::mutex mtx_;
};

#endif
