#include "queue.h"

struct RelocPosition
{
    double x_, y_, z_;
    double timestamp_;

    RelocPosition() : x_(0.0), y_(0.0), z_(0.0), timestamp_(0.0) {}
    RelocPosition(double x, double y, double z, double timestamp) : x_(x), y_(y), z_(z), timestamp_(timestamp) {}
    RelocPosition(double x, double y, double z) : x_(x), y_(y), z_(z), timestamp_(0.0) {}
};

struct RelocOrientation
{
    double qx_, qy_, qz_, qw_;
    double timestamp_;

    RelocOrientation() : qx_(0.0), qy_(0.0), qz_(0.0), qw_(1.0), timestamp_(0.0) {}
    RelocOrientation(double qx, double qy, double qz, double qw, double timestamp) : qx_(qw), qy_(qy), qz_(qz), qw_(qw), timestamp_(timestamp) {}
    RelocOrientation(double qx, double qy, double qz, double qw) : qx_(qw), qy_(qy), qz_(qz), qw_(qw), timestamp_(0.0) {}
};

struct RelocState
{
    double x_, y_, z_;
    double qx_, qy_, qz_, qw_;
    double timestamp_;

    RelocState() : x_(0.0), y_(0.0), z_(0.0), 
            qx_(0.0), qy_(0.0), qz_(0.0), qw_(0.0), timestamp_(0.0) {}

    RelocState(const RelocPosition& position, const RelocOrientation& orientation, double timestamp)
                 : x_(position.x_), y_(position.y_), z_(position.z_), 
                    qx_(orientation.qx_), qy_(orientation.qy_), qz_(orientation.qz_), qw_(orientation.qw_), timestamp_(timestamp) {}
    
    RelocState(double x, double y, double z, 
                double qx, double qy, double qz, double qw, double timestamp)
                 : x_(x), y_(y), z_(z), 
                    qx_(qx), qy_(qy), qz_(qz), qw_(qw), timestamp_(timestamp) {}

};

class RelocStateBuffer
{
    public:
        RelocStateBuffer() : queue_(10) {}
        RelocStateBuffer(size_t capacity) : queue_(capacity) {}

        void Push(const RelocState &state)
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_full_.wait(lock, [this]() { return !queue_.IsFull(); });

            queue_.Push(state);

            cond_empty_.notify_one();
        }

        RelocState Pop()
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_empty_.wait(lock, [this]() { return !queue_.IsEmpty(); });

            auto state = queue_.Pop();

            cond_full_.notify_one();
            return state;
        }

        bool TryPop(RelocState &state)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.IsEmpty()) return false;
            state = queue_.Pop();
            cond_full_.notify_one();
            return true;
        }

        RelocState Front()
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cond_empty_.wait(lock, [this]() { return !queue_.IsEmpty(); });
            return queue_.Front();  
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
        Queue<RelocState> queue_; 
        mutable std::mutex mtx_;
        std::condition_variable cond_full_;
        std::condition_variable cond_empty_; 
};