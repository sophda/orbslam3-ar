//
// Created by sophda on 2025/5/9.
//

#ifndef SAFEQUEUE_THREADSAFEQUEUE_H
#define SAFEQUEUE_THREADSAFEQUEUE_H
#include <iostream>
//#include <deque>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <queue>

template<class T>
class SafeQueue {

private:
    mutable std::mutex mutex_;
    std::queue<std::shared_ptr<T>> queue_;
    std::condition_variable cond_;

public:
    SafeQueue()=default;

    bool is_empty(){
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    void push(std::shared_ptr<T> item)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.push(item);
        cond_.notify_one();
    };

    std::shared_ptr<T> front() {
        std::unique_lock<std::mutex> lock(mutex_);
        return (queue_.front());
    }

    std::shared_ptr<T> back() {
        std::unique_lock<std::mutex> lock(mutex_);
        return queue_.back();
    }

    std::shared_ptr<T> wait_and_pop(){
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this](){return !queue_.empty();});
        std::shared_ptr<T > temp = queue_.front();
        queue_.pop();
        return temp;
    };

    void pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_.pop();
    }

};


#endif //SAFEQUEUE_THREADSAFEQUEUE_H
