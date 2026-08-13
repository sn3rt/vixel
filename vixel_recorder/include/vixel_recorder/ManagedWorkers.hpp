#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace vixel_recorder
{

class ManagedWorkers
{
public:
  ManagedWorkers() = default;
  ManagedWorkers(const ManagedWorkers &) = delete;
  ManagedWorkers & operator=(const ManagedWorkers &) = delete;

  ~ManagedWorkers()
  {
    stop_and_join();
  }

  bool start(std::function<void()> task)
  {
    auto done = std::make_shared<std::atomic_bool>(false);
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {return false;}
    workers_.push_back(Worker{
      done,
      std::thread([done, task = std::move(task)]() mutable {
        try {
          task();
        } catch (...) {
          // Callers wrap tasks to report failures. The registry's final guard
          // still prevents an exception from terminating the whole process.
        }
        done->store(true, std::memory_order_release);
      })
    });
    return true;
  }

  void reap()
  {
    std::vector<std::thread> completed;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto iterator = workers_.begin(); iterator != workers_.end();) {
        if (iterator->done->load(std::memory_order_acquire)) {
          completed.push_back(std::move(iterator->thread));
          iterator = workers_.erase(iterator);
        } else {
          ++iterator;
        }
      }
    }
    for (auto & worker : completed) {
      if (worker.joinable()) {worker.join();}
    }
  }

  void request_stop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }

  void stop_and_join()
  {
    std::vector<std::thread> workers;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      workers.reserve(workers_.size());
      for (auto & worker : workers_) {workers.push_back(std::move(worker.thread));}
      workers_.clear();
    }
    for (auto & worker : workers) {
      if (worker.joinable()) {worker.join();}
    }
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return workers_.size();
  }

private:
  struct Worker
  {
    std::shared_ptr<std::atomic_bool> done;
    std::thread thread;
  };

  mutable std::mutex mutex_;
  bool stopping_{false};
  std::vector<Worker> workers_;
};

}  // namespace vixel_recorder
