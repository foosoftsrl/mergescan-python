#include "ThreadPool.h"

ThreadPool::ThreadPool(size_t threads) : numThreads(threads), stop(false)
{
    for (size_t i = 0; i < threads; ++i)
        workers.emplace_back(
            [this]
            {
                for (;;)
                {
                    QueueItem task;
                    {
                        std::unique_lock<std::mutex> lock(this->queue_mutex);
                        this->condition.wait(lock,
                            [this] { return this->stop || !this->tasks.empty(); });
                        if (this->stop && this->tasks.empty())
                            return;
                        task = std::move(this->tasks.top());
                        this->tasks.pop();
                        processedJobCondition.notify_all();
                    }
                    task.func();
                }
            }
            );
}

ThreadPool::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(queue_mutex);
        stop = true;
    }
    condition.notify_all();
    for (std::thread& worker : workers)
        worker.join();
}

void ThreadPool::checkForStartableJobs() {
    std::unique_lock<std::mutex> lock(queue_mutex);
    auto predicate = [](const QueueItem& q) {
        return q.state->dependencyCount == 0;
    };
    auto first = std::find_if(delayedTasks.begin(), delayedTasks.end(), predicate);
    if (first != delayedTasks.end()) {
        tasks.emplace(std::move(*first));
        condition.notify_one();
        for (auto i = first; ++i != delayedTasks.end(); ) {
            if (predicate(*i)) {
                tasks.emplace(std::move(*i));
                condition.notify_one();
            }
            else {
                *first++ = std::move(*i);
            }
        }
    }
    delayedTasks.erase(first, delayedTasks.end());
}

// add new work item to the pool
void ThreadPool::enqueue(QueueItem&& queueItem)
{
    std::unique_lock<std::mutex> lock(queue_mutex);
    if (stop)
        throw std::runtime_error("enqueue on stopped ThreadPool");
    if (queueItem.state->dependencyCount) {
        delayedTasks.emplace_back(queueItem);
    }
    else {
        // limit the queue in order not to waste too much RAM
        while(tasks.size() > numThreads * 2) {
            processedJobCondition.wait(lock);
        }
        tasks.emplace(queueItem);
        condition.notify_one();
    }
}
