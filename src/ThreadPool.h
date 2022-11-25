#pragma once
#include <vector>
#include <queue>
#include <memory>
#include <thread>
#include <mutex>
#include <optional>
#include <condition_variable>
#include <functional>
#include <stdexcept>
#include <atomic>

/// @brief This class represents the non-template shared state of a future
struct CommonState {
    virtual ~CommonState() {

    }
    std::atomic<int> dependencyCount = 0;
    bool done = false;
    std::mutex mutex;
    std::condition_variable doneCondition;
    std::vector<std::shared_ptr<CommonState>> dependentTasks;
};

/// @brief A future (actually... it's more a promise), which can depend on other futures for execution
/// 
/// @tparam T the type to which this future evaluates
template <typename T>
struct JoinableFuture {

    struct State : public CommonState {
        T result;
    std::optional<std::string> exceptionMsg;
    };
    std::shared_ptr<State> state;

    /// @brief Constructor with no dependencies
    JoinableFuture() : state(new State()) {
    }

    /// @brief commodity constructor with dependencies
    /// @tparam ...Args 
    /// @param ...dependsOn 
    /// @return 
    template <typename... Args>
    static JoinableFuture withDepends(Args&... dependsOn) {
        JoinableFuture future;
        future.dependsOn(dependsOn...);
        return future;
    }

    void setValue(const T& t) {
        auto lock = std::unique_lock(state->mutex);
        state->result = t;
        state->done = true;
        state->doneCondition.notify_all();
        for (auto& dependendTask : state->dependentTasks) {
            dependendTask->dependencyCount--;
        }
        state->dependentTasks.clear();
    }
    
    void setFail(const std::exception& e) {
        auto lock = std::unique_lock(state->mutex);
        state->exceptionMsg = e.what();
        state->done = true;
        state->doneCondition.notify_all();
        for (auto& dependendTask : state->dependentTasks) {
            dependendTask->dependencyCount--;
        }
        state->dependentTasks.clear();
    }
    const T& get() const {
        auto lock = std::unique_lock(state->mutex);
        while (!state->done) {
            state->doneCondition.wait(lock);
        }
        if(state->exceptionMsg) {
            fprintf(stderr, "throwing exception in get!!! : %s\n", state->exceptionMsg->c_str());
            throw std::runtime_error(state->exceptionMsg->c_str());
        }
        return state->result;
    }
private:
    template <typename First, typename... Args>
    void dependsOn(First& first, Args&... rest) {
        dependsOnState(first.state);
        dependsOn(rest...); // recursive call using pack expansion syntax
    }

    void dependsOn() {
    }

    void dependsOnState(std::shared_ptr<CommonState> otherState) {
        auto lock = std::unique_lock(otherState->mutex);
        if (!otherState->done) {
            otherState->dependentTasks.push_back(state);
            state->dependencyCount++;
        }
    }
};


/// @brief A thread pool which can manage dependencies with task
/// 
/// Note that submissions order is respected 
class ThreadPool {
public:
    // the constructor just launches some amount of workers
    ThreadPool(size_t threads);

    // the destructor joins all threads
    ~ThreadPool();

    /// @brief submit a task with optional dependencies
#if 0
    template <class F, typename... Args>
    auto submit(F&& func, Args&... dependsOn) {
        using return_type = typename std::invoke_result<F>::type;
        auto future = JoinableFuture<return_type>::withDepends(dependsOn...);
        return submitToFuture(func, future);
    }
#else
    template <class F, typename... Args>
    auto submit(F&& func, Args&... dependsOn) {
        using return_type = typename std::invoke_result<F>::type;
        auto future = JoinableFuture<return_type>::withDepends(dependsOn...);
	future.setValue(func());
	return future;
    }
#endif

    
private:
    struct QueueItem {
        uint64_t order;
        std::function<void()> func;
        std::shared_ptr<CommonState> state;
        QueueItem() : QueueItem(0) {
        }

        QueueItem(uint64_t order) : order(order) {

        }
        friend bool operator <(const QueueItem& e1, const QueueItem& e2) {
            return e1.order > e2.order;
        }
    };

    template <class F>
    auto submitToFuture(F &&func, JoinableFuture<typename std::invoke_result<F>::type> joinableFuture)
    {
        using return_type = typename std::invoke_result<F>::type;
        QueueItem item(nextOrder++);
        item.func = ([this, joinableFuture, func]() {
            auto& joinableFuture_ = ((JoinableFuture<typename std::invoke_result<F>::type>&)joinableFuture);
            try {
                auto result = func();
                joinableFuture_.setValue(std::move(result));
            } catch(std::exception& e) {
                joinableFuture_.setFail(e);
            }
            checkForStartableJobs(); 
        });
        item.state = joinableFuture.state;
        enqueue(std::move(item));
        return joinableFuture;
    }

    // add new work item to the pool
    void enqueue(QueueItem&& queueItem);

    /// @brief verify if some jobs can be started
    void checkForStartableJobs();

    uint64_t nextOrder = 0;

    size_t numThreads;

    // need to keep track of threads so we can join them
    std::vector<std::thread> workers;

    // tasks which are ready to tasks
    std::priority_queue<QueueItem> tasks;

    // tasks which have some unsatisfied dependency
    std::vector<QueueItem> delayedTasks;

    // synchronization
    std::mutex queue_mutex;
    std::condition_variable condition;
    std::condition_variable processedJobCondition;
    bool stop;
};



