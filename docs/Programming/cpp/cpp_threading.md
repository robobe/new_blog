---
tags:
    - cpp
    - threading
---

# CPP Threading

## Hello Demo

```cpp
#include <iostream>
#include <thread>

void threadFunction(int threadNumber) {
    std::cout << "Thread " << threadNumber << " is running\n";
}

int main() {
    // thread requires a callable object
    std::thread t1(threadFunction, 1);
    std::thread t2(threadFunction, 2);

    // Joining the threads with the main thread
    t1.join();
    t2.join();

    std::cout << "Both threads have completed their execution\n";

    return 0;
}
```

## Mutex, Race Conditiion and Critical Section

```cpp
#include<mutex>
#include<thread>
#include <iostream>>

using namespace std;

int amount=0;

std::mutex m;

void addAmount(){
    m.lock();
    amount++;//Critical Section
    m.unlock();
}

int main(){
    std::thread t1(addAmount);
    std::thread t2(addAmount);
    t1.join();
    t2.join();
    cout << amount << endl;
    return 0;
}
```

---

## Thread Synchronization

!!! note "logging"
    Using spdlog for logging

    ```bash title="install"
    sudo apt update && sudo apt install libspdlog-dev
    ```

    ```cpp title="usage"
    #include <spdlog/spdlog.h>

    int main()
    {
        spdlog::info("Hello, this is an info message!");
        spdlog::warn("Warning! Something might be wrong.");
        spdlog::error("An error occurred!");

        return 0;
    }
    ```

    ```cmake
    find_package(spdlog REQUIRED)
    ...
    target_link_libraries(<target> PRIVATE spdlog::spdlog)

    ```

     
**std::condition_variable** is used for thread synchronization. It allows one or more threads to wait until a condition is met and another thread notifies them.

- std::unique_lock<std::mutex> lock(mtx); → Locks the mutex so no other thread can modify ready.
- cv.wait(lock, [] { return ready; }); →
    - Puts the thread to sleep if ready == false.
    - Releases the mutex while waiting (so other threads can change ready).
    - If another thread calls cv.notify_one() or cv.notify_all(), the waiting thread wakes up.
    - Before resuming execution, it relocks the mutex.
- The consumer thread continues execution only when ready == true.

```cpp title="syc producer/consumer"
#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

std::mutex mtx;
std::condition_variable cv;
bool ready = false;  // Shared flag

void producer()
{
    std::unique_lock<std::mutex> lock(mtx);
    spdlog::info("Producer: wait 3 sec before notify...");
    std::this_thread::sleep_for(std::chrono::seconds(3));

    ready = true; // Update the shared flag
    spdlog::info("Producer: notify consumer");

    cv.notify_one(); // Notify consumer
}

void consumer()
{
    std::unique_lock<std::mutex> lock(mtx);
    spdlog::info("Consumer: waiting for notification...");

    // Wait until `ready` is true
    cv.wait(lock, [] { return ready; });

    spdlog::info("Consumer: received notification!");
}

int main()
{
    std::thread prod1(producer);
    std::thread cons1(consumer);
    cons1.join();
    prod1.join();
    

    return 0;
}

```

---

## Thread Synchronization with timeout

```cpp title="synchronization with timeout"
#include <iostream>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <spdlog/spdlog.h>

std::condition_variable cv;
std::mutex mtx;
bool ready = false;


void producer(int period) {
    std::this_thread::sleep_for(std::chrono::seconds(period));  // Simulate work
    std::unique_lock<std::mutex> lock(mtx);
    spdlog::info("Producer: notify consumer");
    ready = true;
    cv.notify_one();
}

void consumer(int period) {
    std::unique_lock<std::mutex> lock(mtx);
    spdlog::info("Consumer: waiting for notification...");

    if (cv.wait_for(lock, std::chrono::seconds(period), [] { return ready; })) {
        spdlog::info("Received notification!");
    } else {
        spdlog::error("Timeout occurred!");
    }
}

int main() {
    std::thread t1(producer, 3);
    std::thread t2(consumer, 5);

    t1.join();
    t2.join();
    return 0;
}
```

```bash
# notify:3 timeout: 5
[2025-02-10 21:26:02.795] [info] Consumer: waiting for notification...
[2025-02-10 21:26:05.795] [info] Producer: notify consumer
[2025-02-10 21:26:05.795] [info] Received notification!

# notify:5 timeout: 3
[2025-02-10 21:28:57.871] [info] Consumer: waiting for notification...
[2025-02-10 21:29:00.871] [error] Timeout occurred!
[2025-02-10 21:29:02.871] [info] Producer: notify consumer
```