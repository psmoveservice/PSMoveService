#ifndef WORKER_THREAD_H
#define WORKER_THREAD_H

#include <atomic>
#include <string>
#include <thread>

class WorkerThread
{
public:
	WorkerThread(const std::string thread_name);

	inline bool hasThreadStarted() const
	{
		return m_threadStarted;
	}

	inline bool hasThreadEnded() const
	{
		return m_threadEnded.load();
	}

    void startThread();
    void stopThread();

protected:
	virtual void onThreadStarted() { }
	virtual void onThreadHaltBegin() { }
	virtual void onThreadHaltComplete() { }
	virtual bool doWork() = 0;

private:
	void threadFunc();

protected:
    // Multithreaded state
	const std::string m_threadName;
    std::atomic_bool m_exitSignaled;
	std::atomic_bool m_threadEnded;

	// Main Thread State
    bool m_threadStarted;
    std::thread m_workerThread;
};

#endif // WORKER_THREAD_H