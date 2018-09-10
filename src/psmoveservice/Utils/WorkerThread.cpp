#include "WorkerThread.h"
#include "ServerUtility.h"
#include "ServerLog.h"

WorkerThread::WorkerThread(const std::string thread_name) 
	: m_threadName(thread_name)
	, m_exitSignaled({ false })
    , m_threadStarted(false)
	, m_workerThread()
{
}

void WorkerThread::startThread()
{
    if (!m_threadStarted)
    {
		m_exitSignaled= false;

        SERVER_LOG_INFO("WorkerThread::start") << "Starting worker thread: " << m_threadName;
		onThreadStarted();

        m_workerThread = std::thread(&WorkerThread::threadFunc, this);
        m_threadStarted = true;
    }
}

void WorkerThread::stopThread()
{
    if (m_threadStarted)
    {
        if (!m_exitSignaled)
        {
            SERVER_LOG_INFO("WorkerThread::stop") << "Stopping worker thread: " << m_threadName;
			// Set the atomic exit flag
            m_exitSignaled.store(true);

			// Give the thread a chance to set any state in response to the exit flag getting set
			onThreadHaltBegin();

			// Block until the worker thread exists
            m_workerThread.join();

            SERVER_LOG_INFO("WorkerThread::stop") << "Worker thread stopped: " << m_threadName;
			onThreadHaltComplete();
        }
        else
        {
            SERVER_LOG_INFO("WorkerThread::stop") << "Worker thread already stopped: " << m_threadName;
        }

        m_threadStarted = false;
        m_exitSignaled = false;
    }
}


void WorkerThread::threadFunc()
{
    ServerUtility::set_current_thread_name(m_threadName.c_str());

    // Stay in the poll loop until asked to exit by the main thread
    while (!m_exitSignaled)
    {
		if (!doWork())
		{
			m_exitSignaled= true;
		}
    }
}