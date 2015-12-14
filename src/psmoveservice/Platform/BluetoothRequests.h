#ifndef BLUETOOTH_REQUESTS_H
#define BLUETOOTH_REQUESTS_H

//-- includes -----
#include <memory>
#include <string>

//-- typedefs -----
class ServerControllerView;
typedef std::shared_ptr<ServerControllerView> ServerControllerViewPtr;

//-- definitions -----
class AsyncBluetoothRequest
{
public:
    enum eStatusCode
    {
        preflight,
        running,
        succeeded,
        failed
    };

    enum eCancelReason
    {
        timeout,
        connectionClosed
    };

    inline AsyncBluetoothRequest(int connectionId)
        : m_connectionId(connectionId)
        , m_status(preflight)
    { }

    virtual bool start()= 0;
    virtual void update()= 0;
    virtual void cancel(eCancelReason reason)= 0;

    virtual eStatusCode getStatusCode()= 0;
    virtual std::string getDescription()= 0;

protected:
    int m_connectionId;
    eStatusCode m_status;
};

class AsyncBluetoothUnpairDeviceRequest : public AsyncBluetoothRequest
{
public:
    inline AsyncBluetoothUnpairDeviceRequest(
        int connectionId,
        ServerControllerViewPtr controllerView)
        : AsyncBluetoothRequest(connectionId)
        , m_controllerView(controllerView)
    { }

    virtual bool start() override;
    virtual void update() override;
    virtual void cancel(AsyncBluetoothRequest::eCancelReason reason) override;

    virtual eStatusCode getStatusCode() override;
    virtual std::string getDescription() override;

private:
    ServerControllerViewPtr m_controllerView;
};

class AsyncBluetoothPairDeviceRequest : public AsyncBluetoothRequest
{
public:
    inline AsyncBluetoothPairDeviceRequest(
        int connectionId,
        ServerControllerViewPtr controllerView)
        : AsyncBluetoothRequest(connectionId)
        , m_controllerView(controllerView)
    { }

    virtual bool start() override;
    virtual void update() override;
    virtual void cancel(AsyncBluetoothRequest::eCancelReason reason) override;

    virtual eStatusCode getStatusCode() override;
    virtual std::string getDescription() override;

private:
    ServerControllerViewPtr m_controllerView;
};

#endif // BLUETOOTH_REQUESTS_H