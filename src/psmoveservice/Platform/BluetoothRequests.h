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
        userRequested,
        connectionClosed
    };

    inline AsyncBluetoothRequest(int connectionId)
        : m_connectionId(connectionId)
        , m_status(preflight)
    { }
    virtual ~AsyncBluetoothRequest()
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
    AsyncBluetoothUnpairDeviceRequest(
        int connectionId,
        ServerControllerViewPtr controllerView);
    virtual ~AsyncBluetoothUnpairDeviceRequest();

    virtual bool start() override;
    virtual void update() override;
    virtual void cancel(AsyncBluetoothRequest::eCancelReason reason) override;

    virtual eStatusCode getStatusCode() override;
    virtual std::string getDescription() override;

private:
    ServerControllerViewPtr m_controllerView;
    void *m_internal_state;
};

class AsyncBluetoothPairDeviceRequest : public AsyncBluetoothRequest
{
public:
    AsyncBluetoothPairDeviceRequest(
        int connectionId,
        ServerControllerViewPtr controllerView);
    virtual ~AsyncBluetoothPairDeviceRequest();

    virtual bool start() override;
    virtual void update() override;
    virtual void cancel(AsyncBluetoothRequest::eCancelReason reason) override;

    virtual eStatusCode getStatusCode() override;
    virtual std::string getDescription() override;

private:
    ServerControllerViewPtr m_controllerView;
    void *m_internal_state;
};

#endif // BLUETOOTH_REQUESTS_H