//-- includes -----
#include "ClientNetworkManager.h"
#include "packedmessage.h"
#include "PSMoveDataFrame.pb.h"
#include <cassert>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <deque>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/cstdint.hpp>

//-- pre-declarations -----
using namespace std;
namespace asio = boost::asio;
using asio::ip::tcp;
using asio::ip::udp;
using boost::uint8_t;

//-- constants -----
#define DEBUG true

//-- implementation -----

// -ClientNetworkManagerImpl-
// Internal implementation of the client network manager.
class ClientNetworkManagerImpl
{
public:
    ClientNetworkManagerImpl(
        const std::string &host, 
        const std::string &port, 
        IDataFrameListener *dataFrameListener,
        INotificationListener *notificationListener,
        IResponseListener *responseListener,
        IClientNetworkEventListener *netEventListener)
        : m_server_host(host)
        , m_server_port(port)

        , m_io_service()
        , m_tcp_socket(m_io_service)
        , m_tcp_connection_id(-1)
        , m_udp_socket(m_io_service, udp::endpoint(udp::v4(), 0))
        , m_udp_server_endpoint()
        , m_udp_remote_endpoint()
        , m_connection_stopped(false)
        , m_has_pending_tcp_read(false)
        , m_has_pending_tcp_write(false)
        , m_has_pending_udp_read(false)

        , m_response_read_buffer()
        , m_packed_response(boost::shared_ptr<PSMoveDataFrame::Response>(new PSMoveDataFrame::Response()))

        , m_data_frame_read_buffer()
        , m_packed_data_frame(boost::shared_ptr<PSMoveDataFrame::ControllerDataFrame>(new PSMoveDataFrame::ControllerDataFrame()))
    
        , m_write_bufer()
        , m_packed_request()

        , m_data_frame_listener(dataFrameListener)
        , m_notification_listener(notificationListener)
        , m_response_listener(responseListener)
        , m_netEventListener(netEventListener)
        , m_pending_requests()
    {
    }

    bool start()
    {
        tcp::resolver resolver(m_io_service);
        tcp::resolver::iterator endpoint_iter= resolver.resolve(tcp::resolver::query(tcp::v4(), m_server_host, m_server_port));

        m_connection_stopped= false;
        bool success= start_tcp_connect(endpoint_iter);

        return success;
    }

    void send_request(RequestPtr request)
    {
        m_pending_requests.push_back(request);
        start_tcp_write_request();
    }

    void poll()
    {
        m_io_service.poll();
    }

    void stop()
    {
        // drain any pending requests
        while (m_pending_requests.size() > 0)
        {
            if (m_response_listener)
            {
                m_response_listener->handle_request_canceled(m_pending_requests.front());
            }

            m_pending_requests.pop_front();
        }

        // close the tcp request socket
        if (m_tcp_socket.is_open())
        {
            m_tcp_socket.shutdown(asio::socket_base::shutdown_both);

            boost::system::error_code error;
            m_tcp_socket.close(error);

            if (error)
            {
                DEBUG && (cerr << "Problem closing the socket: " << error.value() << endl);
                if (m_netEventListener)
                {
                    m_netEventListener->handle_server_connection_close_failed(error);
                }
            }
            else
            {
                if (m_netEventListener)
                {
                    m_netEventListener->handle_server_connection_closed();
                }
            }
        }

        m_connection_stopped= true;
        m_has_pending_tcp_read= false;
        m_has_pending_tcp_write= false;
    }

private:
    bool start_tcp_connect(tcp::resolver::iterator endpoint_iter)
    {
        bool success= true;

        if (endpoint_iter != tcp::resolver::iterator())
        {
            DEBUG && (cout << "Trying " << endpoint_iter->endpoint() << "..." << endl);

            // Start the asynchronous connect operation.
            m_tcp_socket.async_connect(
                endpoint_iter->endpoint(),
                boost::bind(&ClientNetworkManagerImpl::handle_tcp_connect, this, _1, endpoint_iter));
        }
        else
        {
            // There are no more endpoints to try. Shut down the client.
            stop();
            success= false;

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_open_failed(boost::asio::error::host_unreachable);
            }
        }

        return success;
    }

    void handle_tcp_connect(
        const boost::system::error_code& ec,
        tcp::resolver::iterator endpoint_iter)
    {
        if (m_connection_stopped)
            return;

        // The async_connect() function automatically opens the socket at the start
        // of the asynchronous operation. If the socket is closed at this time then
        // the timeout handler must have run first.
        if (!m_tcp_socket.is_open())
        {
            DEBUG && (std::cerr << "TCP Connect timed out" << endl);

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_open_failed(boost::asio::error::timed_out);
            }

            // Try the next available endpoint.
            start_tcp_connect(++endpoint_iter);
        }
        // Check if the connect operation failed before the deadline expired.
        else if (ec)
        {
            DEBUG && (std::cerr << "TCP Connect error: " << ec.message() << endl);

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_open_failed(ec);
            }

            // We need to close the socket used in the previous connection attempt
            // before starting a new one.
            m_tcp_socket.close();

            // Try the next available endpoint.
            start_tcp_connect(++endpoint_iter);
        }
        // Otherwise we have successfully established a connection.
        else
        {
            tcp::endpoint tcp_endpoint= endpoint_iter->endpoint();

            DEBUG && (std::cout << "Connected to " << tcp_endpoint << endl);

            // Create a corresponding endpoint udp data will be sent to
            m_udp_server_endpoint= udp::endpoint(tcp_endpoint.address(), tcp_endpoint.port());

            // Start listening for any incoming responses (TCP messages)
            // NOTE: Responses that come independent of a request are a "notification"
            start_tcp_read_response_header();
        }
    }

    void handle_tcp_connection_info_notification(ResponsePtr notification)
    {
        assert(notification->type() == PSMoveDataFrame::Response_ResponseType_CONNECTION_INFO);

        // Remember the connection id
        m_tcp_connection_id= notification->result_connection_info().tcp_connection_id();

        // Send the connection id back to the server over UDP
        // to establish a UDP connected and associate it with the TCP connection
        send_udp_connection_id();
    }

    void send_udp_connection_id()
    {
        DEBUG && (cerr << "Sending connection id to server over UDP: " << m_tcp_connection_id << '\n');
        m_udp_socket.async_send_to(
            boost::asio::buffer(&m_tcp_connection_id, sizeof(m_tcp_connection_id)), 
            m_udp_server_endpoint,
            boost::bind(&ClientNetworkManagerImpl::handle_udp_write_connection_id, this, boost::asio::placeholders::error));
    }

    void handle_udp_write_connection_id(const boost::system::error_code& error)
    {
        if (!error) 
        {
            DEBUG && (cerr << "Successfully sent UDP connection id: " << error.message() << '\n');

            // Now wait for the response
            //###bwalker $TODO timeout
            m_udp_socket.async_receive_from(
                boost::asio::buffer(&m_udp_connection_result_read_buffer, sizeof(m_udp_connection_result_read_buffer)),
                m_udp_remote_endpoint,
                boost::bind(&ClientNetworkManagerImpl::handle_udp_read_connection_result, this, boost::asio::placeholders::error));
        }
        else
        {
            DEBUG && (cerr << "Failed to send UDP connection id: " << error.message() << '\n');

            // Try again...
            //###bwalker $TODO timeout after a given number of attempts
            send_udp_connection_id();
        }
    }

    void handle_udp_read_connection_result(const boost::system::error_code& error)
    {
        if (error)
        {
            DEBUG && (std::cerr << "UDP Connect error: " << error.message() << endl);

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_open_failed(error);
            }
        }
        else if (m_udp_connection_result_read_buffer == false)
        {
            DEBUG && (std::cerr << "UDP Connect error: Invalid connection id " << endl);

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_open_failed(boost::system::error_code());
            }
        }
        else
        {
            DEBUG && (std::cout << "UDP Connect Success!" << endl);

            // Start listening for any incoming data frames (UDP messages)
            start_udp_read_data_frame();

            // If there are any requests waiting, send them off
            start_tcp_write_request();

            // Tell the network event listener that we are finally all connected
            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_opened();
            }
        }
    }

    void start_tcp_read_response_header()
    {
        if (!m_has_pending_tcp_read)
        {

            m_has_pending_tcp_read= true;
            m_response_read_buffer.resize(HEADER_SIZE);
            asio::async_read(
                m_tcp_socket, 
                asio::buffer(m_response_read_buffer),
                boost::bind(
                    &ClientNetworkManagerImpl::handle_tcp_read_response_header, 
                    this,
                    asio::placeholders::error));
        }
    }

    void handle_tcp_read_response_header(const boost::system::error_code& error)
    {
        if (m_connection_stopped)
            return;

        if (!error) 
        {
            DEBUG && (cout << "handle_tcp_read_response_header() - Received Message Header:\n");
            DEBUG && (cout << show_hex(m_response_read_buffer) << endl);
            unsigned msg_len = m_packed_response.decode_header(m_response_read_buffer);
            DEBUG && (cout << msg_len << " bytes" << endl);

            if (msg_len > 0)
            {
                start_tcp_read_response_body(msg_len);
            }
            else
            {
                // If there is no body, jump straight to the handle ready response body callback
                handle_tcp_read_response_body(boost::system::error_code());
            }
        }
        else
        {
            DEBUG && (std::cerr << "handle_tcp_read_response_header() - Error on receive: " << error.message() << endl);
            stop();

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(error);
            }
        }
    }

    void start_tcp_read_response_body(unsigned msg_len)
    {
        assert(m_has_pending_tcp_read);
        assert(msg_len > 0);

        // m_read_buffer already contains the header in its first HEADER_SIZE bytes. 
        // Expand it to fit in the body as well, and start async read into the body.
        m_response_read_buffer.resize(HEADER_SIZE + msg_len);
        asio::mutable_buffers_1 buffer = asio::buffer(&m_response_read_buffer[HEADER_SIZE], msg_len);
        asio::async_read(
            m_tcp_socket, 
            buffer,
            boost::bind(
                &ClientNetworkManagerImpl::handle_tcp_read_response_body, 
                this,
                asio::placeholders::error));
    }

    void handle_tcp_read_response_body(const boost::system::error_code& error)
    {
        if (m_connection_stopped)
            return;

        if (!error) 
        {
            DEBUG && (cout << "handle_tcp_read_response_body() - Received Response Body:\n");
            DEBUG && (cout << show_hex(m_response_read_buffer) << endl);

            // Process the response now that we have received all of it
            handle_tcp_response_received();

            // Start reading the next incoming response
            start_tcp_read_response_header();
        }
        else
        {
            DEBUG && (cerr << "handle_tcp_read_response_body() - Error on receive: " << error.message() << endl);
            stop();

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(error);
            }
        }
    }

    // Called when enough data was read into m_readbuf for a complete response message. 
    // Parse the response and forward it on to the response handler.
    void handle_tcp_response_received()
    {
        // No longer is there a pending read
        m_has_pending_tcp_read= false;

        // Parse the response buffer
        if (m_packed_response.unpack(m_response_read_buffer))
        {
            ResponsePtr response = m_packed_response.get_msg();

            if (response->request_id() != -1)
            {
                m_response_listener->handle_response(response);
            }
            else
            {                
                if (response->type() == PSMoveDataFrame::Response_ResponseType_CONNECTION_INFO)
                {
                    // SPECIAL CASE: ConnectionInfo response sent at
                    handle_tcp_connection_info_notification(response);
                }
                else
                {
                    // Responses without a request ID are notifications
                    m_notification_listener->handle_notification(response);
                }
            }
        }
        else
        {
            DEBUG && (cerr << "handle_tcp_response_received() - Error malformed response" << endl);
            stop();

            if (m_netEventListener)
            {
                //###bwalker $TODO pick a better error code that means "malformed data"
                m_netEventListener->handle_server_connection_socket_error(boost::asio::error::message_size);
            }
        }
    }

    void start_tcp_write_request()
    {        
        if (m_connection_stopped)
            return;

        if (m_pending_requests.size() > 0 && !m_has_pending_tcp_write)
        {
            RequestPtr request= m_pending_requests.front();

            m_packed_request.set_msg(request);
            m_packed_request.pack(m_write_bufer);

            // The queue should prevent us from writing more than one request as once
            assert(!m_has_pending_tcp_write);
            m_has_pending_tcp_write= true;

            // Start an asynchronous operation to send a heartbeat message.
            boost::asio::async_write(
                m_tcp_socket, 
                boost::asio::buffer(m_write_bufer),
                boost::bind(&ClientNetworkManagerImpl::handle_tcp_write_request_complete, this, _1));
        }
    }

    void handle_tcp_write_request_complete(const boost::system::error_code& ec)
    {
        if (m_connection_stopped)
            return;

        if (!ec)
        {
            // no longer is there a pending write
            m_has_pending_tcp_write= false;

            // Remove the request from the pending send queue not that it's sent
            m_pending_requests.pop_front();

            // Start listening for the response
            start_tcp_read_response_header();

            // If there are more requests waiting to be sent, start sending the next one
            start_tcp_write_request();
        }
        else
        {
            DEBUG && (cerr << "handle_tcp_write_request_complete() - Error on request send: " << ec.message() << endl);
            stop();

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(ec);
            }
        }
    }

    void start_udp_read_data_frame()
    {
        if (!m_has_pending_udp_read)
        {
            m_has_pending_udp_read= true;
            m_udp_socket.async_receive_from(
                asio::buffer(m_data_frame_read_buffer, sizeof(m_data_frame_read_buffer)),
                m_udp_server_endpoint,
                boost::bind(
                    &ClientNetworkManagerImpl::handle_udp_read_data_frame, 
                    this,
                    asio::placeholders::error));
        }
    }

    void handle_udp_read_data_frame(const boost::system::error_code& error)
    {
        if (m_connection_stopped)
            return;

        if (!error)
        {
            DEBUG && (cout << "handle_udp_read_data_frame_header() - Received DataFrame" << endl);

            // Process the data frame now that we have received all of it
            handle_udp_data_frame_received();

            // Start reading the next incoming data frame
            start_udp_read_data_frame();
        }
        else
        {
            DEBUG && (std::cerr << "handle_udp_read_data_frame_header() - Error on receive: " << error.message() << endl);
            stop();

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(error);
            }
        }
    }

    // Called when enough data was read into m_data_frame_read_buffer for a complete data frame message. 
    // Parse the data_frame and forward it on to the response handler.
    void handle_udp_data_frame_received()
    {
        // No longer is there a pending read
        m_has_pending_udp_read= false;

        DEBUG && (cout << "handle_udp_data_frame_received() - Parsing DataFrame" << endl);
        unsigned msg_len = m_packed_data_frame.decode_header(m_data_frame_read_buffer, sizeof(m_data_frame_read_buffer));
        unsigned total_len= HEADER_SIZE+msg_len;
        DEBUG && (cout << show_hex(m_data_frame_read_buffer, total_len) << endl);
        DEBUG && (cout << msg_len << " bytes" << endl);

        // Parse the response buffer
        if (m_packed_data_frame.unpack(m_data_frame_read_buffer, total_len))
        {
            ControllerDataFramePtr data_frame = m_packed_data_frame.get_msg();

            m_data_frame_listener->handle_data_frame(data_frame);
        }
        else
        {
            DEBUG && (cerr << "handle_udp_data_frame_received() - Error malformed response" << endl);
            stop();

            if (m_netEventListener)
            {
                //###bwalker $TODO pick a better error code that means "malformed data"
                m_netEventListener->handle_server_connection_socket_error(boost::asio::error::message_size);
            }
        }
    }

private:
    const std::string &m_server_host;
    const std::string &m_server_port;

    asio::io_service m_io_service;
    tcp::socket m_tcp_socket;
    int m_tcp_connection_id;

    udp::socket m_udp_socket;
    udp::endpoint m_udp_server_endpoint;
    udp::endpoint m_udp_remote_endpoint;
    bool m_udp_connection_result_read_buffer;

    bool m_connection_stopped;
    bool m_has_pending_tcp_read;
    bool m_has_pending_tcp_write;
    bool m_has_pending_udp_read;
    
    vector<uint8_t> m_response_read_buffer;
    PackedMessage<PSMoveDataFrame::Response> m_packed_response;

    uint8_t m_data_frame_read_buffer[HEADER_SIZE+MAX_DATA_FRAME_MESSAGE_SIZE];
    PackedMessage<PSMoveDataFrame::ControllerDataFrame> m_packed_data_frame;
    
    vector<uint8_t> m_write_bufer;
    PackedMessage<PSMoveDataFrame::Request> m_packed_request;

    IDataFrameListener *m_data_frame_listener;
    INotificationListener *m_notification_listener;
    IResponseListener *m_response_listener;
    IClientNetworkEventListener *m_netEventListener;

    deque<RequestPtr> m_pending_requests;
};

// -ClientNetworkManager-
// Public interface to the psmove client network API
ClientNetworkManager *ClientNetworkManager::m_instance = NULL;

ClientNetworkManager::ClientNetworkManager(
    const std::string &host, 
    const std::string &port, 
    IDataFrameListener *dataFrameListener,
    INotificationListener *notificationListener,
    IResponseListener *responseListener,
    IClientNetworkEventListener *netEventListener)
    : m_implementation_ptr(
        new ClientNetworkManagerImpl(
            host, 
            port, 
            dataFrameListener,
            notificationListener,
            responseListener,
            netEventListener))
{
}

ClientNetworkManager::~ClientNetworkManager()
{
    assert(m_instance == NULL);
    delete m_implementation_ptr;
}

bool ClientNetworkManager::startup()
{
    m_instance= this;

    return m_implementation_ptr->start();
}

void ClientNetworkManager::send_request(RequestPtr request)
{
    m_implementation_ptr->send_request(request);
}

void ClientNetworkManager::update()
{
    m_implementation_ptr->poll();
}

void ClientNetworkManager::shutdown()
{
    m_implementation_ptr->stop();
    m_instance = NULL;
}