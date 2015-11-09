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
#include <boost/enable_shared_from_this.hpp>

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
class ClientNetworkManagerImpl : public boost::enable_shared_from_this<ClientNetworkManagerImpl>
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
        , m_udp_socket(m_io_service, udp::endpoint(udp::v4(), 0))
        , m_udp_remote_endpoint()
        , m_connection_stopped(false)
        , m_has_pending_tcp_read(false)
        , m_has_pending_tcp_write(false)
        , m_has_pending_udp_read(false)

        , m_response_read_buffer()
        , m_packed_response()

        , m_data_frame_read_buffer()
        , m_packed_data_frame()
    
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
        tcp::resolver::iterator endpoint_iter= resolver.resolve(tcp::resolver::query(m_server_host, m_server_port));

        m_connection_stopped= true;
        bool success= start_connect(endpoint_iter);

        return success;
    }

    void send_request(RequestPtr request)
    {
        m_pending_requests.push_back(request);
        start_write_request();
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
    bool start_connect(tcp::resolver::iterator endpoint_iter)
    {
        bool success= true;

        if (endpoint_iter != tcp::resolver::iterator())
        {
            DEBUG && (cout << "Trying " << endpoint_iter->endpoint() << "..." << endl);

            // Start the asynchronous connect operation.
            m_tcp_socket.async_connect(
                endpoint_iter->endpoint(),
                boost::bind(&ClientNetworkManagerImpl::handle_connect, this, _1, endpoint_iter));
        }
        else
        {
            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_open_failed(boost::asio::error::host_unreachable);
            }

            // There are no more endpoints to try. Shut down the client.
            stop();
            success= false;
        }

        return success;
    }

    void handle_connect(
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
            DEBUG && (std::cerr << "Connect timed out" << endl);

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_open_failed(boost::asio::error::timed_out);
            }

            // Try the next available endpoint.
            start_connect(++endpoint_iter);
        }
        // Check if the connect operation failed before the deadline expired.
        else if (ec)
        {
            DEBUG && (std::cerr << "Connect error: " << ec.message() << endl);

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_open_failed(ec);
            }

            // We need to close the socket used in the previous connection attempt
            // before starting a new one.
            m_tcp_socket.close();

            // Try the next available endpoint.
            start_connect(++endpoint_iter);
        }
        // Otherwise we have successfully established a connection.
        else
        {
            tcp::endpoint tcp_remote_endpoint= endpoint_iter->endpoint();

            DEBUG && (std::cout << "Connected to " << tcp_remote_endpoint << endl);

            // Create a corresponding remote endpoint udp data will be sent to
            m_udp_remote_endpoint= udp::endpoint(tcp_remote_endpoint.address(), tcp_remote_endpoint.port());

            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_opened();
            }

            // Start listening for any incoming responses (TCP messages)
            // NOTE: Responses that come independent of a request are a "notification"
            start_read_response_header();

            // Start listening for any incoming data frames (UDP messages)
            start_read_data_frame_header();

            // If there are any requests waiting 
            if (m_pending_requests.size() > 0)
            {
                start_write_request();
            }
        }
    }

    void start_read_response_header()
    {
        if (!m_has_pending_tcp_read)
        {
            m_has_pending_tcp_read= true;
            m_response_read_buffer.resize(HEADER_SIZE);
            asio::async_read(
                m_tcp_socket, 
                asio::buffer(m_response_read_buffer),
                boost::bind(
                    &ClientNetworkManagerImpl::handle_read_response_header, 
                    shared_from_this(),
                    asio::placeholders::error));
        }
    }

    void handle_read_response_header(const boost::system::error_code& error)
    {
        if (m_connection_stopped)
            return;

        DEBUG && (cout << "handle read " << error.message() << endl);
        if (!error) 
        {
            DEBUG && (cout << "Got header!" << endl);
            DEBUG && (cout << show_hex(m_response_read_buffer) << endl);
            unsigned msg_len = m_packed_response.decode_header(m_response_read_buffer);
            DEBUG && (cout << msg_len << " bytes" << endl);
            start_read_response_body(msg_len);
        }
        else
        {
            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(error);
            }

            DEBUG && (std::cerr << "Error on receive: " << error.message() << endl);
            stop();
        }
    }

    void start_read_response_body(unsigned msg_len)
    {
        assert(m_has_pending_tcp_read);

        // m_read_buffer already contains the header in its first HEADER_SIZE bytes. 
        // Expand it to fit in the body as well, and start async read into the body.
        m_response_read_buffer.resize(HEADER_SIZE + msg_len);
        asio::mutable_buffers_1 buffer = asio::buffer(&m_response_read_buffer[HEADER_SIZE], msg_len);
        asio::async_read(
            m_tcp_socket, 
            buffer,
            boost::bind(
                &ClientNetworkManagerImpl::handle_read_response_body, 
                shared_from_this(),
                asio::placeholders::error));
    }

    void handle_read_response_body(const boost::system::error_code& error)
    {
        if (m_connection_stopped)
            return;

        DEBUG && (cout << "handle body " << error << endl);
        if (!error) 
        {
            DEBUG && (cout << "Got body!\n");
            DEBUG && (cout << show_hex(m_response_read_buffer) << endl);

            // Process the response now that we have received all of it
            handle_response_received();

            // Start reading the next incoming response
            start_read_response_header();
        }
        else
        {
            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(error);
            }

            DEBUG && (cerr << "Error on receive: " << error.message() << endl);
            stop();
        }
    }

    // Called when enough data was read into m_readbuf for a complete response message. 
    // Parse the response and forward it on to the response handler.
    void handle_response_received()
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
                // Responses without a request ID are notifications
                m_notification_listener->handle_notification(response);
            }
        }
        else
        {
            if (m_netEventListener)
            {
                //###bwalker $TODO pick a better error code that means "malformed data"
                m_netEventListener->handle_server_connection_socket_error(boost::asio::error::message_size);
            }

            DEBUG && (cerr << "Error malformed response" << endl);
            stop();
        }
    }

    void start_write_request()
    {        
        if (m_connection_stopped)
            return;

        if (m_pending_requests.size() > 0)
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
                boost::bind(&ClientNetworkManagerImpl::handle_write_request_complete, this, _1));
        }
    }

    void handle_write_request_complete(const boost::system::error_code& ec)
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
            start_read_response_header();

            // If there are more requests waiting to be sent, start sending the next one
            start_write_request();
        }
        else
        {
            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(ec);
            }

            DEBUG && (cerr << "Error on request send: " << ec.message() << endl);
            stop();
        }
    }

    void start_read_data_frame_header()
    {
        if (!m_has_pending_udp_read)
        {
            m_has_pending_udp_read= true;
            m_data_frame_read_buffer.resize(HEADER_SIZE);
            m_udp_socket.async_receive_from(
                asio::buffer(m_data_frame_read_buffer),
                m_udp_remote_endpoint,
                boost::bind(
                    &ClientNetworkManagerImpl::handle_read_data_frame_header, 
                    shared_from_this(),
                    asio::placeholders::error));
        }
    }

    void handle_read_data_frame_header(const boost::system::error_code& error)
    {
        if (m_connection_stopped)
            return;

        DEBUG && (cout << "handle data frame read " << error.message() << endl);
        if (!error) 
        {
            DEBUG && (cout << "Got data frame header!" << endl);
            DEBUG && (cout << show_hex(m_data_frame_read_buffer) << endl);
            unsigned msg_len = m_packed_data_frame.decode_header(m_data_frame_read_buffer);
            DEBUG && (cout << msg_len << " bytes" << endl);
            start_read_data_frame_body(msg_len);
        }
        else
        {
            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(error);
            }

            DEBUG && (std::cerr << "Error on receive: " << error.message() << endl);
            stop();
        }
    }

    void start_read_data_frame_body(unsigned msg_len)
    {
        assert(m_has_pending_udp_read);

        // m_data_frame_read_buffer already contains the header in its first HEADER_SIZE bytes. 
        // Expand it to fit in the body as well, and start async read into the body.
        m_data_frame_read_buffer.resize(HEADER_SIZE + msg_len);
        asio::mutable_buffers_1 buffer = asio::buffer(&m_data_frame_read_buffer[HEADER_SIZE], msg_len);
        m_udp_socket.async_receive_from(
            buffer,
            m_udp_remote_endpoint,
            boost::bind(
                &ClientNetworkManagerImpl::handle_read_data_frame_body, 
                shared_from_this(),
                asio::placeholders::error));
    }

    void handle_read_data_frame_body(const boost::system::error_code& error)
    {
        if (m_connection_stopped)
            return;

        DEBUG && (cout << "handle data frame body " << error << endl);
        if (!error) 
        {
            DEBUG && (cout << "Got data frame body!\n");
            DEBUG && (cout << show_hex(m_data_frame_read_buffer) << endl);

            // Process the data frame now that we have received all of it
            handle_data_frame_received();

            // Start reading the next incoming data frame
            start_read_data_frame_header();
        }
        else
        {
            if (m_netEventListener)
            {
                m_netEventListener->handle_server_connection_socket_error(error);
            }

            DEBUG && (cerr << "Error on data frame receive: " << error.message() << endl);
            stop();
        }
    }

    // Called when enough data was read into m_data_frame_read_buffer for a complete data frame message. 
    // Parse the data_frame and forward it on to the response handler.
    void handle_data_frame_received()
    {
        // No longer is there a pending read
        m_has_pending_udp_read= false;

        // Parse the response buffer
        if (m_packed_data_frame.unpack(m_data_frame_read_buffer))
        {
            ControllerDataFramePtr data_frame = m_packed_data_frame.get_msg();

            m_data_frame_listener->handle_data_frame(data_frame);
        }
        else
        {
            if (m_netEventListener)
            {
                //###bwalker $TODO pick a better error code that means "malformed data"
                m_netEventListener->handle_server_connection_socket_error(boost::asio::error::message_size);
            }

            DEBUG && (cerr << "Error malformed response" << endl);
            stop();
        }
    }

private:
    const std::string &m_server_host;
    const std::string &m_server_port;

    asio::io_service m_io_service;
    tcp::socket m_tcp_socket;
    udp::socket m_udp_socket;
    udp::endpoint m_udp_remote_endpoint;
    bool m_connection_stopped;
    bool m_has_pending_tcp_read;
    bool m_has_pending_tcp_write;
    bool m_has_pending_udp_read;
    
    vector<uint8_t> m_response_read_buffer;
    PackedMessage<PSMoveDataFrame::Response> m_packed_response;

    vector<uint8_t> m_data_frame_read_buffer;
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