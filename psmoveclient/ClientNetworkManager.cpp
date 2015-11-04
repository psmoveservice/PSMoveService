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
using boost::uint8_t;

//-- constants -----
#define DEBUG true

//-- implementation -----

// -ClientNetworkManagerImpl-
// Internal implementation of the client network manager.
class ClientNetworkManagerImpl : public boost::enable_shared_from_this<ClientNetworkManagerImpl>
{
public:
    ClientNetworkManagerImpl(const std::string &host, const std::string &port, ResponseHandler &requestHandler)
        : m_server_host(host)
        , m_server_port(port)

        , m_io_service()
        , m_tcp_socket(m_io_service)
        , m_connection_stopped(false)
        , m_has_pending_read(false)
        , m_has_pending_write(false)
        , m_pending_response_count(0)

        , m_read_buffer()
        , m_packed_response()
    
        , m_write_bufer()
        , m_packed_request()

        , m_response_handler_ref(requestHandler)
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
            m_response_handler_ref.pending_request_canceled(m_pending_requests.front());
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
            }
        }

        m_connection_stopped= true;
        m_has_pending_read= false;
        m_has_pending_write= false;
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
            // TODO: handle socket disconnection
        }
        else
        {
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

            // Try the next available endpoint.
            start_connect(++endpoint_iter);
        }
        // Check if the connect operation failed before the deadline expired.
        else if (ec)
        {
            DEBUG && (std::cerr << "Connect error: " << ec.message() << endl);

            // We need to close the socket used in the previous connection attempt
            // before starting a new one.
            m_tcp_socket.close();

            // Try the next available endpoint.
            start_connect(++endpoint_iter);
        }
        // Otherwise we have successfully established a connection.
        else
        {
            DEBUG && (std::cout << "Connected to " << endpoint_iter->endpoint() << endl);

            // If there are any requests waiting 
            if (m_pending_requests.size() > 0)
            {
                start_write_request();
            }
        }
    }

    void start_read_response_header()
    {
        if (!m_has_pending_read && m_pending_response_count > 0)
        {
            m_has_pending_read= true;
            m_read_buffer.resize(HEADER_SIZE);
            asio::async_read(
                m_tcp_socket, 
                asio::buffer(m_read_buffer),
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
            DEBUG && (cout << show_hex(m_read_buffer) << endl);
            unsigned msg_len = m_packed_response.decode_header(m_read_buffer);
            DEBUG && (cout << msg_len << " bytes" << endl);
            start_read_response_body(msg_len);
        }
        else
        {
            DEBUG && (std::cerr << "Error on receive: " << error.message() << endl);
            stop();
        }
    }

    void start_read_response_body(unsigned msg_len)
    {
        assert(m_has_pending_read);

        // m_read_buffer already contains the header in its first HEADER_SIZE bytes. 
        // Expand it to fit in the body as well, and start async read into the body.
        m_read_buffer.resize(HEADER_SIZE + msg_len);
        asio::mutable_buffers_1 buffer = asio::buffer(&m_read_buffer[HEADER_SIZE], msg_len);
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
            DEBUG && (cout << show_hex(m_read_buffer) << endl);

            // Process the response now that we have received all of it
            handle_response_received();

            // Start reading the next incoming request
            start_read_response_header();
        }
        else
        {
            DEBUG && (cerr << "Error on receive: " << error.message() << endl);
            stop();
        }
    }

    // Called when enough data was read into m_readbuf for a complete response message. 
    // Parse the response and forward it on to the response handler.
    void handle_response_received()
    {
        // No longer is there a pending read
        m_has_pending_read= false;

        // One less response we are expecting
        m_pending_response_count--;
        assert(m_pending_response_count >= 0);

        // Parse the response buffer
        if (m_packed_response.unpack(m_read_buffer))
        {
            ResponsePtr response = m_packed_response.get_msg();

            m_response_handler_ref.handle_response(response);
        }
        else
        {
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
            assert(!m_has_pending_write);
            m_has_pending_write= true;

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
            m_has_pending_write= false;

            // Remove the request from the pending send queue not that it's sent
            m_pending_requests.pop_front();

            // We now have one more response pending
            m_pending_response_count++;

            // Start listening for the response
            start_read_response_header();

            // If there are more requests waiting to be sent, start sending the next one
            start_write_request();
        }
        else
        {
            DEBUG && (cerr << "Error on request send: " << ec.message() << endl);

            stop();
        }
    }

private:
    const std::string &m_server_host;
    const std::string &m_server_port;

    asio::io_service m_io_service;
    tcp::socket m_tcp_socket;
    bool m_connection_stopped;
    bool m_has_pending_read;
    bool m_has_pending_write;
    int m_pending_response_count;
    
    vector<uint8_t> m_read_buffer;
    PackedMessage<PSMoveDataFrame::Response> m_packed_response;
    
    vector<uint8_t> m_write_bufer;
    PackedMessage<PSMoveDataFrame::Request> m_packed_request;

    ResponseHandler &m_response_handler_ref;
    deque<RequestPtr> m_pending_requests;
};

// -NetworkManager-
// Public interface to the psmove client network API
ClientNetworkManager::ClientNetworkManager(
    const std::string &host, 
    const std::string &port, 
    ResponseHandler &responseHandler)
    : implementation_ptr(new ClientNetworkManagerImpl(host, port, responseHandler))
{
}

ClientNetworkManager::~ClientNetworkManager()
{
    delete implementation_ptr;
}

bool ClientNetworkManager::startup()
{
    return implementation_ptr->start();
}

void ClientNetworkManager::send_request(RequestPtr request)
{
    implementation_ptr->send_request(request);
}

void ClientNetworkManager::update()
{
    implementation_ptr->poll();
}

void ClientNetworkManager::shutdown()
{
    implementation_ptr->stop();
}