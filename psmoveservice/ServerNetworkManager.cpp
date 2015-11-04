//-- includes -----
#include "ServerNetworkManager.h"
#include "packedmessage.h"
#include "PSMoveDataFrame.pb.h"
#include <cassert>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
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

class ClientConnection;
typedef boost::shared_ptr<ClientConnection> ClientConnectionPtr;
typedef vector<ClientConnectionPtr>::iterator ClientConnectionIter;

//-- constants -----
#define DEBUG true

//-- private implementation -----
// -ClientConnection-
// Maintains TCP and UDP connection state to a single client.
// Handles async socket callbacks on the connection.
// Routes requests through the request handler
class ClientConnection : public boost::enable_shared_from_this<ClientConnection>
{
public:
    virtual ~ClientConnection()
    {
        // Socket should have been closed by this point
        assert(!m_tcp_socket.is_open());
    }

    static ClientConnectionPtr create(asio::io_service& io_service, RequestHandler &request_handler)
    {
        return ClientConnectionPtr(new ClientConnection(io_service, request_handler));
    }

    tcp::socket& get_tcp_socket()
    {
        return m_tcp_socket;
    }

    void start()
    {
        start_read_header();
    }

    void stop()
    {
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
    }

private:
    RequestHandler &m_request_handler_ref;
    tcp::socket m_tcp_socket;
    vector<uint8_t> m_readbuf;
    PackedMessage<PSMoveDataFrame::Request> m_packed_request;

    ClientConnection(asio::io_service& io_service, RequestHandler &request_handler)
        : m_request_handler_ref(request_handler)
        , m_tcp_socket(io_service)
        , m_packed_request(boost::shared_ptr<PSMoveDataFrame::Request>(new PSMoveDataFrame::Request()))
    {
    }
    
    void handle_read_header(const boost::system::error_code& error)
    {
        DEBUG && (cerr << "handle read " << error.message() << '\n');
        if (!error) 
        {
            DEBUG && (cerr << "Got header!\n");
            DEBUG && (cerr << show_hex(m_readbuf) << endl);
            unsigned msg_len = m_packed_request.decode_header(m_readbuf);
            DEBUG && (cerr << msg_len << " bytes\n");
            start_read_body(msg_len);
        }
    }

    void handle_read_body(const boost::system::error_code& error)
    {
        DEBUG && (cerr << "handle body " << error << '\n');
        if (!error) 
        {
            DEBUG && (cerr << "Got body!\n");
            DEBUG && (cerr << show_hex(m_readbuf) << endl);
            handle_request();
            start_read_header();
        }
    }

    // Called when enough data was read into m_readbuf for a complete request
    // message. 
    // Parse the request, execute it and send back a response.
    //
    void handle_request()
    {
        if (m_packed_request.unpack(m_readbuf))
        {
            RequestPtr request = m_packed_request.get_msg();
            ResponsePtr response = m_request_handler_ref.handle_request(request);
            
            vector<uint8_t> writebuf;
            PackedMessage<PSMoveDataFrame::Response> resp_msg(response);

            resp_msg.pack(writebuf);
            asio::write(m_tcp_socket, asio::buffer(writebuf));
        }
    }

    void start_read_header()
    {
        m_readbuf.resize(HEADER_SIZE);
        asio::async_read(
            m_tcp_socket, 
            asio::buffer(m_readbuf),
            boost::bind(
                &ClientConnection::handle_read_header, 
                shared_from_this(),
                asio::placeholders::error));
    }

    void start_read_body(unsigned msg_len)
    {
        // m_readbuf already contains the header in its first HEADER_SIZE
        // bytes. Expand it to fit in the body as well, and start async
        // read into the body.
        //
        m_readbuf.resize(HEADER_SIZE + msg_len);
        asio::mutable_buffers_1 buf = asio::buffer(&m_readbuf[HEADER_SIZE], msg_len);
        asio::async_read(
            m_tcp_socket, buf,
            boost::bind(
                &ClientConnection::handle_read_body, 
                shared_from_this(),
                asio::placeholders::error));
    }
};

// -NetworkManagerImpl-
// Internal implementation of the network manager.
class ServerNetworkManagerImpl
{
public:
    ServerNetworkManagerImpl(unsigned port, RequestHandler &requestHandler)
        : request_handler_ref(requestHandler)
        , io_service()
        , tcp_acceptor(io_service, tcp::endpoint(tcp::v4(), port))
        , connections()
    {
    }

    virtual ~ServerNetworkManagerImpl()
    {
        // All connections should have been closed at this point
        assert(connections.empty());
    }

    void start_accept()
    {
        // Create a new connection to handle a client. 
        // Passing a reference to a request handler to each connection poses no problem 
        // since the server is single-threaded.
        ClientConnectionPtr new_connection = 
            ClientConnection::create(tcp_acceptor.get_io_service(), request_handler_ref);

        // Add the connection to the list
        connections.push_back(new_connection);

        // Asynchronously wait to accept a new client
        tcp_acceptor.async_accept(
            new_connection->get_tcp_socket(),
            boost::bind(&ServerNetworkManagerImpl::handle_accept, this, new_connection, asio::placeholders::error));

        // TODO: Add callbacks for client disconnects
    }

    void poll()
    {
        io_service.poll();
    }

    void close_all_connections()
    {
        for (ClientConnectionIter iter= connections.begin(); iter != connections.end(); ++iter)
        {
            ClientConnectionPtr clientConnection= *iter;

            clientConnection->stop();
        }

        connections.clear();
    }

private:
    RequestHandler &request_handler_ref;
    asio::io_service io_service;
    tcp::acceptor tcp_acceptor;
    vector<ClientConnectionPtr> connections;

    void handle_accept(ClientConnectionPtr connection, const boost::system::error_code& error)
    {
        // A new client has connected
        //
        if (!error)
        {
            // Start the connection
            connection->start();

            // Accept another client
            start_accept();
        }
    }
};

//-- public interface -----
ServerNetworkManager::ServerNetworkManager(unsigned port, RequestHandler &requestHandler)
    : implementation_ptr(new ServerNetworkManagerImpl(port, requestHandler))
{
}

bool ServerNetworkManager::startup()
{
    implementation_ptr->start_accept();

    return true;
}

void ServerNetworkManager::update()
{
    implementation_ptr->poll();
}

void ServerNetworkManager::shutdown()
{
    implementation_ptr->close_all_connections();
}

ServerNetworkManager::~ServerNetworkManager()
{
    delete implementation_ptr;
}
