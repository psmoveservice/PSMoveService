//-- includes -----
#include "ServerNetworkManager.h"
#include "ServerRequestHandler.h"
#include "ServerLog.h"
#include "packedmessage.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
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

class ClientConnection;
typedef boost::shared_ptr<ClientConnection> ClientConnectionPtr;

typedef map<int, ClientConnectionPtr> t_client_connection_map;
typedef map<int, ClientConnectionPtr>::iterator t_client_connection_map_iter;
typedef std::pair<int, ClientConnectionPtr> t_id_client_connection_pair;

//-- private implementation -----
class IServerNetworkEventListener
{
public:
	virtual void handle_client_connection_stopped(int connection_id) = 0;
};

// -ClientConnection-
/**
 * Maintains TCP and UDP connection state to a single client.
 * Handles async socket callbacks on the connection.
 * Routes requests through the request handler
 */
class ClientConnection : public boost::enable_shared_from_this<ClientConnection>
{
public:
    virtual ~ClientConnection()
    {
        // Socket should have been closed by this point
        if (m_tcp_socket.is_open())
        {
            SERVER_LOG_ERROR("~ClientConnection") << "Client connection " << m_connection_id << " deleted without calling stop()";
        }
    }

    static ClientConnectionPtr create(
        IServerNetworkEventListener* network_event_listener,
        asio::io_service& io_service_ref,
        udp::socket& udp_socket_ref, 
        ServerRequestHandler &request_handler_ref)
    {
        return ClientConnectionPtr(
            new ClientConnection(
                network_event_listener, 
                io_service_ref, 
                udp_socket_ref, 
                request_handler_ref));
    }

    int get_connection_id() const
    {
        return m_connection_id;
    }

    tcp::socket& get_tcp_socket()
    {
        return m_tcp_socket;
    }

    void start()
    {
        SERVER_LOG_INFO("ClientConnection::start") << "Starting client connection id " << m_connection_id;

        m_connection_started= true;
        m_connection_stopped= false;

        // Send the connection ID to the client 
        // so that it can send it back to us to establish a UDP connection
        send_connection_info();

        // Wait for incoming requests from the client
        start_tcp_read_request_header();
    }

    void stop()
    {
        if (!m_connection_stopped)
        {
            SERVER_LOG_INFO("ClientConnection::stop") << "Stopping client connection id " << m_connection_id;

            if (m_tcp_socket.is_open())
            {
                boost::system::error_code error;
                
                m_tcp_socket.shutdown(asio::socket_base::shutdown_both, error);
                if (error)
                {
                    SERVER_LOG_ERROR("ClientConnection::stop") << "Unable to shut down the tcp socket: " << error.value();
                }
                
                m_tcp_socket.close(error);
                if (error)
                {
                    SERVER_LOG_ERROR("ClientConnection::stop") << "Unable to close the tcp socket: " << error.value();
                }
            }
            
            m_connection_stopped= true;
            m_has_pending_tcp_write= false;
            m_has_pending_udp_write= false;

            // Notify the parent network manager that this connection is going away
            m_network_event_listener->handle_client_connection_stopped(m_connection_id);
        }
        else
        {
            SERVER_LOG_WARNING("ClientConnection::stop") << "Client connection id " << m_connection_id << " already stopped. Ignoring stop request.";
        }
    }

    void bind_udp_remote_endpoint(const udp::endpoint &connecting_remote_endpoint)
    {
        SERVER_LOG_DEBUG("ClientConnection::bind_udp_remote_endpoint") << "Binding connection_id " 
            << m_connection_id << " to UDP remote endpoint " 
            << connecting_remote_endpoint.address().to_string() << ":"
            << connecting_remote_endpoint.port();

        m_udp_remote_endpoint= connecting_remote_endpoint;
        m_is_udp_remote_endpoint_bound = true;
    }

    bool is_udp_remote_endpoint_bound() const
    {
        return m_is_udp_remote_endpoint_bound;
    }

    bool can_send_data_to_client() const
    {
        return m_connection_started && !m_connection_stopped;
    }

    bool has_pending_udp_write() const
    {
        return m_connection_started && m_has_pending_udp_write;
    }

    bool has_queued_controller_data_frames() const
    {
        return m_connection_started && m_pending_dataframes.size() > 0;
    }

    void add_tcp_response_to_write_queue(ResponsePtr response)
    {
        m_pending_responses.push_back(response);
    }

    bool start_tcp_write_queued_response()
    {
        bool write_in_progress= false;

        if (!m_connection_stopped)
        {
            if (!m_has_pending_tcp_write)
            {
                if (m_pending_responses.size() > 0)
                {
                    ResponsePtr response= m_pending_responses.front();

                    m_packed_response.set_msg(response);
                    m_packed_response.pack(m_response_write_buffer);

                    SERVER_LOG_DEBUG("ClientConnection::start_tcp_write_queued_response") << "Sending TCP response";
                    SERVER_LOG_DEBUG("   ") << show_hex(m_response_write_buffer);
                    SERVER_LOG_DEBUG("   ") << m_packed_response.get_msg()->ByteSize() << " bytes";

                    // The queue should prevent us from writing more than one request as once
                    assert(!m_has_pending_tcp_write);
                    m_has_pending_tcp_write= true;
                    write_in_progress= true;

                    // Start an asynchronous operation to send a heartbeat message.
                    // NOTE: Even if the write completes immediate, the callback will only be called from io_service::poll()
                    boost::asio::async_write(
                        m_tcp_socket, 
                        boost::asio::buffer(m_response_write_buffer),
                        boost::bind(&ClientConnection::handle_write_response_complete, this, _1));
                }
            }
            else
            {
                write_in_progress= true;
            }
        }

        return write_in_progress;
    }
    
    void add_device_data_frame_to_write_queue(DeviceOutputDataFramePtr data_frame)
    {
        m_pending_dataframes.push_back(data_frame);
    }

    bool start_udp_write_queued_device_data_frame()
    {
        bool write_in_progress= false;

        if (m_connection_started && !m_connection_stopped)
        {
            if (!m_has_pending_udp_write)
            {
                if (m_pending_dataframes.size() > 0)
                {
                    DeviceOutputDataFramePtr dataframe= m_pending_dataframes.front();

                    m_packed_output_dataframe.set_msg(dataframe);
                    if (m_packed_output_dataframe.pack(m_output_dataframe_buffer, sizeof(m_output_dataframe_buffer)))
                    {
                        int msg_size= m_packed_output_dataframe.get_msg()->ByteSize();

                        SERVER_LOG_DEBUG("ClientConnection::start_udp_write_queued_device_data_frame") << "Sending UDP DataFrame";
                        SERVER_LOG_DEBUG("   ") << show_hex(m_output_dataframe_buffer, HEADER_SIZE+msg_size);
                        SERVER_LOG_DEBUG("   ") << msg_size << " bytes";

                        // The queue should prevent us from writing more than one data frame at once
                        assert(!m_has_pending_udp_write);
                        m_has_pending_udp_write= true;
                        write_in_progress= true;

                        // Start an asynchronous operation to send the data frame
                        // NOTE: Even if the write completes immediate, the callback will only be called from io_service::poll()
                        m_udp_socket_ref.async_send_to(
                            boost::asio::buffer(m_output_dataframe_buffer, sizeof(m_output_dataframe_buffer)),
                            m_udp_remote_endpoint,
                            boost::bind(&ClientConnection::handle_udp_write_device_data_frame_complete, this, _1));
                    }
                    else
                    {
                        SERVER_LOG_ERROR("ClientConnection::start_udp_write_queued_device_data_frame") 
                            << "DataFrame too big to fit in packet!";
                    }
                }
            }
            else
            {
                write_in_progress= true;
            }
        }

        return write_in_progress;
    }

private:
    static int next_connection_id;

    IServerNetworkEventListener* m_network_event_listener;

    int m_connection_id;

    ServerRequestHandler &m_request_handler_ref;
    tcp::socket m_tcp_socket;
    udp::socket &m_udp_socket_ref;
    udp::endpoint m_udp_remote_endpoint;
    bool m_is_udp_remote_endpoint_bound;

    vector<uint8_t> m_request_read_buffer;
    PackedMessage<PSMoveProtocol::Request> m_packed_request;

    vector<uint8_t> m_response_write_buffer;
    PackedMessage<PSMoveProtocol::Response> m_packed_response;

    uint8_t m_output_dataframe_buffer[HEADER_SIZE+MAX_OUTPUT_DATA_FRAME_MESSAGE_SIZE];
    PackedMessage<PSMoveProtocol::DeviceOutputDataFrame> m_packed_output_dataframe;

    deque<ResponsePtr> m_pending_responses;
    deque<DeviceOutputDataFramePtr> m_pending_dataframes;
    
    bool m_connection_started;
    bool m_connection_stopped;
    bool m_has_pending_tcp_write;
    bool m_has_pending_udp_write;

    ClientConnection(
        IServerNetworkEventListener *network_event_listener,
        asio::io_service& io_service_ref,
        udp::socket& udp_socket_ref , 
        ServerRequestHandler &request_handler_ref)
        : m_network_event_listener(network_event_listener)
        , m_connection_id(next_connection_id)
        , m_request_handler_ref(request_handler_ref)
        , m_tcp_socket(io_service_ref)
        , m_udp_socket_ref(udp_socket_ref)
        , m_udp_remote_endpoint()
        , m_is_udp_remote_endpoint_bound(false)
        , m_request_read_buffer()
        , m_packed_request(std::shared_ptr<PSMoveProtocol::Request>(new PSMoveProtocol::Request()))
        , m_response_write_buffer()
        , m_packed_response()
        , m_packed_output_dataframe()
        , m_pending_responses()
        , m_pending_dataframes()
        , m_connection_started(false)
        , m_connection_stopped(false)
        , m_has_pending_tcp_write(false)
        , m_has_pending_udp_write(false)
    {
        memset(m_output_dataframe_buffer, 0, sizeof(m_output_dataframe_buffer));
        next_connection_id++;
    }

    void send_connection_info()
    {
        SERVER_LOG_INFO("ClientConnection::send_connection_info") 
            << "Sending connection id to client " << m_connection_id;

        ResponsePtr response(new PSMoveProtocol::Response);

        response->set_type(PSMoveProtocol::Response_ResponseType_CONNECTION_INFO);
        response->set_request_id(-1); // This is a notification (no corresponding request)
        response->set_result_code(PSMoveProtocol::Response_ResultCode_RESULT_OK);
        response->mutable_result_connection_info()->set_tcp_connection_id(m_connection_id);

        add_tcp_response_to_write_queue(response);
        start_tcp_write_queued_response();
    }

    void start_tcp_read_request_header()
    {
        SERVER_LOG_DEBUG("ClientConnection::start_tcp_read_request_header") 
            << "Start TCP header read on connection id to client " << m_connection_id;

        m_request_read_buffer.resize(HEADER_SIZE);
        asio::async_read(
            m_tcp_socket, 
            asio::buffer(m_request_read_buffer),
            boost::bind(
                &ClientConnection::handle_tcp_read_request_header, 
                shared_from_this(),
                asio::placeholders::error));
    }

    void handle_tcp_read_request_header(const boost::system::error_code& error)
    {
        if (!error) 
        {
            SERVER_LOG_DEBUG("ClientConnection::handle_tcp_read_request_header") 
                << "Read TCP request header on connection id " << m_connection_id;
            SERVER_LOG_DEBUG("    ") << show_hex(m_request_read_buffer);

            unsigned msg_len = m_packed_request.decode_header(m_request_read_buffer);

            SERVER_LOG_DEBUG("    ") << "Body Size = " << msg_len << " bytes";

            if (msg_len > 0)
            {
                start_tcp_read_request_body(msg_len);
            }
            else
            {
                // If there is no body, jump straight to the handle ready response body callback
                handle_tcp_read_request_body(boost::system::error_code());
            }
        }
        else
        {
            SERVER_LOG_ERROR("ClientConnection::handle_tcp_read_request_header") 
                << "Failed to read header on connection " << m_connection_id << ": " << error.message();
            stop();
        }
    }

    void start_tcp_read_request_body(unsigned msg_len)
    {
        SERVER_LOG_DEBUG("ClientConnection::start_tcp_read_request_body") 
            << "Start TCP request body read on connection id to client " << m_connection_id;

        // m_readbuf already contains the header in its first HEADER_SIZE
        // bytes. Expand it to fit in the body as well, and start async
        // read into the body.
        //
        m_request_read_buffer.resize(HEADER_SIZE + msg_len);
        asio::mutable_buffers_1 buf = asio::buffer(&m_request_read_buffer[HEADER_SIZE], msg_len);
        asio::async_read(
            m_tcp_socket, buf,
            boost::bind(
                &ClientConnection::handle_tcp_read_request_body, 
                shared_from_this(),
                asio::placeholders::error));
    }

    void handle_tcp_read_request_body(const boost::system::error_code& error)
    {
        if (!error) 
        {
            SERVER_LOG_DEBUG("ClientConnection::handle_tcp_read_request_body")
                << "Read request body on connection" << m_connection_id;
            SERVER_LOG_DEBUG("   ") << show_hex(m_request_read_buffer);

            handle_tcp_request();
            start_tcp_read_request_header();
        }
        else
        {
            SERVER_LOG_ERROR("ClientConnection::handle_tcp_read_request_body") 
                << "Failed to read body on connection " << m_connection_id << ": " << error.message();
            stop();
        }
    }

    // Called when enough data was read into m_readbuf for a complete request
    // message. 
    // Parse the request, execute it and send back a response.
    //
    void handle_tcp_request()
    {
        if (m_packed_request.unpack(m_request_read_buffer))
        {
            RequestPtr request = m_packed_request.get_msg();

            SERVER_LOG_DEBUG("ClientConnection::handle_tcp_request") 
                << "Handle request type " << request->request_id() 
                << " on connection id to client " << m_connection_id;

            ResponsePtr response = m_request_handler_ref.handle_request(m_connection_id, request);            
            if (response)
            {
                add_tcp_response_to_write_queue(response);
            }

            start_tcp_write_queued_response();
        }
        else
        {
            SERVER_LOG_ERROR("ClientConnection::handle_tcp_request") 
                << "Failed to parse request on connection " << m_connection_id;
            stop();
        }
    }

    void handle_write_response_complete(const boost::system::error_code& ec)
    {
        if (m_connection_stopped)
            return;

        if (!ec)
        {
            SERVER_LOG_DEBUG("ClientConnection::handle_write_response_complete") 
                << "Sent TCP response on connection id " << m_connection_id;

            // no longer is there a pending write
            m_has_pending_tcp_write= false;

            // Remove the response from the pending send queue now that it's sent
            m_pending_responses.pop_front();

            // If there are more requests waiting to be sent, start sending the next one
            start_tcp_write_queued_response();
        }
        else
        {
            SERVER_LOG_ERROR("ClientConnection::handle_write_response_complete") 
                << "Error sending request on connection " << m_connection_id << ": " << ec.message();
            stop();
        }
    }

    void handle_udp_write_device_data_frame_complete(const boost::system::error_code& ec)
    {
        if (m_connection_stopped)
            return;

        if (!ec)
        {
            SERVER_LOG_TRACE("ClientConnection::handle_udp_write_device_data_frame_complete") 
                << "Sent UDP data frame on connection id " << m_connection_id;

            // no longer is there a pending write
            m_has_pending_udp_write= false;

            // Remove the dataframe from the pending send queue now that it's sent
            m_pending_dataframes.pop_front();
        }
        else
        {
            SERVER_LOG_ERROR("ClientConnection::handle_udp_write_device_data_frame_complete") 
                << "Error sending data frame on connection " << m_connection_id << ": " << ec.message();

            stop();
        }
    }
};
int ClientConnection::next_connection_id = 0;

// -NetworkManagerImpl-
/// Internal implementation of the network manager.
class ServerNetworkManagerImpl : public IServerNetworkEventListener
{
public:
    ServerNetworkManagerImpl(asio::io_service &io_service, unsigned int port, ServerRequestHandler &requestHandler)
        : m_request_handler_ref(requestHandler)
        , m_io_service(io_service)
        , m_tcp_acceptor(m_io_service, tcp::endpoint(tcp::v4(), port))
        , m_udp_socket(m_io_service, udp::endpoint(udp::v4(), port))
        , m_udp_connecting_remote_endpoint()
        , m_packed_input_dataframe(std::shared_ptr<PSMoveProtocol::DeviceInputDataFrame>(new PSMoveProtocol::DeviceInputDataFrame()))
        , m_udp_connection_result_write_buffer(false)
        , m_has_pending_udp_read(false)
        , m_connections()
    {
        memset(m_input_dataframe_buffer, 0, sizeof(m_input_dataframe_buffer));
    }

    virtual ~ServerNetworkManagerImpl()
    {
        // All connections should have been closed at this point
        if (!m_connections.empty())
        {
            SERVER_LOG_ERROR("~ServerNetworkManagerImpl") << "Network manager deleted while there were unclosed connections!";
        }
    }

    //-- ServerNetworkManagerImpl ----
    /// Called during PSMoveService::startup()
    void start_connection_accept()
    {
        SERVER_LOG_DEBUG("ServerNetworkManager::start_tcp_accept") << "Start waiting for a new TCP connection";
        
        // Create a new connection to handle a client.
        // Passing a reference to a request handler to each connection poses no problem 
        // since the server is single-threaded.
        ClientConnectionPtr new_connection = 
            ClientConnection::create(
                this, 
                m_tcp_acceptor.get_io_service(), 
                m_udp_socket, 
                m_request_handler_ref);

        // Add the connection to the list
        t_id_client_connection_pair map_entry(new_connection->get_connection_id(), new_connection);
        m_connections.insert(map_entry);

        // Asynchronously wait to accept a new tcp client
        m_tcp_acceptor.async_accept(
            new_connection->get_tcp_socket(),
            boost::bind(&ServerNetworkManagerImpl::handle_tcp_accept, this, new_connection, asio::placeholders::error));

        
        // Asynchronously wait to accept a new udp clients
        // These should always come after a tcp connection is accepted
        start_udp_read_input_data_frame();
    }

    void poll()
    {
        bool keep_polling= true;
        int iteration_count= 0;
        const static int k_max_iteration_count= 32;

        while (keep_polling && iteration_count < k_max_iteration_count)
        {
            // Start any pending writes on the UDP socket that can be started
            start_udp_queued_data_frame_write();

            // This call can execute any of the following callbacks:
            // * TCP request has finished reading
            // * TCP response has finished writing
            // * UDP data frame has finished writing
            m_io_service.poll();

            // In the event that a UDP data frame write completed immediately,
            // we should start another UDP data frame write.
            keep_polling= has_queued_controller_data_frames_ready_to_start();

            // ... but don't re-run this too many times
            ++iteration_count;
        }
    }

    void close_all_connections()
    {
        SERVER_LOG_DEBUG("ServerNetworkManager::close_all_connections") << "Stopping all client connections";

        // Stop all of the TCP connections
        while (m_connections.size() > 0)
        {
            t_client_connection_map_iter iter= m_connections.begin();
            ClientConnectionPtr clientConnection= iter->second;

            // This should call handle_connection_stopped() which will remove
            // this connection from the list
            clientConnection->stop();
        }

        // Close down the UDP connection
        if (m_udp_socket.is_open())
        {
            boost::system::error_code error;

            m_udp_socket.shutdown(asio::socket_base::shutdown_both, error);
            if (error)
            {
                SERVER_LOG_ERROR("ServerNetworkManager::close_all_connections") << "Problem shutting down the udp socket: " << error.message();
            }

            m_udp_socket.close(error);
            if (error)
            {
                SERVER_LOG_ERROR("ServerNetworkManager::close_all_connections") << "Problem closing the udp socket: " << error.message();
            }
        }

        m_connections.clear();
    }

    void send_notification(int connection_id, ResponsePtr response)
    {
        t_client_connection_map_iter entry = m_connections.find(connection_id);

        // Notifications have an invalid response ID
        response->set_request_id(-1);
        
        if (entry != m_connections.end())
        {
            ClientConnectionPtr connection= entry->second;

            SERVER_LOG_DEBUG("ServerNetworkManager::send_notification") 
                << "Sending response_type " << response->type() 
                << " to connection " << connection_id;

            connection->add_tcp_response_to_write_queue(response);
            connection->start_tcp_write_queued_response();
        }
        else
        {
            SERVER_LOG_DEBUG("ServerNetworkManager::send_notification") 
                << "Can't send response_type " << response->type() 
                << " to a disconnected connection " << connection_id;
        }
    }

    void send_notification_to_all_clients(ResponsePtr response)
    {
        SERVER_LOG_DEBUG("ServerNetworkManager::send_notification") 
            << "Sending response_type " << response->type() << "to all clients";

        // Notifications have an invalid response ID
        response->set_request_id(-1);

        for (t_client_connection_map_iter iter= m_connections.begin(); iter != m_connections.end(); ++iter)
        {
            ClientConnectionPtr connection= iter->second;

            if (connection->can_send_data_to_client())
            {
                connection->add_tcp_response_to_write_queue(response);
                connection->start_tcp_write_queued_response();
            }
        }
    }

    void send_device_data_frame(int connection_id, DeviceOutputDataFramePtr data_frame)
    {
        t_client_connection_map_iter entry = m_connections.find(connection_id);

        if (entry != m_connections.end())
        {
            ClientConnectionPtr connection= entry->second;

            SERVER_LOG_TRACE("ServerNetworkManager::send_device_data_frame") 
                << "Sending data_frame to connection " << connection_id;

            connection->add_device_data_frame_to_write_queue(data_frame);

            start_udp_queued_data_frame_write();
        }
        else
        {
            SERVER_LOG_ERROR("ServerNetworkManager::send_device_data_frame") 
                << "Can't send data_frame to unknown connection " << connection_id;
        }
    }

    // -- IServerNetworkEventListener ----
	virtual void handle_client_connection_stopped(int connection_id) override
    {
        t_client_connection_map_iter entry = m_connections.find(connection_id);

        if (entry != m_connections.end())
        {
            m_connections.erase(entry);
        }

        // Tell the request handler to clean up any state associated with this connection
        m_request_handler_ref.handle_client_connection_stopped(connection_id);
    }

private:
    // Process and responds to incoming PSMoveService request
    ServerRequestHandler &m_request_handler_ref;
    
    // Core i/o functionality for TCP/UDP sockets
    asio::io_service &m_io_service;
    
    // Handles waiting for and accepting new TCP connections
    tcp::acceptor m_tcp_acceptor;

    // UDP socket shared amongst all of the client connections
    udp::socket m_udp_socket;

    // The endpoint of the next connecting 
    udp::endpoint m_udp_connecting_remote_endpoint;

    // A pending udp request from the client
    uint8_t m_input_dataframe_buffer[HEADER_SIZE + MAX_INPUT_DATA_FRAME_MESSAGE_SIZE];
    PackedMessage<PSMoveProtocol::DeviceInputDataFrame> m_packed_input_dataframe;

    // A pending udp result sent to the client
    bool m_udp_connection_result_write_buffer;

    // If true, we are already waiting for a client to send the connection id
    bool m_has_pending_udp_read;

    // A mapping from connection_id -> ClientConnectionPtr
    t_client_connection_map m_connections;

protected:
    void handle_tcp_accept(ClientConnectionPtr connection, const boost::system::error_code& error)
    {        
        // A new client has connected
        //
        if (!error)
        {
            SERVER_LOG_DEBUG("ServerNetworkManager::handle_tcp_accept") << "Accepting a new connection";
            
            // Start the connection
            connection->start();
        }
        else
        {
            SERVER_LOG_DEBUG("ServerNetworkManager::handle_tcp_accept") << 
                "Failed to accept new connection: " << error.message();

            // Stop the failed connection
            connection->stop();
        }

        // Accept another client
        start_connection_accept();
    }

    void start_udp_read_input_data_frame()
    {
        if (!m_has_pending_udp_read)
        {
            SERVER_LOG_DEBUG("ServerNetworkManager::start_udp_receive_connection_id") << "waiting for UDP input dataframe";

            m_has_pending_udp_read = true;
            m_udp_socket.async_receive_from(
                asio::buffer(m_input_dataframe_buffer, sizeof(m_input_dataframe_buffer)),
                m_udp_connecting_remote_endpoint,
                boost::bind(
                    &ServerNetworkManagerImpl::handle_udp_read_data_frame,
                    this,
                    asio::placeholders::error));
        }
    }

    void handle_udp_read_data_frame(const boost::system::error_code& error)
    {
        m_has_pending_udp_read= false;

        if (!error) 
        {
            // Parse the incoming data frame
            handle_udp_data_frame_received();
        }
        else
        {
            SERVER_LOG_ERROR("ServerNetworkManager::handle_udp_read_connection_id") 
                << "Failed to receive UDP connection id: "<< error.message();
        }

        // Start reading the next incoming data frame
        start_udp_read_input_data_frame();
    }

    // Called when enough data was read into m_data_frame_read_buffer for a complete data frame message. 
    // Parse the data_frame and forward it on to the response handler.
    void handle_udp_data_frame_received()
    {
        // No longer is there a pending read
        m_has_pending_udp_read = false;

        SERVER_LOG_DEBUG("ClientNetworkManager::handle_udp_data_frame_received") << "Parsing DataFrame" << std::endl;

        // TODO: Switch on data frame type to choose which m_packed_data_frame_X to use.
        unsigned msg_len = m_packed_input_dataframe.decode_header(m_input_dataframe_buffer, sizeof(m_input_dataframe_buffer));
        unsigned total_len = HEADER_SIZE + msg_len;
        SERVER_LOG_DEBUG("    ") << show_hex(m_input_dataframe_buffer, total_len) << std::endl;
        SERVER_LOG_DEBUG("    ") << msg_len << " bytes" << std::endl;

        // Parse the response buffer
        if (m_packed_input_dataframe.unpack(m_input_dataframe_buffer, total_len))
        {
            DeviceInputDataFramePtr data_frame = m_packed_input_dataframe.get_msg();

            // Find the connection with the matching id
            t_client_connection_map_iter iter = m_connections.find(data_frame->connection_id());

            if (iter != m_connections.end())
            {
                SERVER_LOG_DEBUG("ServerNetworkManager::handle_udp_data_frame_received")
                    << "Found UDP client connected with matching connection_id: " << data_frame->connection_id();

                ClientConnectionPtr connection = iter->second;

                // Bind the udp endpoint if this is the first UDP packet received from the client
                if (!connection->is_udp_remote_endpoint_bound())
                {
                    // Associate this udp remote endpoint with the given connection id
                    connection->bind_udp_remote_endpoint(m_udp_connecting_remote_endpoint);

                    // Tell the client that this was a valid connection id
                    start_udp_send_connection_result(true);
                }

                // Process the incoming data frame
                m_request_handler_ref.handle_input_data_frame(data_frame);
            }
            else 
            {
                SERVER_LOG_ERROR("ServerNetworkManager::handle_udp_data_frame_received")
                    << "UDP client connected with INVALID connection_id: " << data_frame->connection_id();

                if (data_frame->device_category() == PSMoveProtocol::DeviceInputDataFrame_DeviceCategory_INVALID)
                {
                    // If the device category was invalid, then this must have been an initial dataframe sent at device connection
                    // Tell the client that this was an invalid connection id
                    start_udp_send_connection_result(false);
                }
            }
        }
    }

    void start_udp_send_connection_result(bool success)
    {
        SERVER_LOG_DEBUG("ServerNetworkManager::start_udp_send_connection_result") 
            << "Send result: " << success;

        m_udp_connection_result_write_buffer= success;
        m_udp_socket.async_send_to(
            boost::asio::buffer(&m_udp_connection_result_write_buffer, sizeof(m_udp_connection_result_write_buffer)), 
            m_udp_connecting_remote_endpoint,
            boost::bind(&ServerNetworkManagerImpl::handle_udp_write_connection_result, this, boost::asio::placeholders::error));
    }

    void handle_udp_write_connection_result(const boost::system::error_code& error)
    {
        if (error) 
        {
            SERVER_LOG_ERROR("ServerNetworkManager::handle_udp_write_connection_result") 
                << "Failed to send UDP connection response: "<< error.message();
        }

        // Start waiting for the next connection result
        start_udp_read_input_data_frame();
    }

    void start_udp_queued_data_frame_write()
    {
        for (t_client_connection_map_iter iter= m_connections.begin(); iter != m_connections.end(); ++iter)
        {
            ClientConnectionPtr connection= iter->second;

            if (connection->start_udp_write_queued_device_data_frame())
            {
                SERVER_LOG_TRACE("ServerNetworkManager::start_udp_queued_data_frame_write") 
                    << "Send queued UDP data on connection id: " << iter->first;

                // Don't start a write on any other connection until this one is finished 
                break;
            }
        }        
    }

    bool has_queued_controller_data_frames_ready_to_start()
    {
        bool has_queued_write_ready_to_start= false;
        bool udp_socket_available= true;

        for (t_client_connection_map_iter iter= m_connections.begin(); iter != m_connections.end(); ++iter)
        {
            ClientConnectionPtr connection= iter->second;

            if (connection->has_pending_udp_write())
            {
                // Can't start any new udp write until any current udp write is done
                udp_socket_available= false;
                break;
            }

            if (connection->has_queued_controller_data_frames())
            {
                // Found a connection with a pending udp write ready to go
                has_queued_write_ready_to_start= true;
            }
        }

        return udp_socket_available && has_queued_write_ready_to_start;
    }
};

//-- public interface -----
ServerNetworkManager *ServerNetworkManager::m_instance = NULL;

ServerNetworkManager::ServerNetworkManager(
    boost::asio::io_service *io_service,
    unsigned port,
    ServerRequestHandler *requestHandler)
    : implementation_ptr(new ServerNetworkManagerImpl(*io_service, port, *requestHandler))
{
}

ServerNetworkManager::~ServerNetworkManager()
{
    if (m_instance != NULL)
    {
        SERVER_LOG_ERROR("~ServerNetworkManager()") << "Network Manager deleted without shutdown() getting called first";
    }

    if (implementation_ptr != nullptr)
    {
        delete implementation_ptr;
        implementation_ptr= nullptr;
    }
}

bool ServerNetworkManager::startup()
{
    
    m_instance= this;
    
    implementation_ptr->start_connection_accept();

    return true;
}

void ServerNetworkManager::update()
{
    implementation_ptr->poll();
}

void ServerNetworkManager::shutdown()
{
    
    implementation_ptr->close_all_connections();
    
    m_instance= NULL;
}

void ServerNetworkManager::send_notification(int connection_id, ResponsePtr response)
{
    implementation_ptr->send_notification(connection_id, response);
}

void ServerNetworkManager::send_notification_to_all_clients(ResponsePtr response)
{
    implementation_ptr->send_notification_to_all_clients(response);
}

void ServerNetworkManager::send_device_data_frame(int connection_id, DeviceOutputDataFramePtr data_frame)
{
    implementation_ptr->send_device_data_frame(connection_id, data_frame);
}
