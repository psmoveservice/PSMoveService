#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <chrono>

#if defined(__linux) || defined (__APPLE__)
#include <unistd.h>
#endif
#ifdef _WIN32
#include <windows.h>
#endif

#define REQUEST_INTERVAL 1000 // ms

bool g_keep_running= true;
bool g_has_pending_udp_read= false;
boost::array<char, 128> g_recv_buf;
boost::asio::ip::udp::endpoint g_sender_endpoint;

void sleep_millisecond(int sleepMs)
{
#if defined(__linux) || defined (__APPLE__)
    usleep(sleepMs * 1000);
#endif
#ifdef WINDOWS
    Sleep(sleepMs);
#endif
}

void handle_termination_signal(
    const boost::system::error_code& error, // Result of operation.
    int signal_number)
{
    std::cout << "Received termination signal. Stopping Client." << std::endl;
    g_keep_running= false;
}

void handle_udp_receive(
    const boost::system::error_code& error,
    std::size_t bytes_transferred)
{
    g_has_pending_udp_read= false;

    if (!error) 
    {
        std::cout << "Received UDP response: ";
        std::cout.write(g_recv_buf.data(), bytes_transferred);
    }
    else
    {
        std::cerr << "Failed to receive UDP response: "<< error.message() << std::endl;
    }
}

int main(int argc, char* argv[])
{
    try
    {
        if (argc != 2)
        {
            std::cerr << "Usage: client <host>" << std::endl;
            return 1;
        }

        boost::asio::io_service io_service;

        // Register to handle the signals that indicate when the server should exit.
        boost::asio::signal_set signals(io_service);
        signals.add(SIGINT);
        signals.add(SIGTERM);
#if defined(SIGQUIT)
        signals.add(SIGQUIT);
#endif // defined(SIGQUIT)
        signals.async_wait(&handle_termination_signal);

        boost::asio::ip::udp::resolver resolver(io_service);
        boost::asio::ip::udp::resolver::query query(boost::asio::ip::udp::v4(), argv[1], "9512");
        boost::asio::ip::udp::endpoint receiver_endpoint = *resolver.resolve(query);

        boost::asio::ip::udp::socket socket(io_service);
        socket.open(boost::asio::ip::udp::v4());

        while (g_keep_running)
        {
            if (!g_has_pending_udp_read)
            {
                g_has_pending_udp_read= true;

                std::cout << "Sending Daytime Request..." << std::endl;

                boost::array<char, 1> send_buf  = { 0 };
                socket.send_to(boost::asio::buffer(send_buf), receiver_endpoint);
                       
                socket.async_receive_from(boost::asio::buffer(g_recv_buf), g_sender_endpoint, &handle_udp_receive);
            }

            io_service.poll();

            sleep_millisecond(REQUEST_INTERVAL);
        }

        socket.close();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

  return 0;
}
