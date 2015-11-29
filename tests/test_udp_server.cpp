//-- includes -----
#define BOOST_LIB_DIAGNOSTIC

#include <boost/asio.hpp>
#include <boost/application.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <cstdio>
#include <string>
#include <signal.h>

// provide setup example for windows service   
#if defined(BOOST_WINDOWS_API)      
#include "setup/windows/setup/service_setup.hpp"
#endif

using namespace boost;
using namespace boost::asio::ip;

//-- constants -----
const int DAYTIME_SERVER_PORT = 9512;

//-- definitions -----
std::string make_daytime_string()
{
  using namespace std; // For time_t, time and ctime;
  time_t now = time(0);
  return ctime(&now);
}

class DaytimeUDPServer
{
public:
    DaytimeUDPServer() 
        : m_io_service()
        , m_udp_socket(m_io_service, udp::endpoint(udp::v4(), DAYTIME_SERVER_PORT))
        , m_remote_endpoint()
        , m_signals(m_io_service)
        , m_status()
    {
        // Register to handle the signals that indicate when the server should exit.
        m_signals.add(SIGINT);
        m_signals.add(SIGTERM);
#if defined(SIGQUIT)
        m_signals.add(SIGQUIT);
#endif // defined(SIGQUIT)
        m_signals.async_wait(boost::bind(&DaytimeUDPServer::handle_termination_signal, this));
    }

    int operator()(application::context& context)
    {
        BOOST_APPLICATION_FEATURE_SELECT

        // Attempt to start and run the service
        try 
        {
            if (startup())
            {
                m_status = context.find<application::status>();

                while (m_status->state() != application::status::stoped)
                {
                    if (m_status->state() != application::status::paused)
                    {
                        update();
                    }

                    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                }
            }
            else
            {
                std::cerr << "Failed to startup the Daytime service" << std::endl;
            }
        }
        catch (std::exception& e) 
        {
            std::cerr << e.what() << std::endl;
        }

        // Attempt to shutdown the service
        try 
        {
           shutdown();
        }
        catch (std::exception& e) 
        {
            std::cerr << e.what() << std::endl;
        }

        return 0;
    }

    bool stop(application::context& context)
    {
        if (m_status->state() != application::status::stoped)
        {
            std::cout << "Received stop request. Stopping Service." << std::endl;
            m_status->state(application::status::stoped);
        }

        return true;
    }

    bool pause(application::context& context)
    {
        if (m_status->state() == application::status::running)
        {
            std::cout << "Received pause request. Pausing Service." << std::endl;
            m_status->state(application::status::paused);
        }

        return true;
    }

    bool resume(application::context& context)
    {
        if (m_status->state() == application::status::paused)
        {
            std::cout << "Received resume request. Resuming Service." << std::endl;
            m_status->state(application::status::running);
        }

        return true;
    }

private:
    bool startup()
    {
        bool success= true;

        start_receive();

        return success;
    }

    void update()
    {
        m_io_service.poll();
    }

    void shutdown()
    {
        if (m_udp_socket.is_open())
        {
            m_udp_socket.shutdown(asio::socket_base::shutdown_both);

            boost::system::error_code error;
            m_udp_socket.close(error);

            if (error)
            {
                std::cerr << "Problem closing the udp socket: " << error.value() << std::endl;
            }
        }
    }

    void handle_termination_signal()
    {
        std::cout << "Received termination signal. Stopping Service." << std::endl;
        m_status->state(application::status::stoped);    
    }

    void start_receive()
    {
        m_udp_socket.async_receive_from(
            boost::asio::buffer(m_recv_buffer), 
            m_remote_endpoint,
            boost::bind(&DaytimeUDPServer::handle_receive, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }

    void handle_receive(
        const boost::system::error_code& error,
        std::size_t /*bytes_transferred*/)
    {
        if (!error || error == boost::asio::error::message_size)
        {
            boost::shared_ptr<std::string> message(
                new std::string(make_daytime_string()));

            m_udp_socket.async_send_to(
                boost::asio::buffer(*message), m_remote_endpoint,
                boost::bind(&DaytimeUDPServer::handle_send, this, message,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));

            start_receive();
        }
    }

    void handle_send(
        boost::shared_ptr<std::string> /*message*/,
        const boost::system::error_code& /*error*/,
        std::size_t /*bytes_transferred*/)
    {
    }

private:   
    // The io_service used to perform asynchronous operations.
    boost::asio::io_service m_io_service;

    // Udp socket we're receiving data on
    udp::socket m_udp_socket;
    
    // The endpoint data is being sent to
    udp::endpoint m_remote_endpoint;

    // The incoming request buffer
    boost::array<char, 1> m_recv_buffer;

    // The signal_set is used to register for process termination notifications.
    boost::asio::signal_set m_signals;

    // Whether the application should keep running or not
    std::shared_ptr<application::status> m_status;
};

#if defined(BOOST_WINDOWS_API) 
bool win32_service_management_action(
    const program_options::variables_map &options_map)
{
    HMODULE hModule = GetModuleHandleW(NULL);
    CHAR path[MAX_PATH];
    GetModuleFileNameA(hModule, path, MAX_PATH);
    std::string exe_full_path(path);

    bool exit_program= false;

    // Install the service
    if (options_map.count("-i")) 
    {
        std::string service_options(" -d");

        boost::system::error_code ec;
        application::example::install_windows_service(
            application::setup_arg(options_map["name"].as<std::string>()), 
            application::setup_arg(options_map["display"].as<std::string>()), 
            application::setup_arg(options_map["description"].as<std::string>()), 
            application::setup_arg(exe_full_path),
            application::setup_arg(std::string("")), // username
            application::setup_arg(std::string("")), // password
            application::setup_arg(service_options)).install(ec);

        std::cout << ec.message() << std::endl;

        exit_program= true;
    }
    // Uninstall the service
    else if (options_map.count("-u")) 
    {
        boost::system::error_code ec;
        application::example::uninstall_windows_service(
            application::setup_arg(options_map["name"].as<std::string>()), 
            application::setup_arg(exe_full_path)).uninstall(ec);

        std::cout << ec.message() << std::endl;

        exit_program= true;
    }
    // Check the status of the service
    else if (options_map.count("-c")) 
    {
        boost::system::error_code ec;
        bool exist =
            application::example::check_windows_service(
                application::setup_arg(options_map["name"].as<std::string>())).exist(ec);

        if(ec)
        {
            std::cout << ec.message() << std::endl;
        }
        else
        {
            if(exist)
            {
                std::cout 
                    << "The service " 
                    << options_map["name"].as<std::string>()
                    <<  " is installed!" 
                    << std::endl;
            }
            else
            {
                std::cout 
                    << "The service " 
                    << options_map["name"].as<std::string>()
                    <<  " is NOT installed!" 
                    << std::endl;
            }
        }

        exit_program= true;
    }

    return exit_program;
}
#endif // defined(BOOST_WINDOWS_API) 

//-- Entry Point ---
int main(int argc, char *argv[])
{
    // used to select between std:: and boost:: namespaces
    BOOST_APPLICATION_FEATURE_SELECT

    // Parse service options
    program_options::variables_map options_map;
    program_options::options_description desc;

    // Extract the executable name
    std::string exe_name = boost::filesystem::path(argv[0]).stem().string();

    // Define the set of valid command line options
    desc.add_options()
        ("help,h", "Shows help.")
        (",d", "Run as background daemon/service")
#if defined(BOOST_WINDOWS_API) 
        (",i", "install service")
        (",u", "uninstall service")
        (",c", "check service")
        ("name", program_options::value<std::string>()->default_value(exe_name), "service name")
        ("display", program_options::value<std::string>()->default_value("UDPDateTimeServer"), "service display name (optional, installation only)")
        ("description", program_options::value<std::string>()->default_value("Emits the current date and time over UDP"), "service description (optional, installation only)")
#endif // defined(BOOST_WINDOWS_API) 
        ;

    // Parse the command line
    try
    {
        program_options::store(program_options::parse_command_line(argc, argv, desc), options_map);
    }
    catch(boost::program_options::unknown_option &option)
    {
        std::cout << option.what() << std::endl;
        std::cout << "Valid Options: " << std::endl;
        std::cout << desc << std::endl;
        return 0;
    }

    if (options_map.count("-h"))
    {
        std::cout << "Valid Options: " << std::endl;
        std::cout << desc << std::endl;
        return 0;
    }

    #if defined(BOOST_WINDOWS_API)
    // On Windows, make a modification to the service installation if requested
    if (win32_service_management_action(options_map))
    {
        // Exit after completing the service management operation
        return 0;
    }
    #endif // defined(BOOST_WINDOWS_API)

    // Start the service app
    std::cout << "Starting UDP Daytime Service" << std::endl;
    try
    {
        DaytimeUDPServer app;
        application::context app_context;

        // service aspects
        app_context.insert<application::path>(
            make_shared<application::path_default_behaviour>(argc, argv));

        app_context.insert<application::args>(
            make_shared<application::args>(argc, argv));

        // add termination handler
        application::handler<>::parameter_callback termination_callback
            = boost::bind<bool>(&DaytimeUDPServer::stop, &app, _1);

        app_context.insert<application::termination_handler>(
            make_shared<application::termination_handler_default_behaviour>(termination_callback));

        // To  "pause/resume" works, is required to add the 2 handlers.
#if defined(BOOST_WINDOWS_API) 
        // windows only : add pause handler     
        application::handler<>::parameter_callback pause_callback
            = boost::bind<bool>(&DaytimeUDPServer::pause, &app, _1);

        app_context.insert<application::pause_handler>(
            make_shared<application::pause_handler_default_behaviour>(pause_callback));

        // windows only : add resume handler
        application::handler<>::parameter_callback resume_callback
            = boost::bind<bool>(&DaytimeUDPServer::resume, &app, _1);

        app_context.insert<application::resume_handler>(
            make_shared<application::resume_handler_default_behaviour>(resume_callback));
#endif // defined(BOOST_WINDOWS_API) 

        // my common/server instantiation
        if (options_map.count("-d"))
        {
            return application::launch<application::server>(app, app_context);
        }
        else
        {
            return application::launch<application::common>(app, app_context);
        }
    }
    catch (boost::system::system_error& se)
    {
        std::cerr << "Failed to start test udp service: " << se.what() << std::endl;
        return 1;
    }
    catch (std::exception &e)
    {
        std::cerr << "Failed to start test udp service: " <<  e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Failed to start test udp service: Unknown error." << std::endl;
        return 1;
    }

    std::cout << "Exiting test udp service" << std::endl;

    return 0;
}
