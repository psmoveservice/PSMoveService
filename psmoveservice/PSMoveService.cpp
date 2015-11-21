//-- includes -----
#define BOOST_LIB_DIAGNOSTIC

#include "ServerNetworkManager.h"
#include "ServerRequestHandler.h"
#include "ControllerManager.h"
#include "ServerLog.h"

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

//-- constants -----
const int PSMOVE_SERVER_PORT = 9512;

//-- definitions -----
class PSMoveService
{
public:
    PSMoveService() 
        : m_io_service()
        , m_signals(m_io_service)
        , m_controller_manager()
        , m_request_handler(&m_controller_manager)
        , m_network_manager(&m_io_service, PSMOVE_SERVER_PORT, &m_request_handler)
        , m_status()
    {
        // Register to handle the signals that indicate when the server should exit.
        m_signals.add(SIGINT);
        m_signals.add(SIGTERM);
#if defined(SIGQUIT)
        m_signals.add(SIGQUIT);
#endif // defined(SIGQUIT)
        m_signals.async_wait(boost::bind(&PSMoveService::handle_termination_signal, this));
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

                    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                }
            }
            else
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to startup the PSMove service";
            }
        }
        catch (std::exception& e) 
        {
            SERVER_LOG_FATAL("PSMoveService") << e.what();
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
            SERVER_LOG_WARNING("PSMoveService") << "Received stop request. Stopping Service.";
            m_status->state(application::status::stoped);
        }

        return true;
    }

    bool pause(application::context& context)
    {
        if (m_status->state() == application::status::running)
        {
            SERVER_LOG_WARNING("PSMoveService") << "Received pause request. Pausing Service.";
            m_status->state(application::status::paused);
        }

        return true;
    }

    bool resume(application::context& context)
    {
        if (m_status->state() == application::status::paused)
        {
            SERVER_LOG_WARNING("PSMoveService") << "Received resume request. Resuming Service.";
            m_status->state(application::status::running);
        }

        return true;
    }

private:
    bool startup()
    {
        bool success= true;

        // Start listening for client connections
        if (success)
        {
            if (!m_network_manager.startup())
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to initialize the service network manager";
                success= false;
            }
        }

        // Setup the request handler
        if (success)
        {
            if (!m_request_handler.startup())
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to initialize the service request handler";
                success= false;
            }
        }

        // Setup the controller manager
        if (success)
        {
            if (!m_controller_manager.startup())
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to initialize the controller manager";
                success= false;
            }
        }

        return success;
    }

    void update()
    {
        // Update the list of active tracked controllers
        // Send controller updates to the client
        m_controller_manager.update();

        // Process incoming/outgoing networking requests
        m_network_manager.update();
    }

    void shutdown()
    {
        // Disconnect any actively connected controllers
        m_controller_manager.shutdown();

        // Kill any pending request state
        m_request_handler.shutdown();

        // Close all active network connections
        m_network_manager.shutdown();
    }

    void handle_termination_signal()
    {
        SERVER_LOG_WARNING("PSMoveService") << "Received termination signal. Stopping Service.";
        m_status->state(application::status::stoped);
    }

private:   
    // The io_service used to perform asynchronous operations.
    boost::asio::io_service m_io_service;

    // The signal_set is used to register for process termination notifications.
    boost::asio::signal_set m_signals;

    // Keep track of currently connected PSMove controllers
    ControllerManager m_controller_manager;

    // Generates responses from incoming requests sent to the network manager
    ServerRequestHandler m_request_handler;

    // Manages all TCP and UDP client connections
    ServerNetworkManager m_network_manager;

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

        if (options_map.count("log_level"))
        {
            std::string log_level= options_map["log_level"].as<std::string>();

            service_options+= " --log_level ";
            service_options+= log_level;
        }

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
        ("log_level,l", program_options::value<std::string>(), "The level of logging to use: trace, debug, info, warning, error, fatal")
#if defined(BOOST_WINDOWS_API) 
        (",i", "install service")
        (",u", "uninstall service")
        (",c", "check service")
        ("name", program_options::value<std::string>()->default_value(exe_name), "service name")
        ("display", program_options::value<std::string>()->default_value("PSMove Service"), "service display name (optional, installation only)")
        ("description", program_options::value<std::string>()->default_value("Manages PSMove controller and broadcasts state to clients"), "service description (optional, installation only)")
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

    // initialize logging system
    log_init(&options_map);

    // Start the service app
    SERVER_LOG_INFO("main") << "Starting PSMoveService";
    try
    {
        PSMoveService app;
        application::context app_context;

        // service aspects
        app_context.insert<application::path>(
            make_shared<application::path_default_behaviour>(argc, argv));

        app_context.insert<application::args>(
            make_shared<application::args>(argc, argv));

        // add termination handler
        application::handler<>::parameter_callback termination_callback
            = boost::bind<bool>(&PSMoveService::stop, &app, _1);

        app_context.insert<application::termination_handler>(
            make_shared<application::termination_handler_default_behaviour>(termination_callback));

        // To  "pause/resume" works, is required to add the 2 handlers.
#if defined(BOOST_WINDOWS_API) 
        // windows only : add pause handler     
        application::handler<>::parameter_callback pause_callback
            = boost::bind<bool>(&PSMoveService::pause, &app, _1);

        app_context.insert<application::pause_handler>(
            make_shared<application::pause_handler_default_behaviour>(pause_callback));

        // windows only : add resume handler
        application::handler<>::parameter_callback resume_callback
            = boost::bind<bool>(&PSMoveService::resume, &app, _1);

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
        SERVER_LOG_FATAL("main") << "Failed to start PSMoveService: " << se.what();
        return 1;
    }
    catch (std::exception &e)
    {
        SERVER_LOG_FATAL("main") << "Failed to start PSMoveService: " <<  e.what();
        return 1;
    }
    catch (...)
    {
        SERVER_LOG_FATAL("main") << "Failed to start PSMoveService: Unknown error.";
        return 1;
    }

    SERVER_LOG_INFO("main") << "Exiting PSMoveService";

    return 0;
}
