//-- includes -----
#define BOOST_LIB_DIAGNOSTIC

#include "PSMoveService.h"
#include "ServerNetworkManager.h"
#include "ServerRequestHandler.h"
#include "DeviceManager.h"
#include "ServerLog.h"
#include "TrackerManager.h"
#include "USBDeviceManager.h"

#include <boost/asio.hpp>
#include <boost/application.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <cstdio>
#include <string>
#include <chrono>
#include <thread>
#include <signal.h>

// provide setup example for windows service   
#if defined(BOOST_WINDOWS_API)      
#include "setup/windows/setup/service_setup.hpp"
#endif // defined(BOOST_WINDOWS_API)

//-- constants -----
#if defined(BOOST_POSIX_API)
#define DAEMON_RUNNING_DIR	"/tmp"
#define DAEMON_LOCK_FILE	"psmoveserviced.lock"
#endif // defined(BOOST_POSIX_API)

const int PSMOVE_SERVER_PORT = 9512;

//-- constants -----
// List of all possible USB devices that we want to connect to via libusb
// VendorID, ProductID
USBDeviceFilter k_usb_device_whitelist[2] = {
	{ 0x054c, 0x042F }, // PSNavi
	{ 0x1415, 0x2000 }, // PS3Eye
    //{ 0x05a9, 0x058a }, // PS4 Camera - TODO
};

//-- definitions -----
class PSMoveServiceImpl
{
public:
    PSMoveServiceImpl()
        : m_io_service()
        , m_signals(m_io_service)
        , m_usb_device_manager(k_usb_device_whitelist, 2)
        , m_device_manager()
        , m_request_handler(&m_device_manager)
        , m_network_manager(&m_io_service, PSMOVE_SERVER_PORT, &m_request_handler)
        , m_status()
    {
        // Register to handle the signals that indicate when the server should exit.
        m_signals.add(SIGINT);
        m_signals.add(SIGTERM);
#if defined(SIGQUIT)
        m_signals.add(SIGQUIT);
#endif // defined(SIGQUIT)
        m_signals.async_wait(boost::bind(&PSMoveServiceImpl::handle_termination_signal, this));
    }

    /// Entry point into boost::application
    int operator()(boost::application::context& context)
    {
        BOOST_APPLICATION_FEATURE_SELECT

        // Attempt to start and run the service
        try 
        {
            if (startup())
            {
                m_status = context.find<boost::application::status>();

				const TrackerManagerConfig &cfg = DeviceManager::getInstance()->m_tracker_manager->getConfig();

                while (m_status->state() != boost::application::status::stoped)
                {
                    if (m_status->state() != boost::application::status::paused)
                    {
                        update();
                    }

					std::this_thread::sleep_for(std::chrono::milliseconds(cfg.tracker_sleep_ms));
                }
            }
            else
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to startup the PSMove service";
            }
        }
        catch (std::exception& e) 
        {
            SERVER_LOG_FATAL("EXCEPTION - PSMoveService") << e.what();
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
    
    bool stop(boost::application::context& context)
    {
        if (m_status->state() != boost::application::status::stoped)
        {
            SERVER_LOG_WARNING("PSMoveService") << "Received stop request. Stopping Service.";
            m_status->state(boost::application::status::stoped);
        }

        return true;
    }

    bool pause(boost::application::context& context)
    {
        if (m_status->state() == boost::application::status::running)
        {
            SERVER_LOG_WARNING("PSMoveService") << "Received pause request. Pausing Service.";
            m_status->state(boost::application::status::paused);
        }

        return true;
    }

    bool resume(boost::application::context& context)
    {
        if (m_status->state() == boost::application::status::paused)
        {
            SERVER_LOG_WARNING("PSMoveService") << "Received resume request. Resuming Service.";
            m_status->state(boost::application::status::running);
        }

        return true;
    }

private:
    /// Called upon PSMoveService application start.
    bool startup()
    {
        bool success= true;
        
        /** Start listening for client connections */
        if (success)
        {
            if (!m_network_manager.startup())
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to initialize the service network manager";
                success= false;
            }
        }

        /** Setup the usb async transfer thread before we attempt to initialize the trackers */
        if (success)
        {
            if (!m_usb_device_manager.startup())
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to initialize the usb async request manager";
                success = false;
            }
        }

        /** Setup the controller manager */
        if (success)
        {
            if (!m_device_manager.startup())
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to initialize the controller manager";
                success= false;
            }
        }

        /** Setup the request handler */
        if (success)
        {
            if (!m_request_handler.startup())
            {
                SERVER_LOG_FATAL("PSMoveService") << "Failed to initialize the service request handler";
                success= false;
            }
        }

        return success;
    }

    /// Called in the application loop.
    void update()
    {
        /** Update an async requests still waiting to complete */
        m_request_handler.update();

        /** Process any async results from the USB transfer thread */
        m_usb_device_manager.update();

        /**
         Update the list of active tracked controllers
         Send controller updates to the client
         */
        m_device_manager.update();

        /** Process incoming/outgoing networking requests */
        m_network_manager.update();
    }

    void shutdown()
    {
        // Kill any pending request state
        m_request_handler.shutdown();

        // Shutdown the usb async request thread
        m_usb_device_manager.shutdown();

        // Close all active network connections
        m_network_manager.shutdown();

        // Disconnect any actively connected controllers
        m_device_manager.shutdown();
    }

    void handle_termination_signal()
    {
        // flag the service as stopped
        SERVER_LOG_WARNING("PSMoveService") << "Received termination signal. Stopping Service.";
        m_status->state(boost::application::status::stoped);
    }

private:   
    // The io_service used to perform asynchronous operations.
    boost::asio::io_service m_io_service;
       
    // The signal_set is used to register for process termination notifications.
    boost::asio::signal_set m_signals;

    // Manages all control and bulk transfer requests in another thread
    USBDeviceManager m_usb_device_manager;

    // Keep track of currently connected devices (PSMove controllers, cameras, HMDs)
    DeviceManager m_device_manager;

    // Generates responses from incoming requests sent to the network manager
    ServerRequestHandler m_request_handler;

    // Manages all TCP and UDP client connections
    ServerNetworkManager m_network_manager;

    // Whether the application should keep running or not
    std::shared_ptr<boost::application::status> m_status;
};

static void parse_program_settings(
    const boost::program_options::variables_map &options_map,
    PSMoveService::ProgramSettings &settings)
{
    if (options_map.count("log_level"))
    {
        settings.log_level= options_map["log_level"].as<std::string>();
    }
    else
    {
        settings.log_level= "info";
    }
    
    if (options_map.count("admin_password"))
    {
        settings.admin_password= options_map["admin_password"].as<std::string>();
    }
    else
    {
        settings.admin_password.clear();
    }
}

#if defined(BOOST_WINDOWS_API) 
bool win32_service_management_action(
    const boost::program_options::variables_map &options_map)
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
		boost::application::example::install_windows_service(
            boost::application::setup_arg(options_map["name"].as<std::string>()), 
            boost::application::setup_arg(options_map["display"].as<std::string>()), 
            boost::application::setup_arg(options_map["description"].as<std::string>()), 
            boost::application::setup_arg(exe_full_path),
            boost::application::setup_arg(std::string("")), // username
            boost::application::setup_arg(std::string("")), // password
            boost::application::setup_arg(service_options)).install(ec);

        std::cout << ec.message() << std::endl;

        exit_program= true;
    }
    // Uninstall the service
    else if (options_map.count("-u")) 
    {
        boost::system::error_code ec;
		boost::application::example::uninstall_windows_service(
			boost::application::setup_arg(options_map["name"].as<std::string>()),
			boost::application::setup_arg(exe_full_path)).uninstall(ec);

        std::cout << ec.message() << std::endl;

        exit_program= true;
    }
    // Check the status of the service
    else if (options_map.count("-c")) 
    {
        boost::system::error_code ec;
        bool exist =
			boost::application::example::check_windows_service(
				boost::application::setup_arg(options_map["name"].as<std::string>())).exist(ec);

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

#if defined(BOOST_POSIX_API)
void daemonize()
{
    // already a daemon
    if(getppid()==1)
    {
        return;
    }
    
    // Fork off a child process
    {
        int result_pid = fork();
        
        if (result_pid < 0)
        {
            std::cerr << "Error forking child daemon process" << std::endl;
            exit(1); /* fork error */
        }
        
        if (result_pid > 0)
        {
            std::cout << "Successfully forked child daemon process. Exiting parent." << std::endl;
            exit(0); /* parent exits */
        }
    }
    
    // child (daemon) continues
    
    // obtain a new process group
    setsid();
    
    // close all file descriptors
    for (int fd_index=getdtablesize(); fd_index>=0; --fd_index)
    {
        close(fd_index);
    }
    
    // alias cout and cerr to /dev/null
    int fd_dev_null = open("/dev/null",O_RDWR);
    dup(fd_dev_null);
    dup(fd_dev_null);
    
    // set newly created file permissions
    umask(027);
    
    // change running directory
    chdir(DAEMON_RUNNING_DIR);
    
    // Create the lock file
    {
        int lock_fp = open(DAEMON_LOCK_FILE, O_RDWR|O_CREAT, 0640);
        
        // can not open
        if (lock_fp < 0)
        {
            exit(1);
        }
        
        // can not lock
        if (lockf(lock_fp, F_TLOCK, 0) < 0)
        {
            exit(0);
        }
        
        // first instance continues
        char str[10];
        sprintf(str,"%d\n",getpid());
        
        // record pid to lockfile
        write(lock_fp, str, strlen(str));
    }
    
    signal(SIGCHLD,SIG_IGN); // ignore child
    signal(SIGTSTP,SIG_IGN); // ignore tty signals
    signal(SIGTTOU,SIG_IGN);
    signal(SIGTTIN,SIG_IGN);
}
#endif // defined(BOOST_POSIX_API)

//-- Public Interface ---
PSMoveService *PSMoveService::m_instance= nullptr;

PSMoveService::PSMoveService()
    : m_settings()
{
    PSMoveService::m_instance= this;
}

PSMoveService::~PSMoveService()
{
    PSMoveService::m_instance= nullptr;
}

int PSMoveService::exec(int argc, char *argv[])
{
    // used to select between std:: and boost:: namespaces
    BOOST_APPLICATION_FEATURE_SELECT

    // Parse service options
	boost::program_options::variables_map options_map;
	boost::program_options::options_description desc;

    // Extract the executable name
    std::string exe_name = boost::filesystem::path(argv[0]).stem().string();

    // Define the set of valid command line options
    desc.add_options()
        ("help,h", "Shows help.")
        (",d", "Run as background daemon/service")
        ("log_level,l", boost::program_options::value<std::string>(), "The level of logging to use: trace, debug, info, warning, error, fatal")
        ("admin_password,p", boost::program_options::value<std::string>(), "Remember the admin password for this machine (optional)")
#if defined(BOOST_WINDOWS_API)
        (",i", "install service")
        (",u", "uninstall service")
        (",c", "check service")
        ("name", boost::program_options::value<std::string>()->default_value(exe_name), "service name")
        ("display", boost::program_options::value<std::string>()->default_value("PSMove Service"), "service display name (optional, installation only)")
        ("description", boost::program_options::value<std::string>()->default_value("Manages PSMove controller and broadcasts state to clients"), "service description (optional, installation only)")
#endif // defined(BOOST_WINDOWS_API) 
        ;

    // Parse the command line
    try
    {
        // Validate the command line against the arguments description
		boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), options_map);
        
        // Extract the options that should be stored in the program settings
        parse_program_settings(options_map, this->m_settings);
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
    
    #if defined(BOOST_POSIX_API)
    if (options_map.count("-d"))
    {
        daemonize();
    }
    #endif // defined(BOOST_POSIX_API)

    // initialize logging system
    log_init(this->getProgramSettings()->log_level, "PSMoveService.log");

    // Start the service app
    SERVER_LOG_INFO("main") << "Starting PSMoveService";
    try
    {
        PSMoveServiceImpl app;
		boost::application::context app_context;
        
        // service aspects
        app_context.insert<boost::application::path>(
            BOOST_APPLICATION_FEATURE_NS_SELECT::make_shared<boost::application::path_default_behaviour>(argc, argv));

        app_context.insert<boost::application::args>(
            BOOST_APPLICATION_FEATURE_NS_SELECT::make_shared<boost::application::args>(argc, argv));
        
        // add termination handler
		boost::application::handler<>::parameter_callback termination_callback
            = boost::bind<bool>(&PSMoveServiceImpl::stop, &app, _1);

        app_context.insert<boost::application::termination_handler>(
            BOOST_APPLICATION_FEATURE_NS_SELECT::make_shared<boost::application::termination_handler_default_behaviour>(termination_callback));

        // To  "pause/resume" works, is required to add the 2 handlers.
#if defined(BOOST_WINDOWS_API) 
        // windows only : add pause handler     
		boost::application::handler<>::parameter_callback pause_callback
            = boost::bind<bool>(&PSMoveServiceImpl::pause, &app, _1);

        app_context.insert<boost::application::pause_handler>(
            BOOST_APPLICATION_FEATURE_NS_SELECT::make_shared<boost::application::pause_handler_default_behaviour>(pause_callback));

        // windows only : add resume handler
		boost::application::handler<>::parameter_callback resume_callback
            = boost::bind<bool>(&PSMoveServiceImpl::resume, &app, _1);

        app_context.insert<boost::application::resume_handler>(
            BOOST_APPLICATION_FEATURE_NS_SELECT::make_shared<boost::application::resume_handler_default_behaviour>(resume_callback));
#endif // defined(BOOST_WINDOWS_API) 

        // my common/server instantiation
#if defined(BOOST_WINDOWS_API)
        if (options_map.count("-d"))
        {
            return boost::application::launch<boost::application::server>(app, app_context);
        }
        else
#endif // defined(BOOST_WINDOWS_API)
        {
            return boost::application::launch<boost::application::common>(app, app_context);
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

	log_dispose();

    return 0;
}
