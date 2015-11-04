//-- includes -----
//#define BOOST_ALL_DYN_LINK
#define BOOST_LIB_DIAGNOSTIC

#include "ServerNetworkManager.h"
#include "RequestHandler.h"

#include <boost/asio.hpp>
#include <boost/application.hpp>
#include <boost/program_options.hpp>
#include <fstream>
#include <cstdio>

using namespace boost;

//-- constants -----
const int PSMOVE_SERVER_PORT = 9512;

//-- definitions -----
class PSMoveService
{
public:
    PSMoveService() 
        : config_path()
        , request_handler()
        , network_manager(PSMOVE_SERVER_PORT, request_handler)
    {
    }

    int operator()(application::context& context)
    {
        BOOST_APPLICATION_FEATURE_SELECT

        // Attempt to start and run the service
        try 
        {
            if (startup())
            {
                std::shared_ptr<application::status> st = context.find<application::status>();

                while (st->state() != application::status::stoped)
                {
                    update();

                    boost::this_thread::sleep(boost::posix_time::seconds(1));
                }
            }
            else
            {
                std::cerr << "Failed to startup the PSMove service" << std::endl;
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
        return true;
    }

    bool pause(application::context& context)
    {
        return true;
    }

    bool resume(application::context& context)
    {
        return true;
    }

private:
    bool startup()
    {
        bool success= true;

        // Start listening for client connections
        if (success)
        {
            if (!network_manager.startup())
            {
                std::cerr << "Failed to initialize the service network manager" << std::endl;
                success= false;
            }
        }

        return success;
    }

    void update()
    {
        // Process incoming/outgoing networking requests
        network_manager.update();
    }

    void shutdown()
    {
        // Close all active network connections
        network_manager.shutdown();
    }

private:
    filesystem::path config_path;
    RequestHandler request_handler;
    ServerNetworkManager network_manager;
};

//-- Entry program_optionsint ---
int main(int argc, char *argv[])
{
    // used to select between std:: and boost:: namespaces
    BOOST_APPLICATION_FEATURE_SELECT

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
#endif     

        // my common/server instantiation

        program_options::variables_map vm;
        program_options::options_description desc;

        desc.add_options()
            (",h", "Shows help.")
            (",f", "Run as common application")
            ("help", "produce a help message")
            ;

        program_options::store(program_options::parse_command_line(argc, argv, desc), vm);

        if (vm.count("-h"))
        {
            std::cout << desc << std::endl;
            return 0;
        }

        if (vm.count("-f"))
        {
            return application::launch<application::common>(app, app_context);
        }
        else
        {
            return application::launch<application::server>(app, app_context);
        }
    }
    catch (boost::system::system_error& se)
    {
        std::cerr << se.what() << std::endl;
        return 1;
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unknown error." << std::endl;
        return 1;
    }

    return 0;
}
