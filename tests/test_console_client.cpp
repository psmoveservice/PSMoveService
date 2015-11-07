//#define BOOST_ALL_DYN_LINK
#define BOOST_LIB_DIAGNOSTIC

#include "ClientPSMoveAPI.h"
#include "ClientControllerView.h"
#include "PSMoveDataFrame.pb.h"
#include <boost/asio.hpp>
#include <boost/application.hpp>
#include <boost/program_options.hpp>

using namespace boost;

class PSMoveConsoleClient
{
public:
    PSMoveConsoleClient() 
        : app_status()
    {
    }

    int operator()(application::context& context)
    {
        BOOST_APPLICATION_FEATURE_SELECT

        // Attempt to start and run the client
        try 
        {
            if (startup())
            {
                app_status = context.find<application::status>();

                while (app_status->state() != application::status::stoped)
                {
                    update();

                    boost::this_thread::sleep(boost::posix_time::seconds(1));
                }
            }
            else
            {
                std::cerr << "Failed to startup the PSMove Client" << std::endl;
            }
        }
        catch (std::exception& e) 
        {
            std::cerr << e.what() << std::endl;
        }

        // Attempt to shutdown the client
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

private:
    // ClientPSMoveAPI
    void handle_client_psmove_event(ClientPSMoveAPI::eClientPSMoveAPIEvent event_type)
    {
        switch (event_type)
        {
        case ClientPSMoveAPI::connectedToService:
            std::cout << "PSMoveConsoleClient - Connected to service" << std::endl;

            // Kick off request to get psmove controller count
            ClientPSMoveAPI::get_controller_count(
                boost::bind(&PSMoveConsoleClient::handle_get_controller_count, this, _1, _2));
            break;
        case ClientPSMoveAPI::failedToConnectToService:
            std::cout << "PSMoveConsoleClient - Failed to connect to service" << std::endl;
            app_status->state(application::status::stoped);
            break;
        case ClientPSMoveAPI::disconnectedFromService:
            std::cout << "PSMoveConsoleClient - Disconnected from service" << std::endl;
            app_status->state(application::status::stoped);
            break;
        default:
            break;
        }
    }

    void handle_get_controller_count(RequestPtr request, ResponsePtr response)
    {
        if (response->has_response_psmove_count())
        {
            int count= response->response_psmove_count().count();

            if (count > 0)
            {
                std::cout << "PSMoveConsoleClient - controllers available: " << count << std::endl;
                ClientPSMoveAPI::start_controller_data_stream(
                    0, 
                    boost::bind(&PSMoveConsoleClient::handle_acquire_controller, this, _1, _2));
            }
            else
            {
                std::cerr << "PSMoveConsoleClient - no controllers available" << std::endl;
                app_status->state(application::status::stoped);
            }
        }
        else
        {
            std::cerr << "PSMoveConsoleClient - failed to get controller count" << std::endl;
            app_status->state(application::status::stoped);
        }
    }

    void handle_acquire_controller(RequestPtr request, ResponsePtr response)
    {
        if (response->result_code() == PSMoveDataFrame::Response_ResultCode_RESULT_OK)
        {
            std::cout << "PSMoveConsoleClient - Acquired controller 0" << std::endl;

            // Once created, updates will automatically get pushed into this view
            controller_view= ClientPSMoveAPI::allocate_controller_view(0);
        }
        else
        {
            std::cerr << "PSMoveConsoleClient - failed to acquire controller 0" << std::endl;
            app_status->state(application::status::stoped);
        }
    }

    // PSMoveConsoleClient
    bool startup()
    {
        bool success= true;

        // Attempt to connect to the server
        if (success)
        {
            if (!ClientPSMoveAPI::startup(
                    "localhost", "9512", 
                    boost::bind(&PSMoveConsoleClient::handle_client_psmove_event, this, _1)))
            {
                std::cerr << "Failed to initialize the client network manager" << std::endl;
                success= false;
            }
        }

        return success;
    }

    void update()
    {
        // Process incoming/outgoing networking requests
        ClientPSMoveAPI::update();
    }

    void shutdown()
    {
        // Free any allocated controller views
        if (controller_view)
        {
            ClientPSMoveAPI::free_controller_view(controller_view);
            controller_view.reset();
        }

        // Close all active network connections
        ClientPSMoveAPI::shutdown();
    }

private:
    std::shared_ptr<application::status> app_status;
    ClientControllerViewPtr controller_view;
};

int main(int argc, char *argv[])
{   
   BOOST_APPLICATION_FEATURE_SELECT

    try 
    {
        PSMoveConsoleClient app;
        application::context app_context;

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

        // app instantiation
        return application::launch<application::common>(app, app_context);
    }
    catch(boost::system::system_error& se)
    {
        std::cerr << se.what() << std::endl;
        return 1;
    }
}

