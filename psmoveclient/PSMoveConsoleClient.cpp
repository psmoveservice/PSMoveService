// -----------------------------------------------------------------------------

// Copyright 2011-2013 Renato Tegon Forti
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying 
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

// client.cpp : Defines the entry program_optionsint for the console application.
//

//#define BOOST_ALL_DYN_LINK
#define BOOST_LIB_DIAGNOSTIC

#include "ClientNetworkManager.h"
#include "ResponseHandler.h"

#include <boost/asio.hpp>
#include <boost/application.hpp>
#include <boost/program_options.hpp>

using namespace boost;

class PSMoveConsoleClient
{
public:
    PSMoveConsoleClient() 
        : response_handler()
        , network_manager("localhost", "9512", response_handler)
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
                std::shared_ptr<application::status> st = context.find<application::status>();

                while (st->state() != application::status::stoped)
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
    bool startup()
    {
        bool success= true;

        // Start listening for client connections
        if (success)
        {
            if (!network_manager.startup())
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
        network_manager.update();
    }

    void shutdown()
    {
        // Close all active network connections
        network_manager.shutdown();
    }

private:
    ResponseHandler response_handler;
    ClientNetworkManager network_manager;
}; // PSMoveConsoleClient class

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

