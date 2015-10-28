//-- includes -----
//#define BOOST_ALL_DYN_LINK
#define BOOST_LIB_DIAGNOSTIC

#include <boost/asio.hpp>

#include <fstream>

#include <boost/application.hpp>
#include <boost/program_options.hpp>

using namespace boost;

//-- definitions -----
class PSMoveService
{
public:
    int operator()(application::context& context)
    {
        BOOST_APPLICATION_FEATURE_SELECT

        //TODO: Remove thread and replace with this once I switch over to async io
        //boost::shared_ptr<application::status> st = context.find<application::status>();

        //while (st->state() != application::status::stoped)
        //{
        //    // application logic here
        //    boost::this_thread::sleep(boost::posix_time::seconds(1));
        //}

        // launch a work thread
        boost::thread thread(boost::bind(&PSMoveService::work_thread, this, &context));

        context.find<application::wait_for_termination_request>()->wait();
        thread.join(); // the last connection need be served to app exit, comment this to exit quick

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

protected:

    void work_thread(application::context* context)
    {
        BOOST_APPLICATION_FEATURE_SELECT

        using boost::asio::ip::tcp;

        shared_ptr<application::status> st = context->find<application::status>();

        try
        {
            boost::asio::io_service io_service;
            tcp::acceptor acceptor(io_service, tcp::endpoint(tcp::v4(), 9512));

            for (;;)
            {
                if (st->state() == application::status::stoped)
                {
                    return;
                }

                boost::system::error_code error;

                tcp::socket socket(io_service);
                acceptor.accept(socket);

                // our data is limited to 1024 bytes
                char data[1024];

                size_t length = socket.read_some(boost::asio::buffer(data), error);

                if (error == boost::asio::error::eof)
                    break; // Connection closed cleanly by peer.
                else if (error)
                    throw boost::system::system_error(error); // Some other error.

                // resprogram_optionsnse (echo)
                std::string message;

                // detect pause state
                if (st->state() == application::status::paused)
                {
                    message = "PSMoveService is paused, try again later!";
                }
                else
                {
                    // echo
                    message = std::string(data, length);
                }

                boost::asio::write(
                    socket, boost::asio::buffer(message),
                    boost::asio::transfer_all(),
                    error);
            }
        }
        catch (std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }

private:
    filesystem::path config_path;
};

//-- Entry program_optionsint ---
int main(int argc, char *argv[])
{
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
