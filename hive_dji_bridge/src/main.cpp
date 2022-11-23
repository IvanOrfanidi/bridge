#include <future>
#include <thread>

#include <boost/program_options.hpp>

#include <common/make_string.hpp>
#include <application/handler.hpp>
#include <application/config.hpp>
#include <application/logging.hpp>
#include <communication/communication.hpp>

void parseInputArguments(int argc, char** argv);
void bindHandlerWithCommunication(Handler&, const Communication&);

int main(int argc, char** argv) {
    parseInputArguments(argc, argv);

    static constexpr std::string_view NODE_NAME = "hive_dji_bridge";
    Logging::registerNode(NODE_NAME.data());

    ros::init(argc, argv, NODE_NAME.data());

    Handler& handler = Handler::instance();

    Config& config = Config::instance();
    Logging::informationMessage(drone::utils::make_string() << "Config: " << config, __FUNCTION__);

    Communication& communication = Communication::instance(config.gatewayUrl());
    bindHandlerWithCommunication(handler, communication);

    auto commandQueue = std::make_shared<drone::multithread::queue<std::shared_ptr<Command>>>();
    handler.attachCommandQueue(commandQueue);
    communication.attachCommandQueue(commandQueue);

    std::promise<void> signalExit;
    std::shared_future<void> exitFuture(signalExit.get_future());
    communication.start(exitFuture);
    handler.start(exitFuture);

    bool run = true;
    while (run && ros::ok()) {
        ros::spinOnce();
        ros::Duration(1).sleep();
    }

    signalExit.set_value();
    ros::shutdown();
}

void parseInputArguments(int argc, char** argv) {
    boost::program_options::options_description desc("Options");
    desc.add_options()
      // Options:
      ("version,v", "Print version information.") // Output current version
      ("help,h", "Display available options.");   // Output help
    boost::program_options::variables_map options;
    try {
        boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), options);
        boost::program_options::notify(options);
    } catch (const std::exception& exception) {
        std::cerr << "error: " << exception.what() << std::endl;
        exit(EXIT_FAILURE);
    }

    if (options.count("help")) {
        std::cout << desc << std::endl;
        exit(EXIT_SUCCESS);
    }
    if (options.count("version")) {
        std::cout << drone::utils::make_string() << "version: " << hive_dji_bridge_version << std::endl;
        exit(EXIT_SUCCESS);
    }
}

void bindHandlerWithCommunication(Handler& handler, const Communication& communication) {
    for (const auto& subscriber : communication.getSubscribersData()) {
        handler.attachSubscriber(subscriber);
    }
}
