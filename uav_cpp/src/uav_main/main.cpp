#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "PID.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"



using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;
const double STEP_SIZE_DEG = 3.5e-5;;
const double TOLERANCE = 0.1;  // Tolerance in meters
void usage(const std::string& bin_name)
{
    std::cerr << "Usage : " << bin_name << " <connection_url>\n"
              << "Connection URL format should be :\n"
              << " For TCP : tcp://[server_host][:server_port]\n"
              << " For UDP : udp://[bind_host][:bind_port]\n"
              << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
              << "For example, to connect to the simulator use URL: udpin://0.0.0.0:14540\n";
}




bool offb_ctrl_pos_global(mavsdk::Offboard& offboard, mavsdk::Telemetry& telemetry)
{
double target_x = 0;
double target_y = 0;
double target_z = 0;

double X, Y, Z;  // Variables to store the values

    // Open the file for reading
    std::ifstream file("/root/python-files/coordinates.txt");

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return EXIT_FAILURE;
    }

    // Read the single line from the file
    std::string line;
    if (std::getline(file, line)) {
        // Use a stringstream to parse the values
        std::stringstream ss(line);
        char discard; // To discard characters like '[' and ']'

        // Extract values, ignoring the brackets
        ss >> discard >> X >> Y >> Z >> discard;
    } else {
        std::cerr << "Error reading line from file." << std::endl;
        file.close();
        return EXIT_FAILURE;
    }

    // Close the file
    file.close();

    // Print the values
    std::cout << "X: " << X << std::endl;
    std::cout << "Y: " << Y << std::endl;
    std::cout << "Z: " << Z << std::endl;
    std::cout << "Reading home position in Global coordinates\n";

    const auto res_and_gps_origin = telemetry.get_gps_global_origin();
    if (res_and_gps_origin.first != Telemetry::Result::Success) {
        std::cerr << "Telemetry failed: " << res_and_gps_origin.first << '\n';
    }
    Telemetry::GpsGlobalOrigin origin = res_and_gps_origin.second;
    std::cerr << "Origin (lat, lon, alt amsl):\n " << origin << '\n';

    std::cout << "Starting Offboard position control in Global coordinates\n";

        sleep_for(seconds(10));
        double latit = origin.latitude_deg;
        double lon = origin.longitude_deg;
//New code
        PID pid_x(1.0, 0.1, 0.01);  // Adjust PID gains as necessary
        PID pid_y(1.0, 0.1, 0.01);

        double current_x = X;
        double current_y = Y;


     while (true) {
     double X, Y, Z;  // Variables to store the values

    // Open the file for reading
    std::ifstream file("/root/python-files/coordinates.txt");

    // Check if the file is open
    if (!file.is_open()) {
        std::cerr << "Error opening file" << std::endl;
        return EXIT_FAILURE;
    }

    // Read the single line from the file
    std::string line;
    if (std::getline(file, line)) {
        // Use a stringstream to parse the values
        std::stringstream ss(line);
        char discard; // To discard characters like '[' and ']'

        // Extract values, ignoring the brackets
        ss >> discard >> X >> Y >> Z >> discard;
    } else {
        std::cerr << "Error reading line from file." << std::endl;
        file.close();
        return EXIT_FAILURE;
    }

    // Close the file
    file.close();

    // Print the values
    std::cout << "X: " << X << std::endl;
    std::cout << "Y: " << Y << std::endl;
    std::cout << "Z: " << Z << std::endl;

      double current_x = X;
    double current_y = Y;
         const auto res_and_gps_origin = telemetry.get_gps_global_origin();
    if (res_and_gps_origin.first != Telemetry::Result::Success) {
        std::cerr << "Telemetry failed: " << res_and_gps_origin.first << '\n';
                continue;

    }
    Telemetry::GpsGlobalOrigin origin = res_and_gps_origin.second;
    std::cerr << "Origin (lat, lon, alt amsl):\n " << origin << '\n';
        // Calculate if we are within the target tolerance
        if (std::fabs(target_x - current_x) < (TOLERANCE) && std::fabs(target_y - current_y) < TOLERANCE) {
            std::cout << "Target position reached within tolerance.\n";
            offboard.stop();
            return true;
        }

        // Calculate new positions using PID adjustments
        double x_adjust = pid_x.calculate(target_x, current_x) * STEP_SIZE_DEG;
        double y_adjust = pid_y.calculate(target_y, current_y) * STEP_SIZE_DEG;
latit=origin.latitude_deg+ x_adjust;
lon=origin.longitude_deg+y_adjust;
   const Offboard::PositionGlobalYaw new_position{
        latit,
        lon,
        2.5f,
        0.0f};
        offboard.set_position_global(new_position);
//Start offbooard commands
    Offboard::Result offboard_result = offboard.start();

        sleep_for(seconds(2));

        std::cout << "Adjusting position to X: " << ( x_adjust)
                  << " Y: " << ( y_adjust) << '\n';

        // Delay for smoother command sending
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    sleep_for(seconds(10));

    std::cout << "Offboard stopped\n";

    return true;
}

// after receiving the message
void command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    std::cout << "Received command: " << msg->data << std::endl;
        std::cout << "__________________________________________________Starting offboard control... " << msg->data << std::endl;

    if (msg->data == 1 || msg->data == 2) {  // Assuming 1 is the command to initiate offboard control

   
        std::cout << "Starting takeoff... " << msg->data << std::endl;
            Mavsdk mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};


            ConnectionResult connection_result = mavsdk.add_any_connection("udp://:14540");

             auto system = mavsdk.first_autopilot(3.0);
             if (!system) {
                    std::cerr << "Timed out waiting for system\n";
                    return;
                }

                // Instantiate plugins.
        auto action = Action{system.value()};
        auto offboard = Offboard{system.value()};
        auto telemetry = Telemetry{system.value()};
        
    const auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return;
    }
    std::cout << "Armed\n";

    const auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return;
    }

    auto in_air_promise = std::promise<void>{};
    auto in_air_future = in_air_promise.get_future();
    Telemetry::LandedStateHandle handle = telemetry.subscribe_landed_state(
        [&telemetry, &in_air_promise, &handle](Telemetry::LandedState state) {
            if (state == Telemetry::LandedState::InAir) {
                std::cout << "Taking off has finished\n.";
                telemetry.unsubscribe_landed_state(handle);
                in_air_promise.set_value();
            }
        });
    in_air_future.wait_for(seconds(10));
    if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "Takeoff timed out.\n";
        return;
    }

    if(msg->data == 2){    
      if (!offb_ctrl_pos_global(offboard, telemetry)) {
        return ;
   }
       const auto land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Landing failed: " << land_result << '\n';
        return;
    }

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

   }
      }
    
    
    }


int main(int argc, char** argv)
{
 
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

        rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mavsdk_offboard_node");

    // Create a subscriber for uav_commands topic
    auto subscription = node->create_subscription<std_msgs::msg::Int32>(
        "uav_commands", 10, command_callback);


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
