#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include "quanser/quanser_messages.h"
#include "quanser/quanser_memory.h"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cmath>
#include <thread>

#include "quanser/quanser_hid.h"
#include "qcar2_interfaces/msg/boolean_leds.hpp"
#include "qcar2_interfaces/msg/motor_commands.hpp"


using namespace std::chrono_literals;



class Nav2QCarConverter : public rclcpp::Node
{


    public:
    Nav2QCarConverter()
    : Node("nav2_qcar2_command_converter")
    {
    // ── Ackermann parameters ────────────────────────────────────────
    this->declare_parameter("wheelbase", 0.256);       // QCar2 wheelbase (m)
    this->declare_parameter("max_steer_rad", 0.9);     // Physical servo limit (rad)
    // Steering bias: positive = corrige drift a la izquierda (empuja a la derecha)
    //                negative = corrige drift a la derecha (empuja a la izquierda)
    // Valores típicos: 0.03 a 0.09 rad
    this->declare_parameter("steer_bias", -0.3);
    wheelbase_     = this->get_parameter("wheelbase").as_double();
    max_steer_rad_ = this->get_parameter("max_steer_rad").as_double();
    steer_bias_    = this->get_parameter("steer_bias").as_double();

    RCLCPP_INFO(this->get_logger(),
        "Ackermann converter: wheelbase=%.3fm, max_steer=%.2f rad (%.1f deg), steer_bias=%.3f rad",
        wheelbase_, max_steer_rad_, max_steer_rad_ * 180.0 / M_PI, steer_bias_);

    // configuring command publisher
    command_publisher_  = this->create_publisher<qcar2_interfaces::msg::MotorCommands>("qcar2_motor_speed_cmd", 1);
    // led_publisher_      = this->create_publisher<qcar2_interfaces::msg::BooleanLeds>("qcar2_led_cmd",10);

    //configure nav2 subscriber
    nav2_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav",1,std::bind(&Nav2QCarConverter::nav2_command_callback, this, std::placeholders::_1));
    
    //publishing timer for converted command
    timer_ = this->create_wall_timer(1ms, std::bind(&Nav2QCarConverter::command_plublish, this));

    //publishing timer for converted command
    timer2_ = this->create_wall_timer(33ms, std::bind(&Nav2QCarConverter::led_publish, this));



    }

    private:       
        void nav2_command_callback(const geometry_msgs::msg::Twist &nav2_commands){
            // Nav2 usa convención ROS estándar:
            //   linear.x  = velocidad lineal (avance/retroceso)
            //   angular.z = velocidad angular (giro)
            double raw_speed = nav2_commands.linear.x;
            double wz = nav2_commands.angular.z;

            // Delimitar velocidad lineal a 0.2 m/s
            if (raw_speed > 0.2) nav2_speed = 0.2;
            else if (raw_speed < -0.2) nav2_speed = -0.2;
            else nav2_speed = raw_speed;

            // ── Ackermann conversion: angular.z (rad/s) → steering angle (rad) ──
            // Formula: steering_angle = atan(wheelbase * wz / vx)
            //
            // SIGN CONVENTION (MATCHED — no negation needed):
            //   ROS:   angular.z > 0 = girar IZQUIERDA (CCW)
            //   QCar2: steering  > 0 = girar IZQUIERDA
            //   QCar2: steering  < 0 = girar DERECHA
            double vx = nav2_speed;

            if (std::abs(vx) > 0.01) {
                nav2_steering = std::atan(wheelbase_ * wz / vx);
            } else {
                nav2_steering = wheelbase_ * wz;
            }

            // ── Apply steering bias to correct drift ──
            // Drift a la izquierda → steer_bias positivo (empuja derecha)
            nav2_steering += steer_bias_;

            // ── Clamp to physical servo range [-max_steer, +max_steer] ──
            if (nav2_steering >  max_steer_rad_) nav2_steering =  max_steer_rad_;
            if (nav2_steering < -max_steer_rad_) nav2_steering = -max_steer_rad_;
        }


        void command_plublish(){
        
            //configure publisher for LEDS and motor commands:
            // Populate motor command  message for velocity and steering
            qcar2_interfaces::msg::MotorCommands motor_command;

            // // Create a string array for names, and double array for values 
            std::vector<std::string> name;
            std::vector<t_double> val;

            name.push_back("steering_angle");
            name.push_back("motor_throttle");

            // El hardware lee los strings, pero están cruzados internamente.
            // 'steering_angle' controla físicamente el motor (velocidad).
            // 'motor_throttle' controla físicamente el servo (dirección).
            val.push_back(nav2_steering);       // Asignado a "steering_angle"
            val.push_back(nav2_speed);    // Asignado a "motor_throttle"
                                                            
            motor_command.motor_names = name;
            motor_command.values = val;


            this->command_publisher_->publish(motor_command);


        }



        void led_publish(){
            
            
            
            if (nav2_speed !=0) {
                //set LEDs for QCar moving
                led_values[8]= 1;
                led_values[9]= 1;
                led_values[10]= 1;
                led_values[11]= 1;
                led_values[12]= 1;
                led_values[13]= 1;

                if(nav2_steering > 0.01){
                    led_values[14]= 1;
                    led_values[6]= 1;

                }
                else if(nav2_steering <-0.01){
                    led_values[15]= 1;
                    led_values[7]= 1;

                }

            }
            else if (nav2_speed == 0 ){
                led_values[0] = 1;
                led_values[1] = 1;
                led_values[2] = 1;
                led_values[3] = 1;
            }          
            
            
            // Populate LED commands 
            qcar2_interfaces::msg::BooleanLeds led_commands;

            std::vector<std::string> led_name;
            std::vector<bool> led_value_commands;

            
            
            led_name.push_back("left_outside_brake_light");
            led_name.push_back("left_inside_brake_light");
            led_name.push_back("right_inside_brake_light");    
            led_name.push_back("right_outside_brake_light");   
            led_name.push_back("left_reverse_light");          
            led_name.push_back("right_reverse_light");         
            led_name.push_back("left_rear_signal");            
            led_name.push_back("right_rear_signal");           
            led_name.push_back("left_outside_headlight");      
            led_name.push_back("left_middle_headlight");       
            led_name.push_back("left_inside_headlight");       
            led_name.push_back("right_inside_headlight");      
            led_name.push_back("right_middle_headlight");      
            led_name.push_back("right_outside_headlight");     
            led_name.push_back("left_front_signal");           
            led_name.push_back("right_front_signal");

            for(bool index: led_values)
            {
                led_value_commands.push_back(index);
            }

            led_commands.led_names = led_name;
            led_commands.values = led_value_commands;
            // this->led_publisher_->publish(led_commands);
         }


        bool led_values[16] = {0};

        double nav2_speed = 0;
        double nav2_steering = 0;
        double wheelbase_ = 0.256;
        double max_steer_rad_ = 0.7;
        double steer_bias_ = 0.05;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr timer2_;


        rclcpp::Publisher<qcar2_interfaces::msg::MotorCommands>::SharedPtr command_publisher_;
        rclcpp::Publisher<qcar2_interfaces::msg::BooleanLeds>::SharedPtr led_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav2_subscriber_;
        
};


int main(int argc, char ** argv)
{

    // Node creation
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2QCarConverter>());
    rclcpp::shutdown();

    return 0;
}
