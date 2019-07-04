#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <algorithm>
#include <realtime_tools/realtime_buffer.h>
#include <memory>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>


namespace my_controller_ns
{
    class MyPositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {

        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
        {
            std::string my_joint;
            if (!n.getParam("joint", my_joint))
            {
                ROS_ERROR("Could not find joint name");
                return false;
            }
            joint_ = hw->getHandle(my_joint);  // throws on failure
            command_ = joint_.getPosition();


            controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "state", 1));

            // Start command subscriber
            loop_count_ = 0;

            std::ifstream file;
            file.open("/home/daniel/manipulator_ws/src/my_controller/params/weights.txt");
            if (!file) {
                ROS_ERROR("Unable to open weights file");
                return false;
            }
            ROS_DEBUG("Loading weights");

            file >> layers;
            for(int i = 0; i < layers; i++) {
                std::string name;
                file >> name;
                std::vector<std::vector<double> > v;
                params[name] = v;
                int rows, columns;
                file >> rows >> columns;
                std::string activation;
                file >> activation;
                activations[name] = activation;

                for(int j = 0; j < rows; j++) {
                    std::vector<double> temp;
                    for(int k = 0; k < columns; k++) {
                        double t;
                        file >> t;
                        temp.push_back(t);
                    }
                    params[name].push_back(temp);
                }   

                file >> name;
                std::vector<std::vector<double> > v1;
                params[name] = v1;
                file >> rows >> columns;
                for(int j = 0; j < rows; j++) {
                    std::vector<double> temp;
                    for(int k = 0; k < columns; k++) {
                        double t;
                        file >> t;
                        temp.push_back(t);
                    }
                    params[name].push_back(temp);
                }
            }

          //  cout << layers << endl;
    
    // std::map<std::string, std::vector<std::vector<double> > >::iterator it;
    // for(it = params.begin(); it != params.end(); it++) {
    //   string name = it->first;
    //   cout << name << endl;
    //   std::vector<std::vector<double> > v = it->second;
    //   int rows = v.size();
    //   int columns = v[0].size();
    //   cout << rows << " " << columns << endl;
    //   for(int i = 0; i < rows; i++) {
    //     for(int j = 0; j < columns; j++) {
    //       cout << v[i][j] << " ";
    //     }
    //     cout << endl;
    //   }
    // }

            sub_command_ = n.subscribe<std_msgs::Float64>("command", 1, &MyPositionController::setCommandCB, this);

            return true;
        }

        void update(const ros::Time& time, const ros::Duration& period)
        {
            double velocity = joint_.getVelocity();
            double position = joint_.getPosition();
            double error = command_ - position;

            std::vector<std::vector<double> > x;
            std::vector<double> v;
            v.push_back(position);
            v.push_back(velocity);
            v.push_back(error);
            v.push_back(command_);
            x.push_back(v);

            for(int i = 0; i < layers; i++) {
                std::string name = "W ";
                name[1] = (char)('0' + (i + 1));
                std::vector<std::vector<double> > W = params[name];
                std::string activation = activations[name];

                name = "b ";
                name[1] = (char)('0' + (i + 1));
                std::vector<std::vector<double> > b = params[name];

                std::vector<std::vector<double> > temp;
                multiply(x, W, temp);
                add(temp, b, x);
                if(activation == "relu") {
                    relu(x);
                }
            }
            ROS_INFO("Command : %lf, Target: %lf, Current Position: %lf, Current Velocity: %lf, Error: %lf", x[0][0], 
                        command_, position, velocity, error);

            joint_.setCommand(x[0][0]);



                if(controller_state_publisher_ && controller_state_publisher_->trylock())
                {
                    controller_state_publisher_->msg_.header.stamp = time;
                    controller_state_publisher_->msg_.set_point = command_;
                    controller_state_publisher_->msg_.process_value = position;
                    controller_state_publisher_->msg_.process_value_dot = joint_.getVelocity();
                    controller_state_publisher_->msg_.error = error;
                    controller_state_publisher_->msg_.time_step = period.toSec();
                    controller_state_publisher_->msg_.command = x[0][0];

                    controller_state_publisher_->msg_.p = 0.0;
                    controller_state_publisher_->msg_.i = 0.0;
                    controller_state_publisher_->msg_.d = 0.0;
                    controller_state_publisher_->msg_.i_clamp = 0.0;
                    controller_state_publisher_->msg_.antiwindup = false;
                    controller_state_publisher_->unlockAndPublish();
                }
            

        }

        void setCommandCB(const std_msgs::Float64ConstPtr& msg)
        {
            command_ = msg->data;
        }

        void starting(const ros::Time& time) { }
        void stopping(const ros::Time& time) { }

        private:
            hardware_interface::JointHandle joint_;
            double gain_;
            double command_;
            ros::Subscriber sub_command_;
            std::map<std::string, std::vector<std::vector<double> > > params;
            std::map<std::string, std::string> activations;
            int layers;
            std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::JointControllerState> > controller_state_publisher_;
            long long loop_count_;


            void multiply(std::vector<std::vector<double> > a, std::vector<std::vector<double> > b, std::vector<std::vector<double> > &c) {
                c.clear();
                int rows = a.size();
                int columns = b[0].size();
                int dim = b.size();
                for(int i = 0; i < rows; i++) {
                    std::vector<double> v;
                    for(int j = 0; j < columns; j++) {
                        double temp = 0.0;
                        for(int k = 0; k < dim; k++) {
                            temp += a[i][k] * b[k][j];
                        }
                        v.push_back(temp);
                    }
                    c.push_back(v);
                }
            }

                void add(std::vector<std::vector<double> > a, std::vector<std::vector<double> > b, std::vector<std::vector<double> > &c) {
                    c.clear();
                    int rows = a.size();
                    int columns = a[0].size();
                    for(int i = 0; i < rows; i++) {
                        std::vector<double> temp;
                        for(int j = 0; j < columns; j++) {
                            temp.push_back(a[i][j] + b[i][j]);
                        }
                        c.push_back(temp);
                    }
                }

                void relu(std::vector<std::vector<double> > &a) {
                    int rows = a.size();
                    int columns = a[0].size();
                    for(int i = 0; i < rows; i++) {
                        for(int j = 0; j < columns; j++) {
                            a[i][j] = std::max(0.0, a[i][j]);
                        }
                    }
                }

            
    };

    PLUGINLIB_EXPORT_CLASS(my_controller_ns::MyPositionController, controller_interface::ControllerBase);
}