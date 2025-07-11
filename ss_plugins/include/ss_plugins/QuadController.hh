#pragma once

/*
    PreUpdate:
        - Best for applying control commands before physics update.
        - Allows modifying forces, torques, and velocities before the simulation step.
        - Example: PID controller adjustments for UAV stabilization.

    PostUpdate:
        - Used for logging or visualization after physics update.
        - Cannot modify simulation state.
*/

#include <gz/sim/System.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/empty.pb.h>

#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointVelocityCmd.hh>

#include <gz/math/PID.hh>


typedef enum{
    MOTOR_RR = 0,   //Motor 1 (CW)
    MOTOR_FR,   //Motor 2 (CCW)
    MOTOR_RL,   //Motor 3 (CCW)
    MOTOR_FL    //Motor 4 (CW)
}MotorId;

typedef enum{
    LANDED = 1, //UAV has landed
    LANDING,    //UAV is landing
    TAKING_OFF, //UAV is taking off
    FLYING  //UAV is flying and ready to receive velocity commands
}FlightState;


class QuadControllerPrivate{
    public:
        std::string plugin_name = "[QuadController] ";  //Plugin name for debug purpose
        std::string robot_namespace = "";   //Default namespace used. It will be changed to drone1 for instance
        std::string robot_base_frame = "base_footprint";    //Default UAV's canonical link name

        std::mutex mutex;   //Mutex used to lock some blocks to safely update variables
        std::chrono::steady_clock::duration last_controller_update_time{0}; //Last controller update time 

        gz::transport::Node node;   //Gazebo communication node
        
        gz::sim::Entity canonical_link_entity = gz::sim::kNullEntity;   //Initialize canonical link entity
        gz::sim::Entity model_entity = gz::sim::kNullEntity;    //Initialize model entity
        std::vector<gz::sim::Entity> motor_joint_entities;  //Initialize a list that will contain the entities of motor joints 

        std::vector<std::string> motor_joint_names = {"rotor_RR_joint", "rotor_FR_joint", "rotor_RL_joint", "rotor_FL_joint"};  //Default motor joint names
        double speed_test = 0.0;    //Debug purpose
        float z_take_off = 1.0; //Altitude the drone must exceed in order to be considered ready to fly
        float z_land = 0.4; //Altitude below which the drone is considered to have landed
        float multiplier = 1.0; //Multiplier used to convert a desired force (a motor should produce) into a desired motor's rotational speed

        //Dynamics
        double mass;    //UAV's mass
        gz::math::Vector3d inertia; //Diagonal part of the inertia matrix (Ixx, Iyy, Izz)
        gz::math::Pose3d pose;  //Pose of the model in inertial frame
        gz::math::Vector3d linear_velocity;  //Linear velocity expressed in body frame
        gz::math::Vector3d angular_velocity; //Angular velocity expressed in body frame
        gz::math::Vector3d gravity; //Gravity vector expressed in inertial frame
        gz::math::Vector3d body_force; //Body force expressed in body frame
        gz::math::Vector3d body_torque; //Body torque expressed in body frame
        double F_FL, F_FR, F_RL, F_RR; //Force applied to each rotor

        //Control
        uint8_t navi_state = LANDED;    //UAV's navigation state
        std::vector<double> motor_cmds = {0.0, 0.0, 0.0, 0.0}; //Motor commands for each rotor
        gz::math::PID pid_u, pid_v, pid_w, pid_p, pid_q, pid_r; //PID controllers
        float target_u, target_v, target_w; //Target linear velocities in the body frame
        float target_p, target_q, target_r; //Target angular velocities in the body frame
        float target_roll, target_pitch; //Target roll and pitch angles
        float roll, pitch;  //Current roll and pitch angles

        //Others
        bool ready = false; //Whether the UAV is ready to fly

        void cmd_vel_cb(const gz::msgs::Twist &_msg);
        void take_off_cb(const gz::msgs::Empty &_msg);
        void land_cb(const gz::msgs::Empty &_msg);
        void get_updates(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm);
        void load_params(const std::shared_ptr<const sdf::Element> &_sdf);
        uint8_t get_joint_entities(gz::sim::EntityComponentManager &_ecm);
        void check_state();
        void attitude_control(const double _dt);
        void mixer();
        void apply_motor_forces(const double _dt, gz::sim::EntityComponentManager &_ecm);
        void stabilization(const double _dt, gz::sim::EntityComponentManager &_ecm);

};

class QuadController:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
    public:
        QuadController();
        ~QuadController() override = default;

        /*Plugins inheriting ISystemConfigure must implement the Configure 
        callback. This is called when a system is initially loaded. 
        The _entity variable contains the entity that the system is attached to
        The _element variable contains the sdf Element with custom configuration
        The _ecm provides an interface to all entities and components
        The _eventManager provides a mechanism for registering internal signals*/
        void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
            gz::sim::EntityComponentManager &_ecm,
            gz::sim::EventManager &_eventManager) override;
        
        /*Plugins inheriting ISystemPreUpdate must implement the PreUpdate
        callback. This is called at every simulation iteration before the physics
        updates the world. The _info variable provides information such as time,
        while the _ecm provides an interface to all entities and components in
        simulation.*/
        void PreUpdate(const gz::sim::UpdateInfo &_info,
            gz::sim::EntityComponentManager &_ecm) override;
        
        /*Plugins inheriting ISystemPostUpdate must implement the PostUpdate
        callback. This is called at every simulation iteration after the physics
        updates the world. The _info variable provides information such as time,
        while the _ecm provides an interface to all entities and components in
        simulation.*/
        void PostUpdate(const gz::sim::UpdateInfo &_info,
            const gz::sim::EntityComponentManager &_ecm) override;
    
    private:
        std::unique_ptr<QuadControllerPrivate> dataPtr;
};
