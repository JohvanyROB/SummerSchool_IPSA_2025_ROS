/**
 * @file QuadController.cc
 *
 * @brief This is where the controller of the quadcopter is implemented and used as a Gazebo plugin
 *
 * @ingroup sm_plugins
 *
 * @author Johvany Gustave
 * Contact: johvany.gustave@ipsa.fr
 *
 */

#include "ss_plugins/QuadController.hh"


/**
 * @brief Class's constructor
 */
QuadController::QuadController(): dataPtr(std::make_unique<QuadControllerPrivate>())
{
    // std::cout << std::fixed << std::setprecision(2);    //Two decimal places

    //Setting initial gains for the PID controllers
    this->dataPtr->pid_u.Init(0.1, 0, 0, 10, -10, 1.0, 0.0, 0.0);
    this->dataPtr->pid_v.Init(0.1, 0, 0, 10, -10, 1.0, 0.0, 0.0);
    this->dataPtr->pid_w.Init(0.1, 0, 0, 10, -10, 1.0, 0.0, 0.0);
    this->dataPtr->pid_p.Init(0.1, 0, 0, 10, -10, 1.0, 0.0, 0.0);
    this->dataPtr->pid_q.Init(0.1, 0, 0, 10, -10, 1.0, 0.0, 0.0);
    this->dataPtr->pid_r.Init(0.1, 0, 0, 10, -10, 1.0, 0.0, 0.0);
}


/**
    * @brief Configure callback
    * @details This function is called once, in order to configure the plugin and load user's custom parameters.
    * @param _entity Entity ID
    * @param _sdf SDF element
    * @param _ecm EntityComponentManager
    * @param _eventManager EventManager
*/
void QuadController::Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &_ecm, gz::sim::EventManager &_eventManager){
    if(!_ecm.Component<gz::sim::components::Model>(_entity)){   //Check if the entity is a model
        gzerr << this->dataPtr->plugin_name << "This plugin should be attached to a model entity." << std::endl;
        return;
    }
    this->dataPtr->model_entity = _entity;  //Get model entity ID
    std::cout << this->dataPtr->plugin_name << "Configure on entity " << this->dataPtr->model_entity << std::endl;

    //Load parameters from SDF
    this->dataPtr->load_params(_sdf);

    //Get the base link entity
    this->dataPtr->canonical_link_entity = _ecm.EntityByComponents(gz::sim::components::ParentEntity(this->dataPtr->model_entity), gz::sim::components::CanonicalLink());
    if(this->dataPtr->canonical_link_entity != gz::sim::kNullEntity){
        std::cout << this->dataPtr->plugin_name << "Canonical link entity: " << this->dataPtr->canonical_link_entity << std::endl;
    }
    else{
        gzerr << this->dataPtr->plugin_name << "Canonical link entity not found for model entity: " << this->dataPtr->model_entity << std::endl;
        return;
    }

    //Get the inertial component of the canonical link
    if(this->dataPtr->canonical_link_entity != gz::sim::kNullEntity){        
        const gz::sim::components::Inertial *_inertialComp = _ecm.Component<gz::sim::components::Inertial>(this->dataPtr->canonical_link_entity);
        if(_inertialComp){
            this->dataPtr->mass = _inertialComp->Data().MassMatrix().Mass();
            this->dataPtr->inertia = _inertialComp->Data().MassMatrix().DiagonalMoments();
            std::cout << this->dataPtr->plugin_name << "Mass: " << this->dataPtr->mass << "kg" << std::endl;
            std::cout << this->dataPtr->plugin_name << "Inertia: (" << this->dataPtr->inertia.X() << "," << this->dataPtr->inertia.Y() << "," << this->dataPtr->inertia.Z() << ")" << std::endl;
        }
        else
            gzwarn << this->dataPtr->plugin_name << "Inertial component not found for base link entity: " << this->dataPtr->canonical_link_entity << std::endl;
    }

    //Get joint entity of each motor
    if(!this->dataPtr->get_joint_entities(_ecm))
        return;

    //Get world entity
    gz::sim::Entity world_entity = _ecm.EntityByComponents(gz::sim::components::World());
    if(world_entity == gz::sim::kNullEntity){
        gzerr << this->dataPtr->plugin_name << "World entity not found." << std::endl;
        return;
    }

    //Get gravity component
    gz::sim::components::Gravity *_gravityComp = _ecm.Component<gz::sim::components::Gravity>(world_entity);
    if(_gravityComp){
        this->dataPtr->gravity = _gravityComp->Data();
        std::cout << this->dataPtr->plugin_name << "Gravity: (" << this->dataPtr->gravity.X() << "," << this->dataPtr->gravity.Y() << "," << this->dataPtr->gravity.Z() << ")" << std::endl;
    }
    else{
        gzwarn << this->dataPtr->plugin_name << "Gravity component not found for world entity: " << world_entity << std::endl;
        return;
    }

    //Create (local) linear velocity component for canonical link if it does not exist
    if(!_ecm.Component<gz::sim::components::LinearVelocity>(this->dataPtr->canonical_link_entity)){
        _ecm.CreateComponent(this->dataPtr->canonical_link_entity, gz::sim::components::LinearVelocity());
        std::cout << this->dataPtr->plugin_name << "Created linear velocity component for base link entity: " << this->dataPtr->canonical_link_entity << std::endl;
    }

    //Create (local) angular velocity component for canonical link if it does not exist
    if(!_ecm.Component<gz::sim::components::AngularVelocity>(this->dataPtr->canonical_link_entity)){
        _ecm.CreateComponent(this->dataPtr->canonical_link_entity, gz::sim::components::AngularVelocity());
        std::cout << this->dataPtr->plugin_name << "Created angular velocity component for base link entity: " << this->dataPtr->canonical_link_entity << std::endl;
    }

    //Subscribe to topics
    this->dataPtr->node.Subscribe("/"+this->dataPtr->robot_namespace+"/cmd_vel", &QuadControllerPrivate::cmd_vel_cb, this->dataPtr.get());
    this->dataPtr->node.Subscribe("/"+this->dataPtr->robot_namespace+"/takeoff", &QuadControllerPrivate::take_off_cb, this->dataPtr.get());
    this->dataPtr->node.Subscribe("/"+this->dataPtr->robot_namespace+"/land", &QuadControllerPrivate::land_cb, this->dataPtr.get());
}


/**
    * @brief Pre-update callback
    * @details This is called at every simulation iteration before the physics
        updates the world. The _info variable provides information such as time,
        while the _ecm provides an interface to all entities and components in
        simulation.
    * @param _info UpdateInfo
    * @param _ecm EntityComponentManager
*/
void QuadController::PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm){
    //If simulation is not paused and UAV is ready to fly
    if(_info.paused || !this->dataPtr->ready){
        return;
    }

    //Get elapsed time since last update
    const double dt = std::chrono::duration_cast<std::chrono::duration<double>>(_info.simTime - this->dataPtr->last_controller_update_time).count();
    
    //Check UAV's current navigation state
    this->dataPtr->check_state();
    
    //If the UAV is taking off
    if(this->dataPtr->navi_state == TAKING_OFF){
        /**
         * @todo Add the instruction to update the target linear velocity along z_b axis.
         * w_d corresponds to this->dataPtr->target_w.
         * z_d (target altitude) corresponds to this->dataPtr->z_take_off.
         * z (current altitude) corresponds to this->dataPtr->pose.Pos().Z().
         * Use the square root controller to define w_d as discussed in Lecture 3.
         */
        float error = this->dataPtr->z_take_off - this->dataPtr->pose.Pos().Z();
        float K = 0.1;
        if(error >= 0){
            this->dataPtr->target_w = K * sqrt(abs(error));
        }
        else{
            this->dataPtr->target_w = -K * sqrt(abs(error));
        }
        
    }

    //If the UAV is landing
    else if(this->dataPtr->navi_state == LANDING){
        /**
         * @todo Add the instruction to update the target linear velocity along z_b axis.
         * w_d corresponds to this->dataPtr->target_w.
         * z_d (target altitude) corresponds to 0.
         * z (current altitude) corresponds to this->dataPtr->pose.Pos().Z().
         * Use the square root controller to define w_d as discussed in Lecture 3.
         */

        float error = 0 - this->dataPtr->pose.Pos().Z();
        float K = 0.1;
        if(error >= 0){
            this->dataPtr->target_w = K * sqrt(abs(error));
        }
        else{
            this->dataPtr->target_w = -K * sqrt(abs(error));
        }
    }
    
    //If the UAV is not in LANDED mode
    if(this->dataPtr->navi_state != LANDED)
        this->dataPtr->stabilization(dt, _ecm); //perform its stabilization
}


/**
    * @brief Post-update callback
    * @details This is called at every simulation iteration after the physics
        updates the world. The _info variable provides information such as time,
        while the _ecm provides an interface to all entities and components in
        simulation.
    * @param _info UpdateInfo
    * @param _ecm EntityComponentManager
*/
void QuadController::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm){
    //If the simulation is not paused
    if(_info.paused){
        return;
    }

    //Get updates from the simulation, after physics has been applied to the world
    this->dataPtr->get_updates(_info, _ecm);

    //In case you want to print data each 250ms (regarding the simulation time not real time: depends on laptop's performance)
    if (_info.iterations % 250 == 0){
        // std::cout << this->dataPtr->plugin_name << "Body force: " << this->dataPtr->body_force.Z() << "N" << std::endl;
        // std::cout << this->dataPtr->plugin_name << "FL: " << this->dataPtr->F_FL << std::endl;
        // std::cout << this->dataPtr->plugin_name << "FR: " << this->dataPtr->F_FR << std::endl;
        // std::cout << this->dataPtr->plugin_name << "RL: " << this->dataPtr->F_RL << std::endl;
        // std::cout << this->dataPtr->plugin_name << "RR: " << this->dataPtr->F_RR << std::endl;

        // std::cout << this->dataPtr->plugin_name << "cmd_FL: " << this->dataPtr->motor_cmds[MOTOR_FL] << std::endl;
        // std::cout << this->dataPtr->plugin_name << "cmd_FR: " << this->dataPtr->motor_cmds[MOTOR_FR] << std::endl;
        // std::cout << this->dataPtr->plugin_name << "cmd_RL: " << this->dataPtr->motor_cmds[MOTOR_RL] << std::endl;
        // std::cout << this->dataPtr->plugin_name << "cmd_RR: " << this->dataPtr->motor_cmds[MOTOR_RR] << std::endl;
        
        // std::cout << this->dataPtr->plugin_name << "dt: " << _info.dt.count() << "s" << std::endl;

        // std::cout << this->dataPtr->plugin_name << this->dataPtr->robot_base_frame << " at coordinates (" << this->dataPtr->pose.Pos().X() << "," << this->dataPtr->pose.Pos().Y() << "," << this->dataPtr->pose.Pos().Z() << ")" << std::endl;
        // std::cout << this->dataPtr->plugin_name << this->dataPtr->robot_base_frame << " with orientation RPY: (" << this->dataPtr->pose.Rot().Roll() << "," << this->dataPtr->pose.Rot().Pitch() << "," << this->dataPtr->pose.Rot().Yaw() << ")" << std::endl;

        // std::cout << this->dataPtr->plugin_name << this->dataPtr->robot_base_frame << " linear velocity: (" << this->dataPtr->linearVelocity.X() << "," << this->dataPtr->linearVelocity.Y() << "," << this->dataPtr->linearVelocity.Z() << ")" << std::endl;
        // std::cout << this->dataPtr->plugin_name << this->dataPtr->robot_base_frame << " angular velocity: (" << this->dataPtr->angularVelocity.X() << "," << this->dataPtr->angularVelocity.Y() << "," << this->dataPtr->angularVelocity.Z() << ")" << std::endl;
        
        // std::cout << this->dataPtr->plugin_name << "Target u: " << this->dataPtr->target_u << std::endl;
        // std::cout << this->dataPtr->plugin_name << "Current u: " << this->dataPtr->linear_velocity.X() << std::endl;
        // std::cout << this->dataPtr->plugin_name << "target_pitch: " << this->dataPtr->target_pitch << std::endl;
        // std::cout << this->dataPtr->plugin_name << "Pid_q output: " << this->dataPtr->body_torque.Y() << std::endl;

        // std::cout << this->dataPtr->plugin_name << "Target v: " << this->dataPtr->target_v << std::endl;
        // std::cout << this->dataPtr->plugin_name << "Current v: " << this->dataPtr->linear_velocity.Y() << std::endl;
        // std::cout << this->dataPtr->plugin_name << "target_roll: " << this->dataPtr->target_roll << std::endl;
        // std::cout << this->dataPtr->plugin_name << "Pid_p output: " << this->dataPtr->body_torque.X() << std::endl;

        // std::cout << this->dataPtr->plugin_name << "Target w: " << this->dataPtr->target_w << std::endl;
        // std::cout << this->dataPtr->plugin_name << "Current w: " << this->dataPtr->linear_velocity.Z() << std::endl;

        // std::cout << this->dataPtr->plugin_name << "Target r: " << this->dataPtr->target_r << std::endl;
        // std::cout << this->dataPtr->plugin_name << "Current r: " << this->dataPtr->angular_velocity.Z() << std::endl;
        // std::cout << this->dataPtr->plugin_name << "Pid_r output: " << this->dataPtr->body_torque.Z() << std::endl;
        // std::cout << this->dataPtr->plugin_name << "" << std::endl;
    }
}


/**
    * @brief Callback for the cmd_vel topic
    * @details u_d, v_d, w_d, p_d, q_d, r_d are updated according to received messages.
    * @param _msg Twist message
*/
void QuadControllerPrivate::cmd_vel_cb(const gz::msgs::Twist &_msg){
    std::lock_guard<std::mutex> lock(this->mutex);  //Mutex is automatically released when 'lock' goes out of scope
    // gzmsg << this->plugin_name << "Received message from cmd_vel topic: " << _msg.ShortDebugString() << std::endl;
    std::cout << "";
    if(this->navi_state == FLYING){
        // gzmsg << this->plugin_name << "Setting target velocities..." << std::endl;
        this->target_u = gz::math::clamp(_msg.linear().x(), -1.0, 1.0); //Clamp the input to [-1.0, 1.0]m/s
        this->target_v = gz::math::clamp(_msg.linear().y(), -1.0, 1.0); //Clamp the input to [-1.0, 1.0]m/s
        this->target_w = gz::math::clamp(_msg.linear().z(), -0.25, 0.25); //Clamp the input to [-0.25, 0.25]m/s
        this->target_p = gz::math::clamp(_msg.angular().x(), -1.0, 1.0); //Clamp the input to [-1.0, 1.0]rad/s
        this->target_q = gz::math::clamp(_msg.angular().y(), -1.0, 1.0); //Clamp the input to [-1.0, 1.0]rad/s
        this->target_r = gz::math::clamp(_msg.angular().z(), -0.5, 0.5); //Clamp the input to [-0.5, 0.5]rad/s
    }
}


/**
    * @brief Callback for the takeoff topic
    * @param _msg Empty message
*/
void QuadControllerPrivate::take_off_cb(const gz::msgs::Empty &_msg){
    //If the UAV is in LANDED mode, it switches to TAKING_OFF mode
    if(this->navi_state == LANDED){
        std::cout << this->plugin_name << "Taking off..." << std::endl;
        this->navi_state = TAKING_OFF;  //Set the UAV's navigation state to TAKING_OFF
        this->ready = true; //UAV is ready to fly
    }
    else
        gzwarn << this->plugin_name << "Take-off command ignored: drone already took off or is landing!" << std::endl; 
}


/**
    * @brief Callback for the land topic
    * @param _msg Empty message
*/
void QuadControllerPrivate::land_cb(const gz::msgs::Empty &_msg){
    //If the UAV is in FLYING mode, it switches to LANDING mode
    if(this->navi_state == FLYING){
        std::cout << this->plugin_name << "Landing..." << std::endl;
        this->navi_state = LANDING; //Set the UAV's navigation state to LANDING
    }
    else
        gzwarn << this->plugin_name << "Land command ignored: drone already landed or it is taking off!" << std::endl;
}


/**
    * @brief Get the updates from the simulation.
    * @details Get UAV's current pose (position + orientation), linear velocity (in the body frame),
    * angular velocity (in the body frame), and the last update time.
    * @param _info UpdateInfo
    * @param _ecm EntityComponentManager
*/
void QuadControllerPrivate::get_updates(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm){
    this->pose = _ecm.Component<gz::sim::components::Pose>(this->model_entity)->Data(); //UAV's current pose
    this->linear_velocity = _ecm.Component<gz::sim::components::LinearVelocity>(this->canonical_link_entity)->Data();   //UAV's current linear velocity
    this->angular_velocity = _ecm.Component<gz::sim::components::AngularVelocity>(this->canonical_link_entity)->Data(); //UAV's current angular velocity
    this->last_controller_update_time = _info.simTime;  //Last time update
}


/**
    * @brief Get the joint entities of the motors
    * @details These joint entities will later be used to vary the speed of rotation of the corresponding motor.
    * @param _ecm EntityComponentManager
*/
uint8_t QuadControllerPrivate::get_joint_entities(gz::sim::EntityComponentManager &_ecm){   
    for(uint8_t i = 0; i < 4; i++){
        gz::sim::Entity joint_entity = _ecm.EntityByComponents(gz::sim::components::ParentEntity(this->model_entity), gz::sim::components::Name(this->motor_joint_names[i]), gz::sim::components::Joint());
        if(joint_entity != gz::sim::kNullEntity){
            std::cout << this->plugin_name << "Motor " << i+1 << " joint entity: " << joint_entity << std::endl;
            this->motor_joint_entities.push_back(joint_entity);
        }
        else{
            gzerr << this->plugin_name << "Joint entity of motor " << i+1 << " not found for model entity: " << this->model_entity << std::endl;
            return 0;   //Return 0 if any joint entity is not found
        }
    }
    return 1;
}


/**
 * @brief Check whether the UAV is in TAKING_OFF or LANDING mode to then update its navigation state according to its current altitude
 */
void QuadControllerPrivate::check_state(){
    //If the UAV is in TAKING_OFF mode
    if(this->navi_state == TAKING_OFF){
        if(this->z_take_off <= this->pose.Pos().Z()){    //Check if the UAV is high enough
            std::cout << this->plugin_name << "FLYING" << std::endl;
            this->navi_state = FLYING;  //Switch to FLYING mode
        }
    }

    //If the UAV is in LANDING mode
    else if(this->navi_state == LANDING){
        if(this->pose.Pos().Z() <= this->z_land){    //Check if the UAV is close enough to the ground
            std::cout << this->plugin_name << "LANDED" << std::endl;
            this->navi_state = LANDED;  //Switch to LANDED mode
            this->ready = false;
        }
    }
}


/**
 * @brief Implementation of the UAV's attitude controller
 * @param _dt Elapsed time since last update
 */
void QuadControllerPrivate::attitude_control(const double _dt){
    //Get current roll and pitch angles (in radians)
    this->roll = this->pose.Rot().Roll();
    this->pitch = this->pose.Rot().Pitch();

    /**
     * @todo Transform x and y linear velocities (body frame) into lean angles (roll and pitch)
     * The variable corresponding to the target roll is this->target_roll.
     * The variable correponding to the target pitch is this->target_pitch.
     * Refer to Lecture 3 to see how to compute these two angles.
     * 
     * You will have to use the pid controller this->pid_v and this->pid_u as follow:
     * this->pid_u.Update(u - u_d, std::chrono::duration<double>(_dt)) to get the output of PIDu
     * this->pid_v.Update(v - v_d, std::chrono::duration<double>(_dt)) to get the output of PIDv
     * where
     * u is this->linear_velocity.X(),
     * v is this->linear_velocity.Y()
     * 
     * The gravity component can be retrieved as follows:
     * this->gravity.Z(): -g
     */
    
    this->target_roll = this->pid_v.Update(this->linear_velocity.Y() - this->target_v, std::chrono::duration<double>(_dt)) / this->gravity.Z();
    this->target_pitch = -this->pid_u.Update(this->linear_velocity.X() - this->target_u, std::chrono::duration<double>(_dt)) / this->gravity.Z();


    /**
     * @todo Transform lean angles into angle rates.
     * p_d corresponds to this->target_p.
     * q_d corresponds to this->target_q.
     * Use the square root controller to define p_d and q_d as discussed in Lecture 3.
     */

     //Compute target roll rate (p_d)
    float error = this->target_roll - this->roll;
    float K = 0.2;
    if(error >= 0){
        this->target_p = K * sqrt(abs(error));
    }
    else{
        this->target_p = -K * sqrt(abs(error));
    }

    
    //Compute target pitch rate (q_d)

    error = this->target_pitch - this->pitch;
    if(error >= 0){
        this->target_q = K * sqrt(abs(error));
    }
    else{
        this->target_q = -K * sqrt(abs(error));
    }


    /**
     * @todo Apply PID controllers for angle rates to compute the target torques.
     * Do not consider the inertia matrix for now, it will be added at the end of the function.
     * To update the torque applied around x_b, do as follow:
     * this->body_torque.X( PIDp )
     * where
     * PIDp is this->pid_p.Update(p - p_d, dt),
     * p is this->angular_velocity.X(),
     * p_d is this->target_p,
     * dt is std::chrono::duration<double>(_dt)
     * 
     * Do the same thing for PIDq and PIDr.
     */

    this->body_torque.X(this->pid_p.Update(this->angular_velocity.X() - this->target_p, std::chrono::duration<double>(_dt)));
    this->body_torque.Y(this->pid_q.Update(this->angular_velocity.Y() - this->target_q, std::chrono::duration<double>(_dt)));
    this->body_torque.Z(this->pid_r.Update(this->angular_velocity.Z() - this->target_r, std::chrono::duration<double>(_dt)));


    /*************************************DEBUG PURPOSE****************************************** */    
    // std::cout << this->plugin_name << "Target u: " << this->target_u << std::endl;
    // std::cout << this->plugin_name << "Current u: " << this->linear_velocity.X() << std::endl;
    // std::cout << this->plugin_name << "target_pitch: " << this->target_pitch << std::endl;
    // std::cout << this->plugin_name << "Pid_q output: " << this->body_torque.Y() << std::endl;

    // std::cout << this->plugin_name << "Target v: " << this->target_v << std::endl;
    // std::cout << this->plugin_name << "Current v: " << this->linear_velocity.Y() << std::endl;
    // std::cout << this->plugin_name << "target_roll: " << this->target_roll << std::endl;
    // std::cout << this->plugin_name << "Pid_p output: " << this->body_torque.X() << std::endl;

    // std::cout << this->plugin_name << "Target r: " << this->target_r << std::endl;
    // std::cout << this->plugin_name << "Current r: " << this->angular_velocity.Z() << std::endl;
    // std::cout << this->plugin_name << "Pid_r output: " << this->body_torque.Z() << std::endl;
    

    /*********************************DO NOT TOUCH********************************************** */    
    //U2 = Ixx*PIDp, U3 = Iyy*PIDq, U4 = Izz*PIDr
    this->body_torque *= this->inertia;
}


void QuadControllerPrivate::stabilization(const double _dt, gz::sim::EntityComponentManager &_ecm){
    //Run the attitude controller
    this->attitude_control(_dt);

    /**
     * @todo Compute U1.
     * Do not consider neither the UAV's mass nor the gravity component, they will be added at the end of this function.
     * this->pid_w
     * w is this->linear_velocity.Z()
     * w_d is this->target_w
     */

    this->body_force.Z(this->pid_w.Update(this->linear_velocity.Z() - this->target_w, std::chrono::duration<double>(_dt)));


    /***************************************DEBUG PURPOSE******************************************* */
    // std::cout << this->plugin_name << "target_w: " << this->target_w << std::endl;
    // std::cout << this->plugin_name << "PID_w output: " << this->body_force.Z() << std::endl;
    

    /*********************************DO NOT TOUCH********************************************** */    
    //Compensate the body force with the gravity vector
    this->body_force -= this->gravity;    //Gravity vector = [0, 0, -g]
    this->body_force *= this->mass;

    this->mixer();  //motor mixer
    this->apply_motor_forces(_dt, _ecm);    //make motors spin at the target velocity
}


/**
 * @brief Apply motor mixer
 */
void QuadControllerPrivate::mixer(){
    /**
     * @todo Calculate the forces to apply to each motor.
     * Refer to lecture 3 to see how to distribute the force and the torques amoung the target motors' forces.
     */

     this->F_FL = this->body_force.Z()/4.0 + this->body_torque.X();
     this->F_FR = this->body_force.Z()/4.0 - this->body_torque.X();
     this->F_RL = this->body_force.Z()/4.0 + this->body_torque.X();
     this->F_RR = this->body_force.Z()/4.0 - this->body_torque.X();


    
    //Calculate the motor commands (target motor speed) based on their target forces
    this->motor_cmds[MOTOR_FL] = -this->multiplier * this->F_FL;
    this->motor_cmds[MOTOR_FR] = this->multiplier * this->F_FR;
    this->motor_cmds[MOTOR_RL] = this->multiplier * this->F_RL;
    this->motor_cmds[MOTOR_RR] = -this->multiplier * this->F_RR;
}


/**
 * @brief Set the joint velocity for each motor
 * @param _dt Elapsed time
 * @param _ecm  EntityComponentManager
 */
void QuadControllerPrivate::apply_motor_forces(const double _dt, gz::sim::EntityComponentManager &_ecm){
    for(uint8_t i = 0; i < this->motor_joint_entities.size(); i++){
        gz::sim::components::JointVelocityCmd *_jointVelComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(this->motor_joint_entities[i]);
        if(_jointVelComp == nullptr){
            _jointVelComp = _ecm.CreateComponent(this->motor_joint_entities[i], gz::sim::components::JointVelocityCmd({0}));
        }
        _jointVelComp->Data()[0] = this->motor_cmds[i];
    }
}


/**
 * @brief Load user's custom parameters from xacro file
 * @param _sdf Object containing user's custom parameters
 */
void QuadControllerPrivate::load_params(const std::shared_ptr<const sdf::Element> &_sdf){
    gzdbg << this->plugin_name << "Loading parameters from SDF..." << std::endl;

    if(_sdf->HasElement("robot_namespace")){
        this->robot_namespace = _sdf->Get<std::string>("robot_namespace");
        gzdbg << "Robot namespace: " << this->robot_namespace << std::endl;
    }

    if(_sdf->HasElement("robot_base_frame")){
        this->robot_base_frame = _sdf->Get<std::string>("robot_base_frame");
        gzdbg << "Robot base frame: " << this->robot_base_frame << std::endl;
    }

    for(uint8_t i = 0; i < 4; i++){
        if(_sdf->HasElement("motor"+std::to_string(i+1)+"_joint")){
            this->motor_joint_names[i] = _sdf->Get<std::string>("motor"+std::to_string(i+1)+"_joint");
            gzdbg << "Motor " << i+1 << " joint: " << this->motor_joint_names[i] << std::endl;
        }
    }

    if(_sdf->HasElement("speed_test")){
        this->speed_test = _sdf->Get<double>("speed_test");
        gzdbg << "Speed test: " << this->speed_test << std::endl;
    }

    if(_sdf->HasElement("z_take_off")){
        this->z_take_off = _sdf->Get<double>("z_take_off");
        gzdbg << "Z take off: " << this->z_take_off << std::endl;
    }

    if(_sdf->HasElement("z_land")){
        this->z_land = _sdf->Get<double>("z_land");
        gzdbg << "Z land: " << this->z_land << std::endl;
    }

    if(_sdf->HasElement("multiplier")){
        this->multiplier = _sdf->Get<double>("multiplier");
        gzdbg << "Multiplier: " << this->multiplier << std::endl;
    }

    this->pid_u.SetPGain(_sdf->Get<double>("Kp_u"));
    this->pid_u.SetIGain(_sdf->Get<double>("Ki_u"));
    this->pid_u.SetDGain(_sdf->Get<double>("Kd_u"));
    this->pid_u.SetCmdMax(_sdf->Get<double>("uv_cmd_max"));
    this->pid_u.SetCmdMin(-_sdf->Get<double>("uv_cmd_max"));

    this->pid_v.SetPGain(_sdf->Get<double>("Kp_v"));
    this->pid_v.SetIGain(_sdf->Get<double>("Ki_v"));
    this->pid_v.SetDGain(_sdf->Get<double>("Kd_v"));
    this->pid_v.SetCmdMax(_sdf->Get<double>("uv_cmd_max"));
    this->pid_v.SetCmdMin(-_sdf->Get<double>("uv_cmd_max"));

    this->pid_w.SetPGain(_sdf->Get<double>("Kp_w"));
    this->pid_w.SetIGain(_sdf->Get<double>("Ki_w"));
    this->pid_w.SetDGain(_sdf->Get<double>("Kd_w"));
    this->pid_w.SetCmdMax(_sdf->Get<double>("w_cmd_max"));
    this->pid_w.SetCmdMin(-_sdf->Get<double>("w_cmd_max"));

    this->pid_p.SetPGain(_sdf->Get<double>("Kp_p"));
    this->pid_p.SetIGain(_sdf->Get<double>("Ki_p"));
    this->pid_p.SetDGain(_sdf->Get<double>("Kd_p"));
    this->pid_p.SetCmdMax(_sdf->Get<double>("p_cmd_max"));
    this->pid_p.SetCmdMin(-_sdf->Get<double>("p_cmd_max"));

    this->pid_q.SetPGain(_sdf->Get<double>("Kp_q"));
    this->pid_q.SetIGain(_sdf->Get<double>("Ki_q"));
    this->pid_q.SetDGain(_sdf->Get<double>("Kd_q"));
    this->pid_q.SetCmdMax(_sdf->Get<double>("q_cmd_max"));
    this->pid_q.SetCmdMin(-_sdf->Get<double>("q_cmd_max"));

    this->pid_r.SetPGain(_sdf->Get<double>("Kp_r"));
    this->pid_r.SetIGain(_sdf->Get<double>("Ki_r"));
    this->pid_r.SetDGain(_sdf->Get<double>("Kd_r"));
    this->pid_r.SetCmdMax(_sdf->Get<double>("r_cmd_max"));
    this->pid_r.SetCmdMin(-_sdf->Get<double>("r_cmd_max"));
}


/*************************GAZEBO PLUGIN CONFIGURATION************************************************* */
GZ_ADD_PLUGIN(
    QuadController,
    gz::sim::System,
    QuadController::ISystemConfigure,
    QuadController::ISystemPreUpdate,
    QuadController::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(QuadController, "gz::sim::systems::QuadController")
