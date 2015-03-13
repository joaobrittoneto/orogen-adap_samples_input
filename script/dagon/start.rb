#library for displaying data
require 'vizkit'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 



#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angZ.log")

@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/all_in_one.log")

#   require "readline"

Orocos.run 'adap_samples_input::Dagon' => 'adap_samples' do 

    adap_samples = TaskContext.get 'adap_samples'
        
    #adap_samples.apply_conf_file('adap_samples_input::Dagon.yml',['dagon'])
    adap_samples.apply_conf_file('adap_samples_input::Dagon.yml',['dagon_adap_raw_data'])
        
    #Readline.readline
    
    widget = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d
    widget3 = Vizkit.default_loader.Plot2d
    widget4 = Vizkit.default_loader.Plot2d
    widget5 = Vizkit.default_loader.Plot2d
    widget6 = Vizkit.default_loader.Plot2d

      
      
    @log_replay.dvl.velocity_samples.connect_to adap_samples.dvl_samples
    @log_replay.orientation_estimator.attitude_b_g.connect_to adap_samples.orientation_samples
    
    @log_replay.pose_estimator.pose_samples.connect_to adap_samples.position_samples
    @log_replay.dispatcher.all_joint_state.connect_to adap_samples.forces_samples
    
    
 
     
    adap_samples.velocity.connect_to do |sample, _|        
        widget.update(sample.velocity[0], "vel_x [m/s]")
        widget.update(sample.velocity[1], "vel_y [m/s]")
        widget.update(sample.velocity[2], "vel_z [m/s]")
        widget2.update(sample.angular_velocity[0], "ang_vel_x [rad/s]") 
        widget2.update(sample.angular_velocity[1], "ang_vel_y [rad/s]") 
        widget2.update(sample.angular_velocity[2], "ang_vel_z [rad/s]") 
        #widget2.set_y_axis_scale(-1.3,1.3)   
    end
    
    adap_samples.forces.connect_to do |sample, _|   
        widget3.update(sample.elements[0].effort, "force_x [N]")
        widget3.update(sample.elements[1].effort, "force_y [N]")
        widget3.update(sample.elements[2].effort, "force_z [N]")
        widget4.update(sample.elements[3].effort, "torque_x [Nm]")
        widget4.update(sample.elements[4].effort, "torque_y [Nm]")
        widget4.update(sample.elements[5].effort, "torque_z [Nm]")
        #widget3.set_y_axis_scale(-1.3,1.3)        
    end
    
    adap_samples.lin_acceleration.connect_to do |sample, _|        
        widget5.update(sample.acceleration[0], "accel_x [m/s^2]")
        widget5.update(sample.acceleration[1], "accel_y [m/s^2]")
        widget5.update(sample.acceleration[2], "accel_z [m/s^2]") 
        #widget4.set_y_axis_scale(-1.3,1.3)   
    end
    
    adap_samples.ang_acceleration.connect_to do |sample, _|        
        widget6.update(sample.acceleration[0], "ang_accel_x [rad/s^2]") 
        widget6.update(sample.acceleration[1], "ang_accel_y [rad/s^2]") 
        widget6.update(sample.acceleration[2], "ang_accel_z [rad/s^2]") 
        #widget5.set_y_axis_scale(-1.3,1.3)   
    end
    
    adap_samples.configure
    adap_samples.start 
    
   
    widget.show
    widget2.show  
    widget3.show  
    widget4.show 
    widget5.show
    widget6.show 
    
    #Readline.readline
        
    #open control widget and start replay
    Vizkit.control @log_replay
    Vizkit.exec
    
end
