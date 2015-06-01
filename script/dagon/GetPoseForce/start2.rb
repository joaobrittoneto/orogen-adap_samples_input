#library for displaying data
require 'vizkit'
include Orocos
Orocos.initialize

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 
#######################################################################
@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1743/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1707/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1519/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1615/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1635/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1703/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1724/angZ.log")

#######################################################################

#   require "readline"

Orocos.run 'adap_samples_input::GetPoseForcePeriodic' => 'adap_samples',
           'adap_samples_input::ForceApplier' => 'forces&torques'   do 

    #Orocos.transformer.load_conf('config/transforms.rb')

    adap_samples        = TaskContext.get 'adap_samples'
    forces_torques      = TaskContext.get 'forces&torques'
        
   # adap_samples.apply_conf_file('config/adap_samples_input::GetPoseForcePeriodic.yml',['default','dagon_adap_method'])
    adap_samples.apply_conf_file('config/adap_samples_input::GetPoseForcePeriodic.yml',['default','dagon_ls_method'])
    forces_torques.apply_conf_file('config/adap_samples_input::ForceApplier.yml',['dagon'])    
        
    #Readline.readline
    
    widget = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d
    widget3 = Vizkit.default_loader.Plot2d
    widget4 = Vizkit.default_loader.Plot2d
    widget5 = Vizkit.default_loader.Plot2d
    widget6 = Vizkit.default_loader.Plot2d

    pose_estimation     = @log_replay.pose_estimation
    dispatcher          = @log_replay.dispatcher   
    
    pose_estimation.pose_samples.connect_to     adap_samples.pose_samples
    dispatcher.all_joint_state.connect_to       forces_torques.thruster_samples
    forces_torques.forces.connect_to            adap_samples.forces_samples
    
    
 
     
    adap_samples.dynamic.connect_to do |sample, _|        
        widget.update(sample.rbs.velocity[0], "vel_x [m/s]")
        widget.update(sample.rbs.velocity[1], "vel_y [m/s]")
        widget.update(sample.rbs.velocity[2], "vel_z [m/s]")
        widget2.update(sample.rbs.angular_velocity[0], "ang_vel_x [rad/s]") 
        widget2.update(sample.rbs.angular_velocity[1], "ang_vel_y [rad/s]") 
        widget2.update(sample.rbs.angular_velocity[2], "ang_vel_z [rad/s]") 
        
        widget3.update(sample.joints.elements[0].effort, "force_x [N]")
        widget3.update(sample.joints.elements[1].effort, "force_y [N]")
        widget3.update(sample.joints.elements[2].effort, "force_z [N]")
        widget4.update(sample.joints.elements[3].effort, "torque_x [Nm]")
        widget4.update(sample.joints.elements[4].effort, "torque_y [Nm]")
        widget4.update(sample.joints.elements[5].effort, "torque_z [Nm]")
        
        widget5.update(sample.rba.acceleration[0], "accel_x [m/s^2]")
        widget5.update(sample.rba.acceleration[1], "accel_y [m/s^2]")
        widget5.update(sample.rba.acceleration[2], "accel_z [m/s^2]")     
        widget6.update(sample.ang_rba.acceleration[0], "ang_accel_x [rad/s^2]") 
        widget6.update(sample.ang_rba.acceleration[1], "ang_accel_y [rad/s^2]") 
        widget6.update(sample.ang_rba.acceleration[2], "ang_accel_z [rad/s^2]")     
    end
    
  # Orocos.transformer.setup(adap_samples)
    
    adap_samples.configure
    forces_torques.configure
    forces_torques.start
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
