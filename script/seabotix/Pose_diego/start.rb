#library for displaying data
require 'vizkit'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 



#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/surge/linX.log")

@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/yaw_1/angZ_1.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/yaw_2/angZ_2.log")

   require "readline"

Orocos.run 'adap_parameters_estimator::ForceApplier' => 'forces&torques',
           'adap_samples_input::Seabotix' => 'adap_samples' do

    adap_samples = TaskContext.get 'adap_samples'
    forces_torques = TaskContext.get 'forces&torques'
    
    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['seabotix']) 
    
    #Readline.readline
    
    widget = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d
    widget3 = Vizkit.default_loader.Plot2d
    widget4 = Vizkit.default_loader.Plot2d

      
      
    @log_replay.Seabotix.thrusters_states.connect_to forces_torques.thruster_samples
    @log_replay.detector.marker_poses.connect_to adap_samples.position_samples
    forces_torques.forces.connect_to adap_samples.forces_samples
    
    adap_samples.step = 0.1
    adap_samples.delay = 1.7 #1.0155
    adap_samples.number_samples = 50
    
    @log_replay.Seabotix.thrusters_states.connect_to  do |samples|
       # print samples.elements[0].raw
    end
    
    forces_torques.forces.connect_to do |samples|
        #print samples.elements[0].effort
    end    
     
    adap_samples.velocity.connect_to do |sample, _|        
        widget.update(sample.position[0], "postion [m]")
        widget2.update(sample.velocity[0], "velocity [m/s]") 
       # widget2.set_y_axis_scale(-1.3,1.3)   
    end
    
    adap_samples.forces.connect_to do |sample, _|   
        widget3.update(sample.elements[0].effort, "force [N]")
        #widget3.set_y_axis_scale(-1.3,1.3)        
    end
    
    adap_samples.acceleration.connect_to do |sample, _|        
        widget4.update(sample.acceleration[0], "acceleration [m/s^2]") 
       # widget4.set_y_axis_scale(-1.3,1.3)   
    end
    
    adap_samples.configure
    forces_torques.configure
    adap_samples.start
    forces_torques.start 
    
   
#    widget.show
#    widget2.show  
#    widget3.show  
#    widget4.show  
        
    #open control widget and start replay
    Vizkit.control @log_replay
    Vizkit.exec
    
end
