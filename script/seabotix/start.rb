#library for displaying data
require 'vizkit'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 



#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/seabotix_logFiles/seabotix201501091640/detector.0.log", "../../../../../../Log_files/seabotix_logFiles/seabotix201501091640/controller_operation_no_sp.0.log")

@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/seabotix_logFiles/seabotix201501091640/detector.0.log", "../../../../../../Log_files/seabotix_logFiles/seabotix201501091640/Seabotix.0.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/seabotix_logFiles/seabotix201501091647/detector.0.log", "../../../../../../Log_files/seabotix_logFiles/seabotix201501091647/Seabotix.0.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/seabotix_logFiles/seabotix201501191600_step1/detector.1.log", "../../../../../../Log_files/seabotix_logFiles/seabotix201501191600_step1/Seabotix.1.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/seabotix_logFiles/seabotix201501191605_step2/detector.2.log", "../../../../../../Log_files/seabotix_logFiles/seabotix201501191605_step2/Seabotix.2.log")

   require "readline"

Orocos.run 'adap_samples_input::Seabotix' => 'adap_samples' do

    adap_samples = TaskContext.get 'adap_samples'

    #Readline.readline
    
    widget = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d
    widget3 = Vizkit.default_loader.Plot2d
    widget4 = Vizkit.default_loader.Plot2d

      
      
    
    @log_replay.detector.pose.connect_to adap_samples.position_samples
    @log_replay.Seabotix.thrusters_states.connect_to adap_samples.forces_samples
    #@log_replay.EffortToControl.control_output.connect_to adap_samples.forces_samples
    
 
     
    adap_samples.velocity.connect_to do |sample, _|        
        widget.update(sample.position[0], "postion [m]")
        widget2.update(sample.velocity[0], "velocity [m/s]") 
        widget2.set_y_axis_scale(-1.3,1.3)   
    end
    
    adap_samples.forces.connect_to do |sample, _|   
        widget3.update(sample.elements[0].effort, "force [N]")
        #widget3.set_y_axis_scale(-1.3,1.3)        
    end
    
    adap_samples.acceleration.connect_to do |sample, _|        
        widget4.update(sample.acceleration[0], "acceleration [m/s^2]") 
        widget4.set_y_axis_scale(-1.3,1.3)   
    end
    
    adap_samples.configure
    adap_samples.start 
    
   
    widget.show
    widget2.show  
    widget3.show  
    widget4.show  
        
    #open control widget and start replay
    Vizkit.control @log_replay
    Vizkit.exec
    
end
