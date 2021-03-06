#library for displaying data
require 'vizkit'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 


# Experiment made at 24/10/2014 from 18:01:07.207319 until 18:07:32.118947 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t)) of frequencie of 0.3 rad/s in surge direction in the AUV Avalon
@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/avalon_logFiles/avalom_model_tests_20141024/back/20141024-1743/test_18h01m.log")

# Experiment made at 24/10/2014 from 17:47:21.488977 until 17:51:54.407288 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t)) of frequencie of 0.5 rad/s in surge direction in the AUV Avalon
#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/avalon_logFiles/avalom_model_tests_20141024/back/20141024-1743/test_17h47m.log")



Orocos.run 'adap_samples_input::Task' => 'adap_samples' do

    adap_samples = TaskContext.get 'adap_samples'

    
    
    widget = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d
    widget3 = Vizkit.default_loader.Plot2d
    widget4 = Vizkit.default_loader.Plot2d

      
      
    
    @log_replay.sonar_feature_estimator.new_feature.connect_to adap_samples.position_samples
    @log_replay.motion_control.joint_commands.connect_to adap_samples.forces_samples
    
 
     
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
