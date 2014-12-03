#library for displaying data
require 'vizkit'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 

# Index from 16140 to 27100 (from 18:01:07~ to 18:07:50~) and from 2600 to 9600 (from 17:47:30~ to 17:52:10~)
@log_replay = Orocos::Log::Replay.open("../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/sonar_feature_estimator.0.log", "../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/avalon_back_base_control.0.log")



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
