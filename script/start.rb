#library for displaying data
require 'vizkit'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 
@log_replay = Orocos::Log::Replay.open("../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/sonar_feature_estimator.0.log", "../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/avalon_back_base_control.0.log")

#@log_forces = Orocos::Log::Replay.open("../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/avalon_back_base_control.0.log")


#@log_position.use_sample_time = true


Orocos.run 'adap_samples_input::Task' => 'adap_samples' do

    adap_samples = TaskContext.get 'adap_samples'

    
    
    widget = Vizkit.default_loader.Plot2d
    widget2 = Vizkit.default_loader.Plot2d

      
      
    
    @log_replay.sonar_feature_estimator.new_feature.connect_to adap_samples.position_samples
    @log_replay.motion_control.joint_commands.connect_to adap_samples.forces_samples
    
    #@timer = Qt::Timer.new  
     
    adap_samples.velocity.connect_to do |sample, _|
        
     #    time = sample.time
     #    @time ||= time 
     #    x = time.to_f-@time.to_f
        
     #   widget.update_custom("position [m]", x, sample.position[0])
     #   widget2.update_custom("velocity [m/s]", x, sample.velocity[0])
        
        widget.update(sample.position[0], "postion [m]")
        widget2.update(sample.velocity[0], "velocity [m/s]") 
        
        #puts sample.velocity[0]
        #widget2.config(sample) 
        widget2.set_y_axis_scale(-1.3,1.3)
        
    end
    
    adap_samples.forces.connect_to do |sample, _|
          
        #puts sample.elements[0].effort
    
            
    end
    
    adap_samples.configure
    adap_samples.start 
    
   

    widget.show
    widget2.show    
        
    #open control widget and start replay
    Vizkit.control @log_replay
    Vizkit.exec
end
