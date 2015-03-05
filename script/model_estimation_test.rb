#library for displaying data
require 'orocos'
require 'vizkit'
require './adap_properties3.rb'
#require './../../adap_parameters_id/scripts/adap_properties2.rb'
require './model_properties3.rb'
require './gui/gui_parameters.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 

#Orocos.initialize

# Index from 16140 to 27100 (from 18:01:07~ to 18:07:50~) and from 2600 to 9600 (from 17:47:30~ to 17:52:10~)
#@log_replay = Orocos::Log::Replay.open("../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/sonar_feature_estimator.0.log", "../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/avalon_back_base_control.0.log")

# Experiment made at 24/10/2014 from 18:01:07.207319 until 18:07:32.118947 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t)) of frequencie of 0.3 rad/s in surge direction in the AUV Avalon
#@log_replay = Orocos::Log::Replay.open("../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/test_18h01m")

# Experiment made at 24/10/2014 from 17:47:21.488977 until 17:51:54.407288 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t)) of frequencie of 0.5 rad/s in surge direction in the AUV Avalon
#@log_replay = Orocos::Log::Replay.open("../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/test_17h47m")

# Experiment made at 24/10/2014 from 18:01:07.207319 until 18:07:32.118947 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t)) of frequencie of 0.3 rad/s in surge direction in the AUV Avalon
@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/avalon_logFiles/avalom_model_tests_20141024/back/20141024-1743/test_18h01m.log")

# Experiment made at 24/10/2014 from 17:47:21.488977 until 17:51:54.407288 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t)) of frequencie of 0.5 rad/s in surge direction in the AUV Avalon
#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/avalon_logFiles/avalom_model_tests_20141024/back/20141024-1743/test_17h47m.log")


Orocos.run 'adap_samples_input::Task' => 'adap_samples',
           'adap_parameters_estimator::Task' => 'parameters_estimator',
           'motion_model::Task' => 'motion_model' do

    adap_samples = TaskContext.get 'adap_samples'
    adapP        = TaskContext.get 'parameters_estimator'
    model        = TaskContext.get 'motion_model'

    
    
 #   widget = Vizkit.default_loader.Plot2d
 #   widget2 = Vizkit.default_loader.Plot2d
 #   widget3 = Vizkit.default_loader.Plot2d
 #   widget4 = Vizkit.default_loader.Plot2d
 #   widget5 = Vizkit.default_loader.Plot2d
 #   widget6 = Vizkit.default_loader.Plot2d

    configure_adap = adapP
    adap_properties(configure_adap)
    adapP = configure_adap
    
    configure_model = model
    model_properties(configure_model)
    model = configure_model
      
    
    @log_replay.sonar_feature_estimator.new_feature.connect_to adap_samples.position_samples
    @log_replay.motion_control.joint_commands.connect_to adap_samples.forces_samples
    
    adap_samples.forces.connect_to adapP.thruster_samples
    adap_samples.forces.connect_to model.thruster_samples
    #adap_samples.velocity.connect_to adapP.speed_samples
    
    model.velocity.connect_to adapP.speed_samples
    
   
    
    
 
     
 #   adap_samples.velocity.connect_to do |sample, _|        
 #       widget.update(sample.position[0], "postion [m]")
 #       widget2.update(sample.velocity[0], "velocity [m/s]") 
 #       widget2.set_y_axis_scale(-1.3,1.3)   
 #   end
    
 #   adap_samples.forces.connect_to do |sample, _|   
 #       widget3.update(sample.elements[0].effort, "force [N]")
 #       widget3.set_y_axis_scale(-1.3,1.3)        
 #   end
    
 #   adapP.parameters.connect_to do |sample, _|        
 #       widget4.update(sample.inertiaCoeff[0].positive, "mass [kg]") 
 #       widget4.set_y_axis_scale(-10,1000) 
 #        
 #   end
    
 #   adapP.deltaV.connect_to do |sample, _|        
 #       widget5.update(sample, "deltaV [m/s]") 
 #       widget5.set_y_axis_scale(-1.3,1.3)      
 #   end
    
 #   adapP.normDeltaV.connect_to do |sample, _|        
 #       widget6.update(sample, "deltaV [m/s]") 
 #       #widget6.set_y_axis_scale(-1.3,1.3)      
 #   end
    
    adap_samples.configure
    adapP.configure
    model.configure
    adap_samples.start 
    adapP.start
    model.start
   
 #   widget.show
 #   widget2.show  
 #   widget3.show  
 #   widget4.show  
 #   widget5.show 
 #   widget6.show 
 
   ## Defining the proxy for each task 
   parametersproxy = Orocos::Async.proxy("parameters_estimator")
   inputproxy = Orocos::Async.proxy("adap_samples")
   speedproxy = Orocos::Async.proxy("motion_model")
   
	
   ## Defining the port variables using the proxys
   deltaVport       = parametersproxy.port("deltaV")
   nomrDeltaVport   = parametersproxy.port("normDeltaV")
   parametersport   = parametersproxy.port("parameters")
   velocityport     = speedproxy.port("velocity")
   #velocityport     = inputproxy.port("velocity")
   forcesport       = inputproxy.port("forces")
        
    #open control widget and start replay
    supervisory = Supervisory.new(adapP)
    supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
    
        
    supervisory.show
    
    Vizkit.control @log_replay   
    Vizkit.exec
end
