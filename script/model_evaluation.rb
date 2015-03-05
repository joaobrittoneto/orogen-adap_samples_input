#library for displaying data
require 'orocos'
require 'vizkit'
#require './../../adap_parameters_id/scripts/adap_properties2.rb'
require './model_properties3.rb'
require './gui/gui_evaluation.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 

#Orocos.initialize


# Experiment made at 24/10/2014 from 18:01:07.207319 until 18:07:32.118947 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t)) of frequencie of 0.3 rad/s in surge direction in the AUV Avalon
@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/avalon_logFiles/avalom_model_tests_20141024/back/20141024-1743/test_18h01m.log")

# Experiment made at 24/10/2014 from 17:47:21.488977 until 17:51:54.407288 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t)) of frequencie of 0.5 rad/s in surge direction in the AUV Avalon
#@log_replay = Orocos::Log::Replay.open("../../../../../Log_files/avalon_logFiles/avalom_model_tests_20141024/back/20141024-1743/test_17h47m.log")


Orocos.run 'adap_samples_input::Task' => 'adap_samples',
           'motion_model::Compare'    => 'motion_model',
           'motion_model::Evaluation' => 'evaluation' do

    adap_samples = TaskContext.get 'adap_samples'
    model        = TaskContext.get 'motion_model'
    eval         = TaskContext.get 'evaluation'

       
    
    configure_model = model
    model_properties(configure_model)
    model = configure_model
      
    
    @log_replay.sonar_feature_estimator.new_feature.connect_to adap_samples.position_samples
    @log_replay.motion_control.joint_commands.connect_to adap_samples.forces_samples
        
 
    adap_samples.forces.connect_to model.thruster_samples       
    adap_samples.velocity.connect_to model.initial_velocity 
    adap_samples.velocity.connect_to eval.measured_velocity,    :type => :buffer, :size => 500
    model.velocity.connect_to eval.model_velocity,              :type => :buffer, :size => 500
    
    
   # eval.mae_velocity.connect_to do |sample|
   #     prints "ruby MAE_Vel: "
   #     puts sample
   #end
   
   #eval.norm_mae_velocity.connect_to do |sample|
   #     prints "ruby Norm_MAE_Vel: "
  #      puts sample
  # end
    
    
    adap_samples.configure
    model.configure
    eval.configure
    adap_samples.start 
    model.start
    eval.start

 
   ## Defining the proxy for each task 
   inputproxy = Orocos::Async.proxy("adap_samples")
   speedproxy = Orocos::Async.proxy("motion_model")
   evalproxy = Orocos::Async.proxy("evaluation")
   
	
   ## Defining the port variables using the proxys
  
   velocityModelport            = speedproxy.port("velocity")
   velocityMeasuredport         = inputproxy.port("velocity")
   errorvport                   = evalproxy.port("error_velocity")
   maevport                    = evalproxy.port("mae_velocity")
   normmaevport                = evalproxy.port("norm_mae_velocity")
  
  
        
        
    #open control widget and start replay
    supervisory = Supervisory.new(model)
    supervisory.evaluation(velocityModelport,velocityMeasuredport, errorvport, maevport, normmaevport)
    
        
    supervisory.show
   
   
   
   Vizkit.control @log_replay
   Vizkit.exec
   
   #@log_replay.run#(true,1)
   
   
end
