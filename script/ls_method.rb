#library for displaying data
require 'orocos'
require 'vizkit'
require './adap_properties3.rb'
require './gui/gui_parameters.rb'

include Orocos


# Experiment made at 24/10/2014 from 18:01:07.207319 until 18:07:32.118947 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t) m/s) with frequencie of 0.3 rad/s in surge direction in the AUV Avalon
#@log_replay = Orocos::Log::Replay.open("../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/test_18h01m")

# Experiment made at 24/10/2014 from 17:47:21.488977 until 17:51:54.407288 at DFKI, applying a signal (setpoint velocity in surge of 1*sin(f*t) m/s) with frequencie of 0.5 rad/s in surge direction in the AUV Avalon
@log_replay = Orocos::Log::Replay.open("../../../../avalon/orogen/avalonControl/script/avalom_model_tests_20141024/back/20141024-1743/test_17h47m")


Orocos.run 'adap_samples_input::Task' => 'adap_samples',
           'ls_pseudoinverse::Task' => 'ls_method' do

    adap_samples = TaskContext.get 'adap_samples'
    ls = TaskContext.get 'ls_method'


     
    @log_replay.sonar_feature_estimator.new_feature.connect_to adap_samples.position_samples
    @log_replay.motion_control.joint_commands.connect_to adap_samples.forces_samples
    
    #adap_samples.forces.connect_to ls.forces_samples                                        
    #adap_samples.velocity.connect_to ls.speed_samples                   
    #adap_samples.acceleration.connect_to ls.acceleration_samples     
    adap_samples.dynamic.connect_to ls.dynamic_samples,  :type => :buffer, :size => 500    
    
  
   
     
        
    adap_samples.configure
    ls.configure
    adap_samples.start 
    ls.start
 
   ## Defining the proxy for each task 
#   parametersproxy = Orocos::Async.proxy("parameters_estimator")
#   inputproxy = Orocos::Async.proxy("adap_samples")
   
	
   ## Defining the port variables using the proxys
#   deltaVport       = parametersproxy.port("deltaV")
#   nomrDeltaVport   = parametersproxy.port("normDeltaV")
#   parametersport   = parametersproxy.port("parameters")
#   velocityport     = inputproxy.port("velocity")
#   forcesport       = inputproxy.port("forces")
        
    #open control widget and start replay
#    supervisory = Supervisory.new(adapP)
#    supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
    
        
#    supervisory.show
    
    Vizkit.control @log_replay
    Vizkit.exec
end
