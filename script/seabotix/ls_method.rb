#library for displaying data
require 'orocos'
require 'vizkit'
require './adap_properties3.rb'
require './../gui/gui_parameters.rb'

include Orocos


@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/seabotix_logFiles/seabotix201501091640/detector.0.log", "../../../../../../Log_files/seabotix_logFiles/seabotix201501091640/Seabotix.0.log")


Orocos.run 'adap_samples_input::Seabotix' => 'adap_samples',
           'ls_pseudoinverse::Task' => 'ls_method' do

    adap_samples = TaskContext.get 'adap_samples'
    ls = TaskContext.get 'ls_method'


     
    @log_replay.detector.pose.connect_to adap_samples.position_samples
    @log_replay.Seabotix.thrusters_states.connect_to adap_samples.forces_samples
    #@log_replay.EffortToControl.control_output.connect_to adap_samples.forces_samples
    
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
