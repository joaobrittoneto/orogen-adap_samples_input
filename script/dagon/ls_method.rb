#library for displaying data
require 'orocos'
require 'vizkit'
#require './adap_properties.rb'
require './../gui/gui_parameters.rb'

include Orocos


#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/all_in_one.log")

#######################################################################
@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1743/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1707/linX.log")

#######################################################################


Orocos.run 'adap_samples_input::Dagon' => 'adap_samples',
           'ls_pseudoinverse::Task' => 'ls_method' do 

    adap_samples = TaskContext.get 'adap_samples'
    ls = TaskContext.get 'ls_method'
    
    adap_samples.apply_conf_file('adap_samples_input::Dagon.yml',['dagon'])
    ls.apply_conf_file('ls_pseudoinverse::Task.yml',['surge'])
    #ls.apply_conf_file('ls_pseudoinverse::Task.yml',['sway'])
    #ls.apply_conf_file('ls_pseudoinverse::Task.yml',['heave'])
    #ls.apply_conf_file('ls_pseudoinverse::Task.yml',['pitch'])
    #ls.apply_conf_file('ls_pseudoinverse::Task.yml',['yaw'])



     
    #@log_replay.pose_estimator.pose_samples.connect_to adap_samples.position_samples
    @log_replay.pose_estimation.pose_samples.connect_to adap_samples.position_samples
    @log_replay.dispatcher.all_joint_state.connect_to adap_samples.forces_samples
    
        
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
