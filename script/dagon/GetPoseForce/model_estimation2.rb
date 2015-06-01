#library for displaying data
require 'orocos'
require 'vizkit'
#require './adap_properties.rb'
require './../../gui/gui_parameters.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 

#Orocos.initialize
########################################################################
#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/all_in_one.log")
#######################################################################
@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1743/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1707/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1519/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1615/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1635/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1703/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1724/angZ.log")

#######################################################################

Orocos.run 'adap_samples_input::GetPoseForcePeriodic' => 'adap_samples2',
           'adap_samples_input::ForceApplier' => 'forces&torques',
           'adap_parameters_estimator::Task' => 'parameters_estimator2' do

    adap_samples2        = TaskContext.get 'adap_samples2'
    forces_torques      = TaskContext.get 'forces&torques'
    adapP2               = TaskContext.get 'parameters_estimator2'

    adap_samples2.apply_conf_file('config/adap_samples_input::GetPoseForcePeriodic.yml',['default','dagon_adap_method'])
    forces_torques.apply_conf_file('config/adap_samples_input::ForceApplier.yml',['dagon']) 
    adapP2.apply_conf_file('config/adap_parameters_estimator.yml',['dagon_surge_aligned'])
    
 
    pose_estimation     = @log_replay.pose_estimation
    dispatcher          = @log_replay.dispatcher   
    
   #@log_replay.pose_estimator.pose_samples.connect_to  adap_samples2.pose_samples 
    pose_estimation.pose_samples.connect_to     adap_samples2.pose_samples,     :type => :buffer, :size => 100       
    dispatcher.all_joint_state.connect_to       forces_torques.thruster_samples,:type => :buffer, :size => 100 
    forces_torques.forces.connect_to            adap_samples2.forces_samples,   :type => :buffer, :size => 100 
    
    adap_samples2.dynamic.connect_to             adapP2.dynamic_samples,        :type => :buffer, :size => 100 
    
    #adap_samples.velocity.connect_to            adapP.speed_samples,  :type => :buffer, :size => 500 
    #forces_torques.forces.connect_to            adapP.thruster_samples,  :type => :buffer, :size => 500      
    
    
    forces_torques.configure      
    adap_samples2.configure
    adapP2.configure
    forces_torques.start
    adap_samples2.start 
    adapP2.start
    
    
   

 
   ## Defining the proxy for each task 
   parametersproxy = Orocos::Async.proxy("parameters_estimator2")
   inputproxy = Orocos::Async.proxy("adap_samples2")
   
	
   ## Defining the port variables using the proxys
   deltaVport       = parametersproxy.port("deltaV")
   nomrDeltaVport   = parametersproxy.port("normDeltaV")
   parametersport   = parametersproxy.port("parameters")      
   velocityport     = inputproxy.port("dynamic.rbs")
   forcesport       = inputproxy.port("dynamic.joints")
        
    #open control widget and start replay
    supervisory = Supervisory.new(adapP2)
    supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
    
        
    supervisory.show
    
    Vizkit.control @log_replay
    Vizkit.exec
end
