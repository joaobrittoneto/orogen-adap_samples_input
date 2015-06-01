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

Orocos.run 'adap_samples_input::GetPoseForce' => 'adap_samples',
           'adap_parameters_estimator::Task' => 'parameters_estimator' do

    adap_samples = TaskContext.get 'adap_samples'
    adapP        = TaskContext.get 'parameters_estimator'

    adap_samples.apply_conf_file('config/GetPoseForce.yml',['dagon_ls_method'])
    adapP.apply_conf_file('config/adap_parameters_estimator.yml',['dagon_surge_aligned'])
    
    #adap_samples.apply_conf_file('GetPoseForce.yml',['dagon'])
    #adapP.apply_conf_file('adap_parameters_estimator.yml',['dagon_surge_Notaligned'])
    

      
    #@log_replay.pose_estimator.pose_samples.connect_to  adap_samples.pose_samples 
    @log_replay.pose_estimation.pose_samples.connect_to  adap_samples.pose_samples
    @log_replay.dispatcher.all_joint_state.connect_to    adap_samples.thruster_samples
    
    adap_samples.dynamic.connect_to     adapP.dynamic_samples,  :type => :buffer, :size => 500 
    
    
#    adap_samples.velocity.connect_to    adapP.speed_samples,  :type => :buffer, :size => 500 
#    adap_samples.forces.connect_to      adapP.thruster_samples,  :type => :buffer, :size => 500 
    
      
    adap_samples.configure
    adapP.configure
    adap_samples.start 
    adapP.start
    
   
 #   widget.show
 #   widget2.show  
 #   widget3.show  
 #   widget4.show  
 #   widget5.show 
 #   widget6.show 
 
   ## Defining the proxy for each task 
   parametersproxy = Orocos::Async.proxy("parameters_estimator")
   inputproxy = Orocos::Async.proxy("adap_samples")
   
	
   ## Defining the port variables using the proxys
   deltaVport       = parametersproxy.port("deltaV")
   nomrDeltaVport   = parametersproxy.port("normDeltaV")
   parametersport   = parametersproxy.port("parameters")
   velocityport     = inputproxy.port("velocity")
   forcesport       = inputproxy.port("forces")
        
    #open control widget and start replay
    supervisory = Supervisory.new(adapP)
    supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
    
        
    supervisory.show
    
    Vizkit.control @log_replay
    Vizkit.exec
end
