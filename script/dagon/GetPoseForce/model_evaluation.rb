#library for displaying data
require 'orocos'
require 'vizkit'
#require './../../adap_parameters_id/scripts/adap_properties2.rb'
require './model_properties.rb'
require './../../gui/gui_evaluation.rb'

include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon

#load log file 

#Orocos.initialize

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/linZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation_20150303/20150303-1924/all_in_one.log")
#######################################################################
#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1743/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150409-1707/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1519/linY.log")

@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1615/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1635/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1703/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1724/angZ.log")

#######################################################################

Orocos.run 'adap_samples_input::Seabotix' => 'adap_samples',
           'motion_model::Compare'    => 'motion_model',
           'motion_model::Evaluation' => 'evaluation' do

    adap_samples = TaskContext.get 'adap_samples'
    model        = TaskContext.get 'motion_model'
    eval         = TaskContext.get 'evaluation'

       
    
    configure_model = model
    model_properties(configure_model)
    model = configure_model
      
    
   #@log_replay.pose_estimator.pose_samples.connect_to  adap_samples.pose_samples  
    @log_replay.pose_estimation.pose_samples.connect_to  adap_samples.pose_samples
    @log_replay.dispatcher.all_joint_state.connect_to   adap_samples.thruster_samples
        
 
    adap_samples.forces.connect_to      model.thruster_samples       
    adap_samples.velocity.connect_to    model.initial_velocity 
    adap_samples.velocity.connect_to    eval.measured_velocity,    :type => :buffer, :size => 500
    model.velocity.connect_to           eval.model_velocity,       :type => :buffer, :size => 500
    
    
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
