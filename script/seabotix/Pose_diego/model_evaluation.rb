#library for displaying data
require 'orocos'
require 'vizkit'
require './../../gui/gui_evaluation.rb'

include Orocos



#load log file 

#Orocos.initialize

######################################################################
@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/surge/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/yaw_1/angZ_1.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/yaw_2/angZ_2.log")

#######################################################################

Orocos.run 'uwv_motion_model::Task'                             => 'motion_model',
           'adap_samples_input::Seabotix'                       => 'adap_samples',        
           'adap_parameters_estimator::ForceApplier'            => 'forces&torques',
           'adap_parameters_estimator::Evaluation'              => 'evaluation' do

           
    widget = Vizkit.default_loader.Plot2d 
    widget2 = Vizkit.default_loader.Plot2d      

    model               = TaskContext.get 'motion_model'
    adap_samples        = TaskContext.get 'adap_samples'
    forces_torques      = TaskContext.get 'forces&torques'
    eval                = TaskContext.get 'evaluation'

    model.apply_conf_file('config/uwv_motion_model::Task.yml',['seabotix_ls']) 
    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['seabotix']) 
   
    @log_replay.Seabotix.thrusters_states.connect_to    forces_torques.thruster_samples
    @log_replay.detector.marker_poses.connect_to        adap_samples.position_samples
    forces_torques.forces.connect_to                    adap_samples.forces_samples 
    
        
    adap_samples.forces.connect_to              model.cmd_in,           :type => :buffer, :size => 100
    
    adap_samples.velocity.connect_to            eval.measured_velocity, :type => :buffer, :size => 100
    model.cmd_out.connect_to                    eval.model_velocity,    :type => :buffer, :size => 100         
    
    adap_samples.step = 0.1
    adap_samples.delay = 2.2 #1.7 #1.0155
    adap_samples.number_samples = 170
    eval.dof = :SURGE

    eval.aggregator_max_latency = 0.5
    eval.measured_velocity_period = 0.001
    eval.model_velocity_period = 0.01        
 
   ## Defining the proxy for each task 
   inputproxy = Orocos::Async.proxy("adap_samples")
   speedproxy = Orocos::Async.proxy("motion_model")
   evalproxy = Orocos::Async.proxy("evaluation")
   
	
   ## Defining the port variables using the proxys
  
   velocityModelport            = speedproxy.port("cmd_out")
   velocityMeasuredport         = inputproxy.port("velocity")
   errorvport                   = evalproxy.port("error_velocity")
   maevport                     = evalproxy.port("mae_velocity")
   normmaevport                 = evalproxy.port("norm_mae_velocity")    
        
    #open control widget and start replay
  #  supervisory = Supervisory.new()
  #  supervisory.evaluation(velocityModelport,velocityMeasuredport, errorvport, maevport, normmaevport)
    

    
    forces_torques.configure
    adap_samples.configure      
    model.configure
    eval.configure
    
    eval.start
    adap_samples.start
    forces_torques.start
    model.start
    
      
       
                
  #  supervisory.show
   
   
   
   Vizkit.control @log_replay
   Vizkit.exec
   
   #@log_replay.run#(true,1)
   
   
end
