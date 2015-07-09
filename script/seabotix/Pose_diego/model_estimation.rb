#library for displaying data
require 'orocos'
require 'vizkit'
require './../../gui/gui_parameters.rb'

include Orocos


@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/surge/linX.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/yaw_1/angZ_1.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/yaw_2/angZ_2.log")


Orocos.run 'adap_parameters_estimator::ForceApplier'            => 'forces&torques',
           'adap_samples_input::Seabotix'                       => 'adap_samples',
           'ls_pseudoinverse::Task'                             => 'ls_method',
           'adap_parameters_estimator::AdapModelEstimation'     => 'adap_model',
           'uwv_motion_model::Task'                             => 'motion_model',
           'adap_parameters_estimator::Evaluation'              => 'evaluation' do
           
    Orocos.log_all

    adap_samples   = TaskContext.get 'adap_samples'
    model          = TaskContext.get 'motion_model'
    ls             = TaskContext.get 'ls_method'
    forces_torques = TaskContext.get 'forces&torques'
    adap_model     = TaskContext.get 'adap_model'
    eval           = TaskContext.get 'evaluation'
    
    adap_model.apply_conf_file('config/adap_parameters_estimator::AdapModelEstimation.yml',['default', 'surge'])
    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['seabotix'])
    model.apply_conf_file('config/uwv_motion_model::Task.yml',['seabotix']) 
    
    @log_replay.Seabotix.thrusters_states.connect_to    forces_torques.thruster_samples, :type => :buffer, :size => 100
    @log_replay.detector.marker_poses.connect_to        adap_samples.position_samples,  :type => :buffer, :size => 100
    forces_torques.forces.connect_to                    adap_samples.forces_samples,  :type => :buffer, :size => 100

    adap_samples.velocity.connect_to    adap_model.pose_samples,        :type => :buffer, :size => 100 
    adap_samples.forces.connect_to      adap_model.forces_samples,      :type => :buffer, :size => 100     
    adap_samples.dynamic.connect_to     ls.dynamic_samples,  :type => :buffer, :size => 100    
        
    adap_samples.forces.connect_to              model.cmd_in,           :type => :buffer, :size => 100
    adap_samples.velocity.connect_to            model.sync_pose,        :type => :buffer, :size => 10
    adap_samples.velocity.connect_to            eval.measured_velocity, :type => :buffer, :size => 100
    model.cmd_out.connect_to                    eval.model_velocity,    :type => :buffer, :size => 100     
        
    adap_samples.step = 0.1
    adap_samples.delay = 2.2 #1.65 #1.0155
    #number = (Math::PI/2)/0.1
    adap_samples.number_samples = 170 #surge
#    adap_samples.number_samples = 80   #yaw 
    ls.dof = :SURGE 
    adap_model.aggregator_max_latency = 0.5
    adap_model.pose_samples_period = 0.0001
    adap_model.forces_samples_period = 0.01
    adap_model.sTime = 2.0
    
    model.sync = true 
    eval.dof = :SURGE
    eval.aggregator_max_latency = 0.5
    eval.measured_velocity_period = 0.001
    eval.model_velocity_period = 0.01     
       
 
    adap_samples.configure
    adap_model.configure
    forces_torques.configure
    ls.configure
    model.configure
    eval.configure
    
    forces_torques.start
    ls.start
    adap_samples.start 
    adap_model.start
    model.start
    eval.start


   ## Defining the proxy for each task 
   parametersproxy = Orocos::Async.proxy("adap_model")
   inputproxy = Orocos::Async.proxy("adap_samples")
   speedproxy = Orocos::Async.proxy("motion_model")
   
   
	
   ## Defining the port variables using the proxys
   deltaVport       = parametersproxy.port("deltaV")
   nomrDeltaVport   = parametersproxy.port("normDeltaV")
   parametersport   = parametersproxy.port("parameters")
   velocityport     = inputproxy.port("velocity")
   forcesport       = inputproxy.port("forces")
   
     
    #open control widget and start replay
    supervisory = Supervisory.new(adap_model)
    supervisory.evolution(deltaVport, nomrDeltaVport, velocityport, forcesport, parametersport)
    
        
    supervisory.show
    
    Vizkit.control @log_replay
    Vizkit.exec
end
