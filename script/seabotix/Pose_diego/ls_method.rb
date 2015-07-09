#library for displaying data
require 'orocos'
require 'vizkit'
#require './../gui/gui_parameters.rb'

include Orocos

require "readline"

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/surge/linX.log")

@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/yaw_1/angZ_1.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/yaw_2/angZ_2.log")



Orocos.run 'adap_parameters_estimator::ForceApplier' => 'forces&torques',
           'adap_samples_input::Seabotix' => 'adap_samples',
           'ls_pseudoinverse::Task' => 'ls_method' do


    adap_samples = TaskContext.get 'adap_samples'
    ls = TaskContext.get 'ls_method'
    forces_torques = TaskContext.get 'forces&torques'
    
    forces_torques.apply_conf_file('config/adap_parameters_estimator::ForceApplier.yml',['seabotix']) 

    @log_replay.Seabotix.thrusters_states.connect_to forces_torques.thruster_samples, :type => :buffer, :size => 100000
    @log_replay.detector.marker_poses.connect_to adap_samples.position_samples,  :type => :buffer, :size => 100000
    forces_torques.forces.connect_to adap_samples.forces_samples,  :type => :buffer, :size => 100000
    
    adap_samples.dynamic.connect_to ls.dynamic_samples,  :type => :buffer, :size => 100000    
    adap_samples.step = 0.1
    adap_samples.delay = 1.65 #1.7 #1.0155
    #number = (Math::PI/2)/0.1
    adap_samples.number_samples = 100
    ls.dof = :YAW    
     
        
    adap_samples.configure
    forces_torques.configure
    ls.configure
    adap_samples.start 
    forces_torques.start
    ls.start

#    supervisory.show
    
    Vizkit.control @log_replay
    Vizkit.exec
    #@log_replay.run(true,1)
    #Readline.readline
end
