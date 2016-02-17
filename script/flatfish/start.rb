#library for displaying data
require 'vizkit'
#require './../../../adap_paramters_estimator/gui/gui_parameters.rb'
include Orocos

#Orocos::CORBA.name_service = "192.168.128.51"  # Avalon


#load log file 
#######################################################################

@log_replay = Orocos::Log::Replay.open("../../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linX.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linY.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linZ.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/angZ.log")

#######################################################################

Orocos.run 'adap_parameters_estimator::ForceApplier'        => 'forces&torques',
           'adap_samples_input::Task'                       => 'adap_samples' do


    adap_samples = TaskContext.get 'adap_samples'
    forces_torques = TaskContext.get 'forces&torques'
        
    forces_torques.apply_conf_file('../../../adap_parameters_estimator/scripts/config/adap_parameters_estimator::ForceApplier.yml',['flatfish']) 
    adap_samples.apply_conf_file('./config/adap_samples_input::Task.yml',['default'])
        
    #Readline.readline
    

    #########################################################
    pose_estimator     = @log_replay.pose_estimator 
    thurster            = @log_replay.acceleration_controller     
    #########################################################    

    thurster.cmd_out.connect_to                 forces_torques.thruster_samples  
    forces_torques.forces.connect_to            adap_samples.forces_samples,    :type => :buffer, :size => 100       
    pose_estimator.pose_samples.connect_to      adap_samples.pose_samples,        :type => :buffer, :size => 100  
     
    adap_samples.aggregator_max_latency = 2.0
    adap_samples.pose_samples_period = 0.0001
    adap_samples.forces_samples_period = 0.0001
    
    forces_torques.configure      
    adap_samples.configure
    forces_torques.start
    adap_samples.start
     
    

    #Readline.readline
        
    #open control widget and start replay
    Vizkit.control @log_replay
    Vizkit.exec
    
end
