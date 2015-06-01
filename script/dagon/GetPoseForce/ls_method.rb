#library for displaying data
require 'orocos'
require 'vizkit'
#require './adap_properties.rb'
#require './../gui/gui_parameters.rb'

include Orocos


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

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1615/linY.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1635/angZ.log")

#@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1703/angZ.log")

@log_replay = Orocos::Log::Replay.open("../../../../../../../Log_files/dagon_logFiles/dagon_model_parameter_estimation/20150414-1724/angZ.log")

#######################################################################


Orocos.run 'adap_samples_input::GetPoseForce' => 'adap_samples',
           'ls_pseudoinverse::Task' => 'ls_method' do 

    adap_samples = TaskContext.get 'adap_samples'
    ls = TaskContext.get 'ls_method'
    
    adap_samples.apply_conf_file('config/GetPoseForce.yml',['dagon_ls_method'])
    
    #ls.apply_conf_file('config/ls_pseudoinverse::Task.yml',['surge'])
    #ls.apply_conf_file('config/ls_pseudoinverse::Task.yml',['sway'])
    #ls.apply_conf_file('config/ls_pseudoinverse::Task.yml',['heave'])
    #ls.apply_conf_file('config/ls_pseudoinverse::Task.yml',['pitch'])
    ls.apply_conf_file('config/ls_pseudoinverse::Task.yml',['yaw'])



    #@log_replay.pose_estimator.pose_samples.connect_to  adap_samples.pose_samples
    @log_replay.pose_estimation.pose_samples.connect_to  adap_samples.pose_samples
    @log_replay.dispatcher.all_joint_state.connect_to    adap_samples.thruster_samples
    
        
    adap_samples.dynamic.connect_to ls.dynamic_samples,  :type => :buffer, :size => 500    
    
  
   
     
        
    adap_samples.configure
    ls.configure
    adap_samples.start 
    ls.start
 
    
    Vizkit.control @log_replay
    Vizkit.exec
end
