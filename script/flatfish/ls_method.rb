#library for displaying data
require 'orocos'
require 'vizkit'
#require './adap_properties.rb'
require './../gui/gui_parameters.rb'

include Orocos


#load log file 
#######################################################################

@log_replay = Orocos::Log::Replay.open("../../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linX.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linY.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/linZ.log")
#@log_replay = Orocos::Log::Replay.open("../../../../../../../../media/joao/1E24C6A424C67E71/AUV_Log_files/Flatfish/20150708-1920/angZ.log")

#######################################################################


Orocos.run 'adap_samples_vel', #'adap_samples_pose',
           'adap_parameters_estimator::ForceApplier'    => 'forces&torques',
           'filter_apriltag_detector::Task'                 => 'filter' do
        #   'filter_apriltag_detector::AngVel'               => 'ang_vel'  do 

    forces_torques      = TaskContext.get 'forces&torques'
    adap_samples_vel    = TaskContext.get 'adap_samples_vel'
    ls_vel              = TaskContext.get 'ls_method_vel'  
#    filter              = TaskContext.get 'filter' 
 #   ang_vel             = TaskContext.get 'ang_vel'       
#    adap_samples_pose   = TaskContext.get 'adap_samples_pose'
#    ls_pose             = TaskContext.get 'ls_method_pose'

    
    
 #   Orocos.log_all
    
    #########################################################
    pose_estimator      = @log_replay.pose_estimator
    ori_estimator        = @log_replay.orientation_estimator    
    cam_pose            = @log_replay.apriltag_detector 
    thurster            = @log_replay.acceleration_controller           
    ######################################################### 
            
    forces_torques.apply_conf_file('../../../adap_parameters_estimator/scripts/config/adap_parameters_estimator::ForceApplier.yml',['flatfish']) 
   
      dof = "surge" 
    
   adap_samples_vel.apply_conf_file('./config/adap_samples_input::Task.yml',['default'])
   ls_vel.apply_conf_file('./config/ls_pseudoinverse::Task.yml',[dof])

#    adap_samples_pose.apply_conf_file('./config/adap_samples_input::Task.yml',['cam_pose'])
#    adap_samples_pose.apply_conf_file('../../../adap_parameters_estimator/scripts/config/adap_samples_input::Task.yml',['cam_pose'])
#    filter.apply_conf_file('../../../adap_parameters_estimator/scripts/config/filter.yml',['default'])
#    ls_pose.apply_conf_file('./config/ls_pseudoinverse::Task.yml',[dof])




    thurster.cmd_out.connect_to                 forces_torques.thruster_samples  
    
 #   if dof == 'yaw'
 #       ori_estimator.orientation_samples_out.connect_to      ang_vel.orientation_sample,     :type => :buffer, :size => 100
 #       pose_estimator.pose_samples.connect_to                ang_vel.pose_sample,            :type => :buffer, :size => 100  
 #       ang_vel.output.connect_to                             adap_samples_vel.pose_samples,  :type => :buffer, :size => 100 
#    else             
#        pose_estimator.pose_samples.connect_to      adap_samples_vel.pose_samples,            :type => :buffer, :size => 100 
#    end
   forces_torques.forces.connect_to            adap_samples_vel.forces_samples,        :type => :buffer, :size => 100       
    pose_estimator.pose_samples.connect_to      adap_samples_vel.pose_samples,          :type => :buffer, :size => 100 
    
#    forces_torques.forces.connect_to            adap_samples_pose.forces_samples,       :type => :buffer, :size => 100       
#    cam_pose.marker_poses.connect_to            filter.pose_sample
#    filter.output.connect_to                    adap_samples_pose.pose_samples,         :type => :buffer, :size => 100    
    
    adap_samples_vel.dynamic.connect_to         ls_vel.dynamic_samples,                 :type => :buffer, :size => 500    
#    adap_samples_pose.dynamic.connect_to        ls_pose.dynamic_samples,                :type => :buffer, :size => 500    
    
  
    forces_torques.configure      
    adap_samples_vel.configure 
    ls_vel.configure        
#    adap_samples_pose.configure    
#    ls_pose.configure 
#    filter.configure
        
 #   if dof == 'yaw'
 #           ang_vel.configure    
 #           ang_vel.start
 #   end
    
    forces_torques.start
    adap_samples_vel.start
    ls_vel.start 
#    adap_samples_pose.start
#    ls_pose.start     
#    filter.start
    

        
    
    Vizkit.control @log_replay
    Vizkit.exec
end
