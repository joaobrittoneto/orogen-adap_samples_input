#library for displaying data
require 'orocos'
require 'vizkit'
#require './../gui/gui_parameters.rb'

include Orocos


#@delay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/delay1/delay1.log")

@delay = Orocos::Log::Replay.open("../../../../../../../Log_files/seabotix_logFiles/seabotix_pose_diego/delay2/delay2.log")



Orocos.run do #'adap_parameters_estimator::ForceApplier' => 'forces&torques',
 #          'adap_samples_input::Seabotix' => 'adap_samples',
 #          'ls_pseudoinverse::Task' => 'ls_method' do

        
   widget = Vizkit.default_loader.Plot2d
   widget2 = Vizkit.default_loader.Plot2d

   @start_counting = true
   @stop_counting  = true 
   @first_time  = true   
   @init_time = Time.new
   @end_time = Time.new 
    
   @delay.Seabotix.thrusters_states.connect_to do |sample|
        widget.update(sample.elements[0].raw , "thruster")
        if sample.elements[0].raw > 0 and @start_counting
           @init_time = sample.time 
           @start_counting = false 
           print "trhuster: "
           puts sample.elements[0].raw
           puts @init_time
        end
    end 
    
   @delay.detector.marker_poses.connect_to do |sample|
       widget.update(sample.position[0], "postion [m]")
       if @first_time
        @last_sample = sample.position[0]
        @first_time = false
       end
       if (sample.position[0] - @last_sample).abs > sample.position[0].abs/10 and @stop_counting 
           @end_time = sample.time 
           @stop_counting = false 
           print "pose: "
           puts sample.position[0]
           puts @end_time
       end
       if !@stop_counting and !@start_counting
         d = @end_time - @init_time
         print "delay: "
         puts d
       end 
    end
    
    widget.show    
    
       
    Vizkit.control @delay
    Vizkit.exec
end
