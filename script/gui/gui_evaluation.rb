###########################################################################
##                                                                       ##
##                      SUPERVISORY SYSTEM CLASS                         ##
##                                                                       ##
###########################################################################


class Supervisory
  
 def initialize(parent = nil)

   @window = Vizkit.load(File.join(File.dirname(__FILE__), 'Evaluation.ui'), parent)
    
      
 
  ##########################################################################
  #                     CONFIGURING PLOT2D OBJETCS                          #
  ##########################################################################

   @plotArray = [ @window.v,     @window.error_v,     @window.maev,    @window.normmaev]    
     

   for i in 0..(@plotArray.size - 1) 
        @plotArray[i].options[:auto_scrolling_y] = true    
        @plotArray[i].options[:time_window] = 50        
        @plotArray[i].getLegend.setVisible(false)          
        @plotArray[i].options[:update_time] = @stime
   end    

   @window.v.setTitle("Velocity")
   @window.error_v.setTitle("Error Velocity")
   @window.maev.setTitle("MAE velocity")
   @window.normmaev.setTitle("Normalized error")

   
   
 
 end

  def v_model (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.v.update(sample.velocity[0], "Velocity model")
        @window.v.set_y_axis_scale(-0.6, 0.6)
        #@window.v.getLegend.setVisible(true)     
    end
  end  
  
  def v_measured (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.v.update(sample.velocity[0], "Velocity measured")
        @window.v.set_y_axis_scale(-0.6, 0.6) 
        #@window.v.getLegend.setVisible(true)                           
    end
  end 
  
  def error_v (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.error_v.update(sample, "Error velocity (m/s or rad/s)")
        @window.error_v.set_y_axis_scale(-0.5, 0.5)                       
    end
  end 
  
  def mae_v (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.maev.update(sample, "MAE velocity")
        @window.maev.set_y_axis_scale(-0.5, 0.5) 
        @window.mae_v.display sample                     
    end
  end 
  
  def norm_mae_v (port)
    @port = port
    @port.connect_to do |sample, _|
        @window.normmaev.update(sample, "Norm MAE velocity")
        @window.normmaev.set_y_axis_scale(-0.5, 0.5) 
        @window.norm_mae_v.display sample                     
    end
  end 
  
  
   
    
  def evaluation (vmodel, vmeasured, error, mae, normmae)
        
        v_model (vmodel)
        v_measured (vmeasured)
        error_v (error)
        mae_v (mae)
        norm_mae_v (normmae)
        
  end      
        
 

  def show
    @window.show
  end

  
end
