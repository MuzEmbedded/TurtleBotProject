clear all;
clc;

%Just setting up so that matlab connect to ROS 
setenv('ROS_MASTER_URI', 'http://localhost:11311');
rosinit;


% Create a subscriber and publisher
depthImageSub = rossubscriber('/turtlebot2/camera/depth/image_raw', 'sensor_msgs/Image');
camera_info = rossubscriber("/turtlebot2/camera/rgb/image_raw",'sensor_msgs/Image');
cmdVelPub = rospublisher("/turtlebot2/cmd_vel",'geometry_msgs/Twist');


%Create array for frame 1 and frame 2 to use for SURF
frame_1 = [];
frame_2 = [];


%CmdVelMsg is the message for cmdVelpub
cmdVelMsg = rosmessage(cmdVelPub);


%Obtained the desired frame to compare with the rest, call it frame 1
frame_1 = receive(camera_info);




%Setting up the intergal and derivative for control system, don't need to
%adjust this
intergal_distance_different =0;
derivative_distance_different =0;
intergal_angular_displacement =0;
derivative_angular_displacement =0; 
intergal_translation_x =0;
derivative_translation_x =0;
translation_x_old =0;
translation_x =0;
b_old =0;
distance_different_old=0;
%The main loop contain 2 mode, 1 mode maintain distance and tracking, and
%the other mode mainly used for tracking in case the marker for depth value
%is NaN
while true
    % Receive depth image
    depthImage = receive(depthImageSub);
    

    % Read the depth data (x,y value that display the distanc)
    depthData = double(readImage(depthImage));


    %place the marker at the center of the depth camera
    markerX = 540;  
    markerY = 960;  
    

    % Get the depth value at the marker position
    currentDepth = round(depthData(markerX, markerY),2);  
    
   
    % fixing offset, and then find the different between current and
    % desired distance
    currentDistance = currentDepth+0.07  
    Desired_distance = 1.0;
    distance_different = currentDistance - Desired_distance; 
    
   
  
    
    % Distance less than 0.01 can be considered to be 0.
    if abs(distance_different) <= 0.01
        distance_different=0;
    end 
    

   
    

    % activate this mode when the marker become NaN
    if isnan(distance_different) 
        %main purpose of these 0 are to reset the gain from control system
        cmdVelMsg.Linear.X =0; 
        cmdVelMsg.Linear.Y  =0;
        intergal_distance_different =0;
        derivative_distance_different =0;
        intergal_angular_displacement =0;
        derivative_angular_displacement =0;
        intergal_translation_x =0;
        derivative_translation_x =0;
        intergal_translation_x_NaN=0;
        derivative_translation_x_NaN=0;
        translation_x_NaN_old =0;
        a_old =0;
      while true 

      
     %the k value is used for control system. kp is gain, kd is derivative,
     %ki is intergrate, adjust this value as need. 
            kpa=2.9; 
            kda=0.1;
            kia=0;

            kp1=0.5;
            kd1=0.2;
            ki1=0.1;


            %SURF function don't need to change this           
            [frame1,frame2,inlierframe2,inlierframe1] = SURF_tracking(camera_info,frame_1,frame_2);
        
            

            % see the different in x between 2 frame, and adjust the 'y'
            % velocity of the turtlebot
            translation_x_NaN = (inlierframe2.Location(:,1) - inlierframe1.Location(:,1))/1000; %Find the different
            [~, max_x] = max(translation_x_NaN); % This line and below use to identify the largest value (Usaually error) and set it to 0
            translation_x_NaN(max_x)=0;
            translation_x_NaN = mean(translation_x_NaN) %Find the average different 

            if translation_x_NaN*translation_x_NaN_old < 0 %This is to reset the control system value in case the translation_x change direction
                intergal_translation_x_NaN =0;
                derivative_translation_x_NaN=0;
            end
            %Setting up the control system including interative and
            %derivative
            intergal_translation_x_NaN =   translation_x_NaN + intergal_translation_x_NaN;
            derivative_translation_x_NaN= translation_x_NaN - derivative_translation_x_NaN;



            %Obtain the different in radian from frame 1 and frame2 
            different_in_orientation = inlierframe1.Orientation-inlierframe2.Orientation;
           [~, maxIndex] = max(different_in_orientation); %this and the code below it is used to remove the largest value which usually is not correct.
           different_in_orientation(maxIndex)=0;
            Angular_displacement_NaN = mean(abs(different_in_orientation)) %Just find the average radian different

            if Angular_displacement_NaN >= 0.5  % if the angular different is too large usually due to error then this will restrict the value. 
                Angular_displacement_NaN =0.5;
            end

      
  
                         
        
          
          %Rotating to reach the desired orientation
           a=kpa*Angular_displacement_NaN + kda*derivative_angular_displacement+kia*intergal_angular_displacement;


            if a >= 0.3 %if the different radian different is larger than 0.2 then the maximum radian rotating is 0.15 (this is to avoid error due to outline)
               a =0.3;
            end

           if mean(inlierframe2.Location(:,1) -inlierframe1.Location(:,1)) > 0 %By using the different in x value -> determine rotating left or right
               a=-a;
           else 
               a=a;
           end

            if a_old*a < 0 %This is to reset the control system value in case the angular change change direction
                intergal_angular_displacement =0;
                derivative_angular_displacement=0;
            end


          % this is just control system don't need to change. 
            intergal_angular_displacement   = intergal_angular_displacement + Angular_displacement;
            derivative_angular_displacement = Angular_displacement - derivative_angular_displacement;


            cmdVelMsg.Angular.Z = a; %rotate around z axis


            %move in x direction, can adjust this value as need.
            cmdVelMsg.Linear.X  = 0.1;
            
         d= 0.8*translation_x_NaN+0.1*intergal_translation_x_NaN +0*derivative_translation_x_NaN; %This is the control equation for 'y' velocity
         
         %Threshold for the translation x during NaN
         if translation_x_NaN >0.02 %if the value is lowewr than -0.03 then d=-d, and larger then d=d
        
          if d <=-0.4            %This if just to control the value to not jump too high
            d=-0.4;
         end
         cmdVelMsg.Linear.Y  =  -d;

        elseif translation_x_NaN <-0.02 
           d= d;
        if d >= 0.4
            d=0.4;
        end
         cmdVelMsg.Linear.Y  =  d;        
       
       
        elseif (-0.02<translation_x_NaN)&&(translation_x_NaN<0.02)
            cmdVelMsg.Linear.Y  =0;
         end 

         % to test the current value and the new value to see if the sign change
         translation_x_NaN_old = translation_x_NaN;
        
         a_old =a; %setting up to reset control system

           send(cmdVelPub,cmdVelMsg); %Send message to move the turtle bot
           showdetails(cmdVelMsg); %use to show x,y,z linear and angular value of the turtlebot
           

           % if the radian different is less than 0.1 then go into the
           % other mode
           if ~isnan(currentDistance) || Angular_displacement_NaN <=0.03
                 break
           end
           pause (0.1);
     
      end


      %this is the second mode: constant distant and tracking when marker
      %is not NaN
   else

        %setting up control gain
        kp=0.7;
        kd=0.15;
        ki=0;
        cmdVelMsg = rosmessage(cmdVelPub);
   

         frame_2 = receive(camera_info);
         kpa =2.5;
         kia =0.1;
         kda = 0;%proportional (kp), Integral (ki), and Derivative (kd) Gains:

   % Fine-tune the values of kp, ki, and kd based on your specific system's behavior. 
         if distance_different_old* distance_different< 0 %This is to reset the control system value in case the angular change change direction
                intergal_distance_different =0;
                derivative_distance_different=0;
      end
   
        %Just control system, don't change this    
        intergal_distance_different = intergal_distance_different + distance_different;
        derivative_distance_different = distance_different-derivative_distance_different;
   
        %Move in x direction
        f=kp*distance_different + kd*derivative_distance_different + ki*intergal_distance_different;
        cmdVelMsg.Linear.X  = f;


         %SURF function
     [frame1,frame2,inlierframe2,inlierframe1] = SURF_tracking(camera_info,frame_1,frame_2);

    %Checking the SURF function, can comment out if needed. 

    

    %Setting up the difference for the control system
    different_in_orientation = inlierframe1.Orientation-inlierframe2.Orientation;


    [~, maxIndex] = max(different_in_orientation);
    different_in_orientation(maxIndex)=0;
    Angular_displacement = mean(abs(different_in_orientation))

    if Angular_displacement >= 0.2
       Angular_displacement =0.2;
    end



        % Setting up control system
    intergal_angular_displacement   = intergal_angular_displacement + Angular_displacement;
    derivative_angular_displacement = Angular_displacement - derivative_angular_displacement;
    
      
      % Threshold setup for angular velocity
        b= round(kpa*Angular_displacement+kia*intergal_angular_displacement+kda*derivative_angular_displacement,2);
       
        if b>0.3
             b=0.3;
         end
          if mean(inlierframe2.Location(:,1) -inlierframe1.Location(:,1)) >0
               b=-b;
           else 
               b=b;
          end
         if  (Angular_displacement <0.03) 
            b=0;
        end 

      if b_old*b < 0 %This is to reset the control system value in case the angular change change direction
                intergal_angular_displacement =0;
                derivative_angular_displacement=0;
      end
    
         %Rotate around Z axis 

        cmdVelMsg.Angular.Z = b;
        
        
    
              
        %Setting up the average value for translation x and the control
        %value
        translation_x = (inlierframe2.Location(:,1) - inlierframe1.Location(:,1))/1000;
            [~, max_x] = max(translation_x);
            translation_x(max_x)=0;
            translation_x = mean(translation_x)
         
            if translation_x*translation_x_old < 0 %This is to reset the control system value in case the translation_x change direction
                intergal_translation_x =0;
                derivative_translation_x=0;
            end
            intergal_translation_x =   translation_x + intergal_translation_x;
            derivative_translation_x= translation_x - derivative_translation_x;
         c=  0.7*translation_x + 0.1*intergal_translation_x + 0*derivative_translation_x ;
         
        
        %Setting threshold for translation_x
        if translation_x <-0.03
     
          if c >0.3           
            c=0.3;
         end
         cmdVelMsg.Linear.Y  =  c;
        end

        if translation_x >0.03
 
        if c<-0.3
            c=-0.3;
        end
         cmdVelMsg.Linear.Y  =  -c;        
        end
        
        if ((-0.03<translation_x)&&(translation_x<0.03)) 
            cmdVelMsg.Linear.Y  =0;
        end 
        
        %setting up to reset control system
        b_old =b; 
        translation_x_old = translation_x;
        distance_different_old = distance_different;
        send(cmdVelPub,cmdVelMsg); %Publish message to turtlebot
        pause(0.1);
    end



    
end