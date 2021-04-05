% The simulator is based on a 3D Matlab Kinematic model of a Puma robot located
% in the robotics lab of Walla Walla University. Robot geometry uses the CAD2MATDEMO code in the Mathworks file exchange

function robot_arm
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Init
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load 3D CAD model
[linkdata]=load('linksdata.mat','s1','s2', 's3','s4','s5','s6','s7','A1');
% load predictions on the test set
svm_predictions_spectrograms=load('svm_predictions_td_features.csv');
mlp_predictions_spectrograms=load('mlp_predictions_spectrograms.csv');
cnn_predictions_spectrograms=load('cnn_predictions_spectrograms.csv');
lstm_predictions_spectrograms=load('lstm_predictions_spectrograms.csv');
% Store links
setappdata(0,'Link1_data',linkdata.s1);
setappdata(0,'Link2_data',linkdata.s2);
setappdata(0,'Link3_data',linkdata.s3);
setappdata(0,'Link4_data',linkdata.s4);
setappdata(0,'Link5_data',linkdata.s5);
setappdata(0,'Link6_data',linkdata.s6);
setappdata(0,'Link7_data',linkdata.s7);
setappdata(0,'Area_data',linkdata.A1);
% init GUI
set(0,'Units','pixels')
dim = get(0,'ScreenSize');
fig_1 = figure('doublebuffer','on','Position',[450,300,1200,600],...
    'Name','Robot Arm Simulator',...
    'NumberTitle','off');
hold on;
fig_2 = 0;
light
daspect([1 1 1])
view(135,25)
xlabel('X'),ylabel('Y'),zlabel('Z');
title('EMG-based robotic control - ITR');
axis([-1500 1500 -1500 1500 -1120 1500]);
plot3([-1500,1500],[-1500,-1500],[-1120,-1120],'k')
plot3([-1500,-1500],[-1500,1500],[-1120,-1120],'k')
plot3([-1500,-1500],[-1500,-1500],[-1120,1500],'k')
plot3([-1500,-1500],[1500,1500],[-1120,1500],'k')
plot3([-1500,1500],[-1500,-1500],[1500,1500],'k')
plot3([-1500,-1500],[-1500,1500],[1500,1500],'k')

s1 = getappdata(0,'Link1_data');
s2 = getappdata(0,'Link2_data');
s3 = getappdata(0,'Link3_data');
s4 = getappdata(0,'Link4_data');
s5 = getappdata(0,'Link5_data');
s6 = getappdata(0,'Link6_data');
s7 = getappdata(0,'Link7_data');
A1 = getappdata(0,'Area_data');
%
a2 = 650;
a3 = 0;
d3 = 190;
d4 = 600;
Px = 5000;
Py = 5000;
Pz = 5000;

%The 'home' position, for init.
t1 = 90;
t2 = -90;
t3 = -90;
t4 = 0;
t5 = 0;
t6 = 0;

% Forward Kinematics
T_01 = tmat(0, 0, 0, t1);
T_12 = tmat(-90, 0, 0, t2);
T_23 = tmat(0, a2, d3, t3);
T_34 = tmat(-90, a3, d4, t4);
T_45 = tmat(90, 0, 0, t5);
T_56 = tmat(-90, 0, 0, t6);

% Each link fram to base frame transformation
T_02 = T_01*T_12;
T_03 = T_02*T_23;
T_04 = T_03*T_34;
T_05 = T_04*T_45;
T_06 = T_05*T_56;

% Actual vertex data of robot links
Link1 = s1.V1;
Link2 = (T_01*s2.V2')';
Link3 = (T_02*s3.V3')';
Link4 = (T_03*s4.V4')';
Link5 = (T_04*s5.V5')';
Link6 = (T_05*s6.V6')';
Link7 = (T_06*s7.V7')';

% points are no fun to watch, make it look 3d.
L1 = patch('faces', s1.F1, 'vertices' ,Link1(:,1:3));
L2 = patch('faces', s2.F2, 'vertices' ,Link2(:,1:3));
L3 = patch('faces', s3.F3, 'vertices' ,Link3(:,1:3));
L4 = patch('faces', s4.F4, 'vertices' ,Link4(:,1:3));
L5 = patch('faces', s5.F5, 'vertices' ,Link5(:,1:3));
L6 = patch('faces', s6.F6, 'vertices' ,Link6(:,1:3));
L7 = patch('faces', s7.F7, 'vertices' ,Link7(:,1:3));
A1 = patch('faces', A1.Fa, 'vertices' ,A1.Va(:,1:3));
Tr = plot3(0,0,0,'b.'); % holder for trail paths
%
setappdata(0,'patch_h',[L1,L2,L3,L4,L5,L6,L7,A1,Tr])
%
setappdata(0,'xtrail',0); % used for trail tracking.
setappdata(0,'ytrail',0); % used for trail tracking.
setappdata(0,'ztrail',0); % used for trail tracking.
%
set(L1, 'facec', [0.717,0.116,0.123]);
set(L1, 'EdgeColor','none');
set(L2, 'facec', [0.216,1,.583]);
set(L2, 'EdgeColor','none');
set(L3, 'facec', [0.306,0.733,1]);
set(L3, 'EdgeColor','none');
set(L4, 'facec', [1,0.542,0.493]);
set(L4, 'EdgeColor','none');
set(L5, 'facec', [0.216,1,.583]);
set(L5, 'EdgeColor','none');
set(L6, 'facec', [1,1,0.255]);
set(L6, 'EdgeColor','none');
set(L7, 'facec', [0.306,0.733,1]);
set(L7, 'EdgeColor','none');
set(A1, 'facec', [.8,.8,.8],'FaceAlpha',.25);
set(A1, 'EdgeColor','none');
%
setappdata(0,'ThetaOld',[90,-90,-90,0,0,0]);
%

t1_home = 90;
t2_home = -90;
t3_home = -90;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% GUI
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Window size
%
LD = 105; % Left, used to set the GUI.
HT = 18;  % Height
BT = 156; % Bottom
%%
% Create Buttons
%
    uicontrol(fig_1,'String','Control 1','callback',@ctrl1_button_press,...
        'Position',[20 130 60 20]);
    uicontrol(fig_1,'String','Control 2','callback',@ctrl2_button_press,...
    'Position',[20 105 60 20]);
    uicontrol(fig_1,'String','Control 3','callback',@ctrl3_button_press,...
        'Position',[20 80 60 20]);
    uicontrol(fig_1,'String','Control 4','callback',@ctrl4_button_press,...
        'Position',[20 55 60 20]);
    uicontrol(fig_1,'String','Control 5','callback',@ctrl5_button_press,...
        'Position',[20 30 60 20]);
    uicontrol(fig_1,'String','Control 6','callback',@ctrl6_button_press,...
        'Position',[20 5 60 20]);
    
    uicontrol(fig_1,'String','Random Prediction','callback',@pred_ctrl_button_press,...
        'Position',[100 5 100 20]);
    
    uicontrol(fig_1,'String','Random Move','callback',@random_button_press,...
        'Position',[100 30 100 20]);

    uicontrol(fig_1,'String','Home','callback',@home_button_press,...
    'Position',[220 5 60 20]);

    uicontrol(fig_1,'style','text',... %edit
    'String','Ground Truth:',...
    'Position',[20 325 100 20]); % L, from bottom, W, H

    ground_truth_text = uicontrol(fig_1,'style','text',... %edit
    'String','_',...
    'Position',[120 325 100 20]); % L, from bottom, W, H
    
    uicontrol(fig_1,'style','text',... %edit
    'String','Prediction:',...
    'Position',[20 300 100 20]); % L, from bottom, W, H
    
    pred_text = uicontrol(fig_1,'style','text',... %edit
    'String','_',...
    'Position',[120 300 100 20]); % L, from bottom, W, H

    uicontrol(fig_1,'style','text',... %edit
    'String','Model:',...
    'Position',[20 350 100 20]); % L, from bottom, W, H

    selected_model = uicontrol(fig_1,'style','popupmenu',... %edit
    'String',{'SVM','MLP','CNN', 'LSTM'},...
    'Position',[120 350 100 20]); % L, from bottom, W, H
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% EMG Maps - Target positions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Mapping
%
    function [theta1,theta2,theta3,theta4,theta5,theta6] =  emg_joints_mapper(motion_class)
        %if motion_class == 10 %right
        %    [theta1,theta2,theta3,theta4,theta5,theta6] = Inverse_Kinematics(1250,0,0);
        %    theta3 = theta3-180;
        if motion_class == 1 %reflexion
            [theta1,theta2,theta3,theta4,theta5,theta6] = Inverse_Kinematics(0,1250,0);
            theta3 = theta3-180;
        elseif motion_class == 2 %flexion
            [theta1,theta2,theta3,theta4,theta5,theta6] = Inverse_Kinematics(0,1250,0);
            theta3 = 0;
            theta3 = theta3-180;
        elseif motion_class == 3 %down
            theta1 = 0; theta2 = 90; theta3 = -90; theta4 = 0; theta5 = 0; theta6 = 0;
        elseif motion_class == 4 %left
            [theta1,theta2,theta3,theta4,theta5,theta6] = Inverse_Kinematics(1250,0,0);
            theta3 = theta3-180;
        elseif motion_class == 5 %down_2
            theta1 = 90; theta2 = 90; theta3 = -90; theta4 = 0; theta5 = 0; theta6 = 0;
            
        elseif motion_class == 6 %forward
            theta1 = 90; theta2 = -45; theta3 = -90; theta4 = 0; theta5 = 0; theta6 = 0;
        
        %elseif motion_class == 20 %backwards
        %    [theta1,theta2,theta3,theta4,theta5,theta6] = Inverse_Kinematics(0,-1250,0);
        %    theta3 = theta3-180;
        
        end
        
    end
    
%%
%% Control
%
    function ctrl1_button_press(h,dummy)
        if fig_2 ~= 0
            close(fig_2)
        end
        fig_2 = figure('doublebuffer','on','Position',[150,300,300,600],...
            'Name','Human Control',...
            'NumberTitle','off');
        image(imread('0.png'));
        hold off;
        [theta1,theta2,theta3,theta4,theta5,theta6] = emg_joints_mapper(1)
        Forward_Kinematics(theta1,theta2,theta3,0,0,0,30,'y')
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        
        
        
    end
    function ctrl2_button_press(h,dummy)
        if fig_2 ~= 0
            close(fig_2)
        end
        fig_2 = figure('doublebuffer','on','Position',[150,300,300,600],...
            'Name','Human Control',...
            'NumberTitle','off');
        image(imread('1.png'));
        hold off;
        [theta1,theta2,theta3,theta4,theta5,theta6] = emg_joints_mapper(2)
        Forward_Kinematics(theta1,theta2,theta3,0,0,0,30,'y')
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        
        
        
    end
    function ctrl3_button_press(h,dummy)
        if fig_2 ~= 0
            close(fig_2)
        end
        fig_2 = figure('doublebuffer','on','Position',[150,300,300,600],...
            'Name','Human Control',...
            'NumberTitle','off');
        image(imread('2.png'));
        hold off;
        [theta1,theta2,theta3,theta4,theta5,theta6] = emg_joints_mapper(3)
        Forward_Kinematics(theta1,theta2,theta3,0,0,0,30,'y')
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        
        
        
    end
    function ctrl4_button_press(h,dummy)
        if fig_2 ~= 0
            close(fig_2)
        end
        fig_2 = figure('doublebuffer','on','Position',[150,300,300,600],...
            'Name','Human Control',...
            'NumberTitle','off');
        image(imread('3.png'));
        hold off;
        [theta1,theta2,theta3,theta4,theta5,theta6] = emg_joints_mapper(4)
        Forward_Kinematics(theta1,theta2,theta3,0,0,0,30,'y')
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        
        
        
    end
    function ctrl5_button_press(h,dummy)
        if fig_2 ~= 0
            close(fig_2)
        end
        fig_2 = figure('doublebuffer','on','Position',[150,300,300,600],...
            'Name','Human Control',...
            'NumberTitle','off');
        image(imread('4.png'));
        hold off;
        [theta1,theta2,theta3,theta4,theta5,theta6] = emg_joints_mapper(5)
        Forward_Kinematics(theta1,theta2,theta3,0,0,0,30,'y')
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        
        
        
    end
    function ctrl6_button_press(h,dummy)
        if fig_2 ~= 0
            close(fig_2)
        end
        fig_2 = figure('doublebuffer','on','Position',[150,300,300,600],...
            'Name','Human Control',...
            'NumberTitle','off');
        image(imread('5.png'));
        hold off;
        [theta1,theta2,theta3,theta4,theta5,theta6] = emg_joints_mapper(6)
        Forward_Kinematics(theta1,theta2,theta3,0,0,0,30,'y')
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        
        
        
    end
    
    function pred_ctrl_button_press(h,dummy)
        disp("--------------------------------------")
        if fig_2 ~= 0
            close(fig_2)
        end
        fig_2 = figure('doublebuffer','on','Position',[150,300,300,600],...
            'Name','Human Control',...
            'NumberTitle','off');
        
        
        disp(selected_model.Value)
        %true_ctrl = 1;
        if selected_model.Value == 1
            gt_pred = svm_predictions_spectrograms;
            disp("************************")
            gt_pred
        elseif selected_model.Value == 2
            gt_pred = mlp_predictions_spectrograms;
        elseif selected_model.Value == 3
            gt_pred = cnn_predictions_spectrograms;
        elseif selected_model.Value == 4
            gt_pred = lstm_predictions_spectrograms;
        end
        
        size_pred = size(gt_pred);
        pos = randi(size_pred(2), 1);
        ground_truth_ctrl = gt_pred(1,pos)
        pred_ctrl = gt_pred(2,pos)
        
        if ground_truth_ctrl == 0
            img = '0.png';
        elseif ground_truth_ctrl == 1
            img = '1.png';
         elseif ground_truth_ctrl == 2
            img = '2.png';
         elseif ground_truth_ctrl == 3
            img = '3.png';
         elseif ground_truth_ctrl == 4
            img = '4.png';
         elseif ground_truth_ctrl == 5
            img = '5.png';
        end
        img
        image(imread(img));
        hold off;
        
        ground_truth_text.String = ground_truth_ctrl+1;
        pred_text.String = pred_ctrl+1;
        
        
        
        handles = getappdata(0,'patch_h');           %
        Tr = handles(9);
        %
        setappdata(0,'xtrail',0); % used for trail tracking.
        setappdata(0,'ytrail',0); % used for trail tracking.
        setappdata(0,'ztrail',0); % used for trail tracking.
        %
        set(Tr,'xdata',0,'ydata',0,'zdata',0);
        [theta1,theta2,theta3,theta4,theta5,theta6] = emg_joints_mapper(pred_ctrl+1);
        Forward_Kinematics(theta1,theta2,theta3,0,0,0,30,'y')
        
        
        
    end

%
%
%%
%% Home position
%
    function home_button_press(h,dummy)

        gohome
    end
%
%%
%%
%% Clear Trail
% %
%     function clr_trail_button_press(h,dummy)
%         %disp('pushed clear trail bottom');
%         handles = getappdata(0,'patch_h');           %
%         Tr = handles(9);
%         %
%         setappdata(0,'xtrail',0); % used for trail tracking.
%         setappdata(0,'ytrail',0); % used for trail tracking.
%         setappdata(0,'ztrail',0); % used for trail tracking.
%         %
%         set(Tr,'xdata',0,'ydata',0,'zdata',0);
%     end
% %
%
%%
%% Random Move
%
function random_button_press(h, dummy)
    %disp('pushed random demo bottom');
    % a = 10; b = 50; x = a + (b-a) * rand(5)
    %     Angle    Range                Default Name
    %     Theta 1: 320 (-160 to 160)    90       Waist Joint
    %     Theta 2: 220 (-110 to 110)    -90       Shoulder Joint
    %     Theta 3: 270 (-135 to 135)    -90       Elbow Joint
    %     Theta 4: 532 (-266 to 266)    0       Wrist Roll
    %     Theta 5: 200 (-100 to 100)    0       Wrist Bend
    %     Theta 6: 532 (-266 to 266)    0       Wrist Swival
%         t1_home = 90; % offsets to define the "home" postition as UP.
%         t2_home = -90;
%         t3_home = -90;
    theta1 = -160 + 320*rand(1); % offset for home
    theta2 = -110 + 220*rand(1); % in the UP pos.
    theta3 = -135 + 270*rand(1);
    theta4 = -266 + 532*rand(1);
    theta5 = -100 + 200*rand(1);
    theta6 = -266 + 532*rand(1);
    n = 50;
    Forward_Kinematics(theta1+t1_home,theta2+t2_home,theta3+t3_home,theta4,theta5,theta6,n,'y')
end


%%
%% Close Function
%
function del_app(varargin)
    
    delete(fig_1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Utility Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Initial position
%
function gohome()
    Forward_Kinematics(90,-90,-90,0,0,0,10,'n') % show it animate home
end

%%
% Transformation Matrix (T matrix) defined in equation 3.6 in Craig's "Introduction to Robotics."
% 
function T = tmat(alpha, a, d, theta) %alpha, a, d, theta are the Denavit-Hartenberg parameters.
    alpha = alpha*pi/180;
    theta = theta*pi/180;
    c = cos(theta);
    s = sin(theta);
    ca = cos(alpha);
    sa = sin(alpha);
    T = [c -s 0 a; s*ca c*ca -sa -sa*d; s*sa c*sa ca ca*d; 0 0 0 1];
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Controller Design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Forward Kinematics
%
function Forward_Kinematics(theta1,theta2,theta3,theta4,theta5,theta6,n,trail)  
    % This function will animate the Puma 762 robot given joint angles.
    % n is number of steps for the animation
    % trail is 'y' or 'n' (n = anything else) for leaving a trail.
    %
    %disp('in animate');
    a2 = 650; %D-H paramaters
    a3 = 0;
    d3 = 190;
    d4 = 600;
    % Err2 = 0;
    %
    ThetaOld = getappdata(0,'ThetaOld');
    %
    theta1old = ThetaOld(1);
    theta2old = ThetaOld(2);
    theta3old = ThetaOld(3);
    theta4old = ThetaOld(4);
    theta5old = ThetaOld(5);
    theta6old = ThetaOld(6);
    %
    t1 = linspace(theta1old,theta1,n); 
    t2 = linspace(theta2old,theta2,n); 
    t3 = linspace(theta3old,theta3,n);% -180;  
    t4 = linspace(theta4old,theta4,n); 
    t5 = linspace(theta5old,theta5,n); 
    t6 = linspace(theta6old,theta6,n); 

    n = length(t1);
    for i = 2:1:n
        % Forward Kinematics
        %
        T_01 = tmat(0, 0, 0, t1(i));
        T_12 = tmat(-90, 0, 0, t2(i));
        T_23 = tmat(0, a2, d3, t3(i));
        T_34 = tmat(-90, a3, d4, t4(i));
        T_45 = tmat(90, 0, 0, t5(i));
        T_56 = tmat(-90, 0, 0, t6(i));

% 
%             %     T_67 = [   1            0      0 0
%             %                0            1      0 0
%             %                0            0      1 188
%             %                0            0      0 1];

        %T_01 = T_01;  % it is, but don't need to say so.
        T_02 = T_01*T_12;
        T_03 = T_02*T_23;
        T_04 = T_03*T_34;
        T_05 = T_04*T_45;
        T_06 = T_05*T_56;
        %     T_07 = T_06*T_67;
        %
        s1 = getappdata(0,'Link1_data');
        s2 = getappdata(0,'Link2_data');
        s3 = getappdata(0,'Link3_data');
        s4 = getappdata(0,'Link4_data');
        s5 = getappdata(0,'Link5_data');
        s6 = getappdata(0,'Link6_data');
        s7 = getappdata(0,'Link7_data');
        %A1 = getappdata(0,'Area_data');

        Link1 = s1.V1;
        Link2 = (T_01*s2.V2')';
        Link3 = (T_02*s3.V3')';
        Link4 = (T_03*s4.V4')';
        Link5 = (T_04*s5.V5')';
        Link6 = (T_05*s6.V6')';
        Link7 = (T_06*s7.V7')';
        %     Tool = T_07;

        %     if sqrt(Tool(1,4)^2+Tool(2,4)^2)<514
        %         Err2 = 1;
        %         break
        %     end
        %
        handles = getappdata(0,'patch_h');           %
        L1 = handles(1);
        L2 = handles(2);
        L3 = handles(3);
        L4 = handles(4);
        L5 = handles(5);
        L6 = handles(6);
        L7 = handles(7);
        Tr = handles(9);
        %
        set(L1,'vertices',Link1(:,1:3),'facec', [0.717,0.116,0.123]);
        set(L1, 'EdgeColor','none');
        set(L2,'vertices',Link2(:,1:3),'facec', [0.216,1,.583]);
        set(L2, 'EdgeColor','none');
        set(L3,'vertices',Link3(:,1:3),'facec', [0.306,0.733,1]);
        set(L3, 'EdgeColor','none');
        set(L4,'vertices',Link4(:,1:3),'facec', [1,0.542,0.493]);
        set(L4, 'EdgeColor','none');
        set(L5,'vertices',Link5(:,1:3),'facec', [0.216,1,.583]);
        set(L5, 'EdgeColor','none');
        set(L6,'vertices',Link6(:,1:3),'facec', [1,1,0.255]);
        set(L6, 'EdgeColor','none');
        set(L7,'vertices',Link7(:,1:3),'facec', [0.306,0.733,1]);
        set(L7, 'EdgeColor','none');
        % store trail in appdata 
        if trail == 'y'
            x_trail = getappdata(0,'xtrail');
            y_trail = getappdata(0,'ytrail');
            z_trail = getappdata(0,'ztrail');
            %
            xdata = [x_trail T_04(1,4)];
            ydata = [y_trail T_04(2,4)];
            zdata = [z_trail T_04(3,4)];
            %
            setappdata(0,'xtrail',xdata); % used for trail tracking.
            setappdata(0,'ytrail',ydata); % used for trail tracking.
            setappdata(0,'ztrail',zdata); % used for trail tracking.
            %
            set(Tr,'xdata',xdata,'ydata',ydata,'zdata',zdata);
        end
        drawnow
    end
    setappdata(0,'ThetaOld',[theta1,theta2,theta3,theta4,theta5,theta6]);
end
%%
% Inverse Kinematics
%
function [theta1,theta2,theta3,theta4,theta5,theta6] = Inverse_Kinematics(Px,Py,Pz)
    theta4 = 0;
    theta5 = 0;
    theta6 = 0;
    sign1 = 1;
    sign3 = 1;
    nogo = 0;
    noplot = 0;
    % Because the sqrt term in theta1 & theta3 can be + or - we run through
    % all possible combinations (i = 4) and take the first combination that
    % satisfies the joint angle constraints.
    while nogo == 0;
        for i = 1:1:4
            if i == 1
                sign1 = 1;
                sign3 = 1;
            elseif i == 2
                sign1 = 1;
                sign3 = -1;
            elseif i == 3
                sign1 = -1;
                sign3 = 1;
            else
                sign1 = -1;
                sign3 = -1;
            end
            a2 = 650;
            a3 = 0;
            d3 = 190;
            d4 = 600;
            rho = sqrt(Px^2+Py^2);
            phi = atan2(Py,Px);
            K = (Px^2+Py^2+Pz^2-a2^2-a3^2-d3^2-d4^2)/(2*a2);
            c4 = cos(theta4);
            s4 = sin(theta4);
            c5 = cos(theta5);
            s5 = sin(theta5);
            c6 = cos(theta6);
            s6 = sin(theta6);
            theta1 = (atan2(Py,Px)-atan2(d3,sign1*sqrt(Px^2+Py^2-d3^2)));

            c1 = cos(theta1);
            s1 = sin(theta1);
            theta3 = (atan2(a3,d4)-atan2(K,sign3*sqrt(a3^2+d4^2-K^2)));

            c3 = cos(theta3);
            s3 = sin(theta3);
            t23 = atan2((-a3-a2*c3)*Pz-(c1*Px+s1*Py)*(d4-a2*s3),(a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py));
            theta2 = (t23 - theta3);

            c2 = cos(theta2);
            s2 = sin(theta2);
            s23 = ((-a3-a2*c3)*Pz+(c1*Px+s1*Py)*(a2*s3-d4))/(Pz^2+(c1*Px+s1*Py)^2);
            c23 = ((a2*s3-d4)*Pz+(a3+a2*c3)*(c1*Px+s1*Py))/(Pz^2+(c1*Px+s1*Py)^2);
            r13 = -c1*(c23*c4*s5+s23*c5)-s1*s4*s5;
            r23 = -s1*(c23*c4*s5+s23*c5)+c1*s4*s5;
            r33 = s23*c4*s5 - c23*c5;
            theta4 = atan2(-r13*s1+r23*c1,-r13*c1*c23-r23*s1*c23+r33*s23);

            r11 = c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)+s1*(s4*c5*c6+c4*s6);
            r21 = s1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-c1*(s4*c5*c6+c4*s6);
            r31 = -s23*(c4*c5*c6-s4*s6)-c23*s5*c6;
            s5 = -(r13*(c1*c23*c4+s1*s4)+r23*(s1*c23*c4-c1*s4)-r33*(s23*c4));
            c5 = r13*(-c1*s23)+r23*(-s1*s23)+r33*(-c23);
            theta5 = atan2(s5,c5);

            s6 = -r11*(c1*c23*s4-s1*c4)-r21*(s1*c23*s4+c1*c4)+r31*(s23*s4);
            c6 = r11*((c1*c23*c4+s1*s4)*c5-c1*s23*s5)+r21*((s1*c23*c4-c1*s4)*c5-s1*s23*s5)-r31*(s23*c4*c5+c23*s5);
            theta6 = atan2(s6,c6);

            theta1 = theta1*180/pi;
            theta2 = theta2*180/pi;
            theta3 = theta3*180/pi;
            theta4 = theta4*180/pi;
            theta5 = theta5*180/pi;
            theta6 = theta6*180/pi;
            if theta2>=160 && theta2<=180
                theta2 = -theta2;
            end

            if theta1<=160 && theta1>=-160 && (theta2<=20 && theta2>=-200) && theta3<=45 && theta3>=-225 && theta4<=266 && theta4>=-266 && theta5<=100 && theta5>=-100 && theta6<=266 && theta6>=-266
                nogo = 1;
                theta3 = theta3+180;
                break
            end
            if i == 4 && nogo == 0
                h = errordlg('Point unreachable due to joint angle constraints.','JOINT ERROR');
                waitfor(h);
                nogo = 1;
                noplot = 1;
                break
            end
        end
    end
end
% Finally.
end