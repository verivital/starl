close all;
format longg;

load('run_number.mat')

% This is the main program for tracking. It has an option for using Kinect
% or Optitrack. Optitrack has not been tested. Assuming it does work with
% Optitrack, you should use this file instead of mainprog for either system

% Select tracking system
% For Kinect, you will need to specify number of robots below
OPTITRACK = 1;
KINECT = 2;
opt_system = KINECT;

% Matlab didn't like them being in the Kinect if statement below
% They aren't needed when using Optitrack except for the robot types
% MINIDRONE, CREATE2, ARDRONE, THREEDR, GHOST2, MAVICPRO, PHANTOM3,
% and PHANTOM4
global numCreates
global numDrones
global numARDrones
global num3DRDrones
global numGhostDrones
global numMavicDrones
global numPhant3Drones
global numPhant4Drones

global imgColorAll
global mm_per_pixel
global camDistToFloor
global botArray
global BBoxFactor
global hysteresis
global MINIDRONE
global CREATE2
global ARDRONE
global THREEDR
global GHOST2
global MAVICPRO
global PHANTOM3
global PHANTOM4
MINIDRONE = 100;
CREATE2 = 101;
ARDRONE = 102;
THREEDR = 103;
GHOST2 = 104;
MAVICPRO = 105;
PHANTOM3 = 106;
PHANTOM4 = 107;

ip_prefix = '10.255.24.';
ip_broadcast = [ip_prefix, '255'];

% If using Kinect, modify these as necessary
if opt_system == KINECT
    numCreates =0;
    numDrones = 0; %minidrones
    numARDrones = 0;
    num3DRDrones = 0;
    numGhostDrones = 1;
    numMavicDrones = 0;
    numPhant3Drones = 0;
    numPhant4Drones = 0;
    BBoxFactor = 2;
    hysteresis = 10; % number of consecutive frames bot must be lost for auto-shutdown
    
    % Other things needed for Kinect tracking
    warning('off','images:imfindcircles:warnForSmallRadius')
    num_frames = 10000; % number of frames kinect will capture
    imgColorAll = zeros(480,640,3,num_frames,'uint8'); % stores all captured imgs
    mm_per_pixel = 5.663295322; % mm in one pixel at ground level
    found = false;
    robot_count = numCreates + numDrones + numARDrones + num3DRDrones + numGhostDrones + numMavicDrones + numPhant3Drones + numPhant4Drones;
    camDistToFloor = 3058; % in mm, as measured with Kinect
    robot_names = cell(1,robot_count);
    botArray = Robot.empty(robot_count,0);
    for i = 1:robot_count
        botArray(i) = Robot;
    end
    
    times = [];
end

% Start the tracker interface if it isn't running
if opt_system == OPTITRACK
    if ~exist('cameraCount', 'var')
        cameraCount = track_init(TTP_FILENAME);
        clc;
    end
    
% Start or restart Kinect tracking  
elseif opt_system == KINECT
    if exist('vid', 'var') && exist('vid2', 'var')
        stop([vid vid2]);
        clear vid vid2;
    elseif exist('vid1', 'var') && exist('vid2', 'var')
        stop([vid1 vid2]);
        clear vid1 vid2;
    end
    
    vid = videoinput('kinect',1); %color 
    vid2 = videoinput('kinect',2); %depth

    vid.FramesPerTrigger = 1;
    vid2.FramesPerTrigger = 1;
    
    vid.TriggerRepeat = num_frames;
    vid2.TriggerRepeat = num_frames;
    
    triggerconfig([vid vid2],'manual');
    start([vid vid2]);
    % tigger a couple times in case first frames are bad
    trigger([vid vid2])
    trigger([vid vid2])      
end
    

% Display keyboard shortcuts
disp('L - Launch robots');
disp('A - Abort all');
disp('W - Toggle waypoint viewing');
disp('N - Toggle waypoint name viewing');
disp('S - Resend waypoints');
disp('C - Clear history');
disp('X - Exit');
disp('Q - Track shutdown');

% Load the walls and waypoints (if required)
[walls, waypoints] = load_wpt(WPT_FILENAME, USE_WPT);
global disp_waypoints;
disp_waypoints = 1;
global disp_waypoint_names;
disp_waypoint_names = 0;
global fig_closed;
fig_closed = 0;
global clear_history;
clear_history = 0;

% Only transmit the waypoints once
global waypoints_transmitted;
waypoints_transmitted = 0;

% Launch variable
global send_launch;
send_launch = 1;

% Get all trackable robots, set up a structure to hold them
if opt_system == OPTITRACK
    [robot_count, robot_names] = track_getTrackables();
end

bots = struct('X',{0},'Y',{0},'Z',{0},'yaw',{0},'pitch',{0},'roll',{0},'type',{0},'visible',{0},'name',robot_names,...
    'history',{ones(MOTION_HISTORY_SIZE,2)*-1},'histangle',{ones(MOTION_HISTORY_SIZE,1)*-1},...
    'hist_index',{1},'drawhistory',{ones(HISTORY_SIZE,2)*-1},'draw_hist_index',{1});

if opt_system == KINECT
    while ~found
        trigger([vid vid2])
        % Get the acquired frames and metadata.
        [imgColor, ts_color, metaData_Color] = getdata(vid);
        [imgDepth, ts_depth, metaData_Depth] = getdata(vid2);
        % make this function modify botArray, instead of return so many things
        found = findBotsNateTest(imgColor, imgDepth); %HEY CHANGE THIS!!!
        if found == true
            for j = 1:robot_count
                bots(j).type = botArray(j).type;
                % name the robots according to color
                if botArray(j).color == 'r'
                    bots(j).name = 'bot0';
                elseif botArray(j).color == 'g'
                    bots(j).name = 'bot1';
                elseif botArray(j).color == 'b'
                    bots(j).name = 'bot2';
                elseif botArray(j).color == 'w'
                    bots(j).name = 'bot3'; %quick fix since matlab wasnt picking up red drone as bot0
                end
            end
        end
    end
end
    

% Set up the plot
fig = figure('KeyPressFcn',@fig_key_handler);

% Set up the file handle
if(SAVE_TO_FILE)
   fileHandle = fopen(OUTPUT_FILENAME,'w+'); 
end

% Start the frame count for drawing and timer for transmitting
frameCount = 0;
tic;
while 1
    %tic;
    frameCount = frameCount + 1;
    
    % get a frame
    if opt_system == OPTITRACK
        track_updateFrame()
    
    elseif opt_system == KINECT
        % Trigger both objects.
        trigger([vid vid2])
        % Get the acquired frames and metadata.
        [imgColor, ts_color, metaData_Color] = getdata(vid);
        [imgDepth, ts_depth, metaData_Depth] = getdata(vid2);
        % Save the frame so it can turned into a video after running
        imgColorAll(:,:,:,frameCount) = imgColor;
    end
    
    % If the figure was closed, exit
    if fig_closed == 1
        break;
    end
    
    if clear_history == 1
        disp('Clearing history');
        for i = 1:robot_count
            bots(i).drawhistory = ones(HISTORY_SIZE,2)*-1;
        end
        clear_history = 0;
    end
    
    % Get each robots position (if visible)
    for i = 1:robot_count
        if opt_system == OPTITRACK
            if track_isVisible(i-1)
                [x y z roll pitch yaw] = track_getTrackablePosition(i-1);
                % Round and convert to millimeters
                bots(i).X = round(x*1000);
                bots(i).Y = round(z*1000);
                bots(i).yaw = round(yaw)+90;
                % these were added to be compatable with new StarL 3D
                % if using drones, you will need to modify this
                bots(i).Z = 0;
                bots(i).type = CREATE2;
                bots(i).visible = 1;
                
                % Append the new point to the history
                hist_index = mod(bots(i).hist_index,MOTION_HISTORY_SIZE)+1;
                bots(i).history(bots(i).hist_index,:) = [bots(i).X bots(i).Y];
                bots(i).histangle(bots(i).hist_index) = bots(i).yaw;
                bots(i).hist_index = hist_index;
                
                hist = bots(i).drawhistory;
                if ~ismember([bots(i).X bots(i).Y], hist, 'rows')
                    draw_hist_index = mod(bots(i).draw_hist_index,HISTORY_SIZE)+1;
                    bots(i).drawhistory(bots(i).draw_hist_index,:) = [bots(i).X bots(i).Y];
                    bots(i).draw_hist_index = draw_hist_index;
                end
                
            else
                bots(i).visible = 0;
            end
        elseif opt_system == KINECT
            trackBots(imgColor, imgDepth, i)
            centerMM = getMMCoord(botArray(i).center, botArray(i).radius, botArray(i).type);
            bots(i).X = centerMM(1,1);
            bots(i).Y = centerMM(1,2);
            bots(i).Z = botArray(i).depth - camDistToFloor;
            bots(i).yaw = botArray(i).yaw;
            % Pitch and roll aren't estimated with Kinect, so set to 0
            bots(i).roll = 0;
            bots(i).pitch = 0;
            bots(i).visible = 1;
            % if bot hasn't been found for hysteresis frames, shutdown StarL 
            if botArray(i).hyst >= hysteresis 
               disp('Exiting...'); 
               close all; 
               shutdown_track = 0; 
               judp('SEND',4000,ip_broadcast,int8('ABORT')); 
               % And show the last frame recorded to see exactly where the robot was lost 
               figure(); %figure(2); 
               frame = getPixelsInBB(imgColor, botArray(i).BBox); 
               image(frame) 
               figure(); %figure(3); 
               image(imgColor) 
               hold on 
               viscircles(botArray(i).center, botArray(i).radius); 
               break; 
            end
            
            %may want to add the history stuff from above here
        end
    end
    %times = [times; toc];
    % Update the plot on every 4th frame
    if rem(frameCount,4) == 0
        plot_bots(fig, LINE_LEN, X_MAX, Y_MAX, bots, waypoints, walls,...
            disp_waypoints, disp_waypoint_names)
    end
    
    % Interpret an exit key press
    if get(fig,'currentkey') == 'x'
        disp('Exiting...');
        close all;
        shutdown_track = 0;
        judp('SEND',4000,ip_broadcast,int8('ABORT'));
        break;
    end
    
    if get(fig,'currentkey') == 'q'
        disp('Exiting...');
        close all;
        shutdown_track = 1;
        judp('SEND',4000,ip_broadcast,int8('ABORT'));
        break;
    end

    if USE_SERVER == 1       
        % Send positions to all connected nodes
        if ((toc > TX_PERIOD && send_launch ~= 1))
            server_send_robots(bots);
            tic;
        end
        
        % Send waypoints and robot positions
         if(waypoints_transmitted == 0)
             waypoints_transmitted = 1;
             server_send_waypoints(waypoints);
             server_send_robots(bots);
             if(SAVE_TO_FILE)
                save_robot_data(bots, fileHandle);
             end
         end

        % If launching the robots
        if send_launch == 1
            server_send_waypoints(waypoints);
            server_send_robots(bots);
            judp('SEND',4000,ip_broadcast,int8(['GO ' int2str(size(waypoints,2)) ' ' int2str(run_number)]));
            run_number = run_number + 1;
            send_launch = 0;
        end
        
        % If aborting the robots
        if send_launch == -1
            judp('SEND',4000,ip_broadcast,int8('ABORT'));
            disp('Aborting...');
            send_launch = 0;
            waypoints_transmitted = 1;
        end
    end
    
end

if(SAVE_TO_FILE)
    fclose(fileHandle);
end

if shutdown_track == 1
    track_shutdown();
    save('run_number.mat', 'run_number');
    clear
end

close all