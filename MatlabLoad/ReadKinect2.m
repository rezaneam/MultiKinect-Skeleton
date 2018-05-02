function [ DATACELL ] = ReadKinect2( address )
%% Public Constants and Variables
PacketLength = 847;
Joint_Packet_Length = 15;
Orientation_Packet_Length = 18;
Joints_No = 25;
Joint_Types = { '1.Spine Base' 
    '2.Spin Mid'
    '3.Neck'
    '4.Head'
    '5.Shoulder Left'
    '6.Elbow Left'
    '7.Wrist Left'
    '8.Hand Left'
    '9.Shoulder Right'
    '10.Elbow Right'
    '11.Wrist Right'
    '12.Hand Right'
    '13.Hip Left'
    '14.Knee Left'
    '15.Ankel Left'
    '16.Foot Left'
    '17.Hip Right'
    '18.Knee Right'
    '19.Ankel Right'
    '20.Foot Right'
    '21.Spin Shoulder'
    '22.Hand Tip Left'
    '23.Thumb Left'
    '24.Hand Tip Right'
    '25.Thumb Right'};

%% Read File


fid = fopen(address,'r');
%data = fread(fid,inf);
%fclose(fid); 

%fid = fopen('Kinect Binary File2.bin','r');              % open file for reading
[data,~] = fread(fid,inf,'uint8=>uint8');
fclose(fid);                                % close file


TotalPackageNo = floor(length(data)/PacketLength);
n = 1;
for i=1:847:floor(length(data)/PacketLength)*PacketLength
    rows(n,:) = data(i:i+PacketLength-1);
   
    n = n + 1;
end


time = zeros(size(rows,1));
date = zeros(size(rows,1));
joint_pos = zeros(size(rows,1),Joints_No,4);

for i = 1:size(rows,1)
    
    % Check Header
    %header(i,:) = rows(i,1:3);
    for j= 1:3
        if rows(i,j) ~= 36;
            error('Header check failed');
        end
    end
    
    % Read Payload
    time(i,:)   = typecast(rows(i,4:7),'uint32');
    date(i,:) = typecast(rows(i,8:9),'uint16');
    Time_vec(i,:) = datevec(date(i) + 730486 + time(i) /  (60*1000*60*24));
    right_hand(i)  = typecast(rows(i,10),'uint8');
    left_hand(i) = typecast(rows(i,11),'uint8');
    lean (i) = typecast(rows(i,12),'uint8');
    LeanX(i,:)  = typecast(rows(i,13:16),'single');
    LeanY(i,:)  = typecast(rows(i,17:20),'single');
    
    
    % Read Joints
    m = 1;
    for j=1:Joint_Packet_Length:(Joints_No)*Joint_Packet_Length
        %joint_flag(i,ceil(j/Joint_Packet_Length)) = rows(i,20+j);
        if rows(i,20+j) ~= 37
            error('Joint check failed');
        end
        joint_stat(i,m) = rows(i,34+j);
        joint_pos_no(i,m) = rows(i,21+j);
        joint_pos_x(i,m) = typecast(rows(i,22+j:25+j),'single');
        joint_pos_y(i,m) = typecast(rows(i,26+j:29+j),'single');
        joint_pos_z(i,m) = typecast(rows(i,30+j:33+j),'single');
        
        m = m + 1;
    end
    
    % Read Orientation
    m = 1;
    for j=1:Orientation_Packet_Length:(Joints_No)*Orientation_Packet_Length
        %joint_flag(i,ceil(j/Joint_Packet_Length)) = rows(i,20+j);
        if rows(i,395+j) ~= 165
            error('Orientation check failed');
        end

        joint_orient_no(i,m) = rows(i,396+j);
        joint_orient_w(i,m) = typecast(rows(i,397+j:400+j),'single');
        joint_orient_x(i,m) = typecast(rows(i,401+j:404+j),'single');
        joint_orient_y(i,m) = typecast(rows(i,405+j:408+j),'single');
        joint_orient_z(i,m) = typecast(rows(i,409+j:412+j),'single');
        
        m = m + 1;
    end
    
    for j= 846:847
        if rows(i,j) ~= 69;
            error('Footer check failed');
        end
    end



end

fprintf('%s kinect binary file read.\n\n',address);

% Create Cell Data
    for i = 1:size(rows,1)
        
        
           
       DATACELL.Time.MATLAB(i)  = datetime(Time_vec(i,:));
       DATACELL.Time.RAW(i) = date(i) + time(i) /  (60*1000*60*24);
       DATACELL.Time.DATENUM(i) = datenum(Time_vec(i,:));
       DATACELL.RightHand{i} = GetHandStatus(right_hand(i));
       DATACELL.LeftHand{i} = GetHandStatus(left_hand(i));
       DATACELL.LeanStat{i} = GetLeanStatus(lean(i));
       DATACELL.Lean(1,i) = LeanX(i);
       DATACELL.Lean(2,i) = LeanY(i);
       
       for j=1:Joints_No
           DATACELL = GetJointPosition(DATACELL,i,joint_pos_no(i,j), ...
               joint_stat(i,j),joint_pos_x(i,j),joint_pos_y(i,j), ...
                joint_pos_z(i,j));
       end
       %disp('Kinect Joints Position generated.');
       
       for j=1:Joints_No
           DATACELL = GetJointOrientation(DATACELL,i,joint_orient_no(i,j), ...
               joint_orient_w(i,j),joint_orient_x(i,j),joint_orient_y(i,j), ...
               joint_orient_z(i,j));
       end
       %disp('Kinect Joints Orientation generated.');
       
       for  j=1:Joints_No
           [joint_polar_theta(:,j),joint_polar_rho(:,j),joint_polar_z(:,j)] = ...
               cart2pol(joint_pos_x(:,j) ,joint_pos_y(:,j) ,joint_pos_z(:,j)  );
           DATACELL = GetJointPolar(DATACELL,i,joint_pos_no(i,j), ...
               joint_polar_theta(i,j),joint_polar_rho(i,j),joint_polar_z(i,j));
       end
       %disp('Kinect Joints Polar Coordinates generated.');
       
       for  j=1:Joints_No
           [joint_spherical_azimuth(:,j),joint_spherical_elevation(:,j),...
               joint_spherical_r(:,j)] = cart2sph(joint_pos_x(:,j)...
                ,joint_pos_y(:,j) ,joint_pos_z(:,j));
           DATACELL = GetJointSpherical(DATACELL,i,joint_pos_no(i,j), ...
               joint_spherical_azimuth(i,j),...
               joint_spherical_elevation(i,j),joint_spherical_r(i,j));
       end
       %disp('Kinect Joints Spherical Coordinates generated.');
       
    end
    
    %disp('Kinect cell generated.');
end

%% Returns Hand Status 
function HandStat = GetHandStatus(stat)
    switch bitand(7,stat)
        case 0
            HandStat = 'Unknown';
        case 1
            HandStat = 'Not Tracked';
        case 2
            HandStat = 'Open';
        case 3
            HandStat = 'Close';
        case 4
            HandStat = 'Lasso';
        otherwise
            
            error ('Hand Status verification failed');
    end
    
    if stat > 7
        HandStat =  sprintf('%s .Hi Confidence', HandStat);
    else
        HandStat =  sprintf('%s .Low Confidence', HandStat);
        
    end
end

%% Returns Lean Tracking Status
function LeanStat = GetLeanStatus (stat)
    switch stat
        case 0
            LeanStat = 'Not Tracked';
        case 1
            LeanStat = 'Inferred';
        case 2
            LeanStat = 'Tracked';
        otherwise
            error ('Lean Status verification failed');
    end
end

%% Returns Joiny Tracking Status
function LeanStat = GetJointStatus (stat)
    switch stat
        case 0
            LeanStat = 'Not Tracked';
        case 1
            LeanStat = 'Inferred';
        case 2
            LeanStat = 'Tracked';
        otherwise
            error ('Joiny Status verification failed');
    end
end
%% Returns Position and Status of each Joint
function data = GetJointPosition (data,i,joint_no, joint_stat , posx , ...
    posy , posz)
 
switch joint_no 
    case 1
        data.SpineBase.Position.Raw(1,i) = posx;
        data.SpineBase.Position.Raw(2,i) = posy;
        data.SpineBase.Position.Raw(3,i) = posz;
        data.SpineBase.Stat{i} = GetJointStatus(joint_stat);
        
    case 2
        data.SpinMid.Position.Raw(1,i) = posx;
        data.SpinMid.Position.Raw(2,i) = posy;
        data.SpinMid.Position.Raw(3,i) = posz;
        data.SpinMid.Stat{i} = GetJointStatus(joint_stat);
        
    case 3
        data.Neck.Position.Raw(1,i) = posx;
        data.Neck.Position.Raw(2,i) = posy;
        data.Neck.Position.Raw(3,i) = posz;
        data.Neck.Stat{i} = GetJointStatus(joint_stat);
        
        case 4
        data.Head.Position.Raw(1,i) = posx;
        data.Head.Position.Raw(2,i) = posy;
        data.Head.Position.Raw(3,i) = posz;
        data.Head.Stat{i} = GetJointStatus(joint_stat);
        
        case 5
        data.ShoulderLeft.Position.Raw(1,i) = posx;
        data.ShoulderLeft.Position.Raw(2,i) = posy;
        data.ShoulderLeft.Position.Raw(3,i) = posz;
        data.ShoulderLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 6
        data.ElbowLeft.Position.Raw(1,i) = posx;
        data.ElbowLeft.Position.Raw(2,i) = posy;
        data.ElbowLeft.Position.Raw(3,i) = posz;
        data.ElbowLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 7
        data.WristLeft.Position.Raw(1,i) = posx;
        data.WristLeft.Position.Raw(2,i) = posy;
        data.WristLeft.Position.Raw(3,i) = posz;
        data.WristLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 8
        data.HandLeft.Position.Raw(1,i) = posx;
        data.HandLeft.Position.Raw(2,i) = posy;
        data.HandLeft.Position.Raw(3,i) = posz;
        data.HandLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 9
        data.ShoulderRight.Position.Raw(1,i) = posx;
        data.ShoulderRight.Position.Raw(2,i) = posy;
        data.ShoulderRight.Position.Raw(3,i) = posz;
        data.ShoulderRight.Stat{i} = GetJointStatus(joint_stat);
        
        case 10
        data.ElbowRight.Position.Raw(1,i) = posx;
        data.ElbowRight.Position.Raw(2,i) = posy;
        data.ElbowRight.Position.Raw(3,i) = posz;
        data.ElbowRight.Stat{i} = GetJointStatus(joint_stat);
        
        
        case 11
        data.WristRight.Position.Raw(1,i) = posx;
        data.WristRight.Position.Raw(2,i) = posy;
        data.WristRight.Position.Raw(3,i) = posz;
        data.WristRight.Stat{i} = GetJointStatus(joint_stat);
        
        case 12
        data.HandRight.Position.Raw(1,i) = posx;
        data.HandRight.Position.Raw(2,i) = posy;
        data.HandRight.Position.Raw(3,i) = posz;
        data.HandRight.Stat{i} = GetJointStatus(joint_stat);
        
        case 13
        data.HipLeft.Position.Raw(1,i) = posx;
        data.HipLeft.Position.Raw(2,i) = posy;
        data.HipLeft.Position.Raw(3,i) = posz;
        data.HipLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 14
        data.KneeLeft.Position.Raw(1,i) = posx;
        data.KneeLeft.Position.Raw(2,i) = posy;
        data.KneeLeft.Position.Raw(3,i) = posz;
        data.KneeLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 15
        data.AnkelLeft.Position.Raw(1,i) = posx;
        data.AnkelLeft.Position.Raw(2,i) = posy;
        data.AnkelLeft.Position.Raw(3,i) = posz;
        data.AnkelLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 16
        data.FootLeft.Position.Raw(1,i) = posx;
        data.FootLeft.Position.Raw(2,i) = posy;
        data.FootLeft.Position.Raw(3,i) = posz;
        data.FootLeft.Stat{i} = GetJointStatus(joint_stat);
        
        
        case 17
        data.HipRight.Position.Raw(1,i) = posx;
        data.HipRight.Position.Raw(2,i) = posy;
        data.HipRight.Position.Raw(3,i) = posz;
        data.HipRight.Stat{i} = GetJointStatus(joint_stat);
        
        case 18
        data.KneeRight.Position.Raw(1,i) = posx;
        data.KneeRight.Position.Raw(2,i) = posy;
        data.KneeRight.Position.Raw(3,i) = posz;
        data.KneeRight.Stat{i} = GetJointStatus(joint_stat);
        
        
        case 19
        data.AnkelRight.Position.Raw(1,i) = posx;
        data.AnkelRight.Position.Raw(2,i) = posy;
        data.AnkelRight.Position.Raw(3,i) = posz;
        data.AnkelRight.Stat{i} = GetJointStatus(joint_stat);
        
        case 20
        data.FootRight.Position.Raw(1,i) = posx;
        data.FootRight.Position.Raw(2,i) = posy;
        data.FootRight.Position.Raw(3,i) = posz;
        data.FootRight.Stat{i} = GetJointStatus(joint_stat);
        
        case 21
        data.SpinShoulder.Position.Raw(1,i) = posx;
        data.SpinShoulder.Position.Raw(2,i) = posy;
        data.SpinShoulder.Position.Raw(3,i) = posz;
        data.SpinShoulder.Stat{i} = GetJointStatus(joint_stat);
        
        case 22
        data.HandTipLeft.Position.Raw(1,i) = posx;
        data.HandTipLeft.Position.Raw(2,i) = posy;
        data.HandTipLeft.Position.Raw(3,i) = posz;
        data.HandTipLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 23
        data.ThumbLeft.Position.Raw(1,i) = posx;
        data.ThumbLeft.Position.Raw(2,i) = posy;
        data.ThumbLeft.Position.Raw(3,i) = posz;
        data.ThumbLeft.Stat{i} = GetJointStatus(joint_stat);
        
        case 24
        data.HandTipRight.Position.Raw(1,i) = posx;
        data.HandTipRight.Position.Raw(2,i) = posy;
        data.HandTipRight.Position.Raw(3,i) = posz;
        data.HandTipRight.Stat{i} = GetJointStatus(joint_stat);
        
        case 25
        data.ThumbRight.Position.Raw(1,i) = posx;
        data.ThumbRight.Position.Raw(2,i) = posy;
        data.ThumbRight.Position.Raw(3,i) = posz;
        data.ThumbRight.Stat{i} = GetJointStatus(joint_stat);
        
    otherwise
            erro('Joint Number invalide');
        
end
end
%% Returns Position and Status of each Joint
function data = GetJointOrientation (data,i,joint_no , orientw , ...
    orientx , orienty , orientz)
 
switch joint_no 
    case 1
        data.SpineBase.Orientation.Raw(1,i) = orientw;
        data.SpineBase.Orientation.Raw(2,i) = orientx;
        data.SpineBase.Orientation.Raw(3,i) = orienty;
        data.SpineBase.Orientation.Raw(4,i) = orientz;
        
    case 2
        data.SpinMid.Orientation.Raw(1,i) = orientw;
        data.SpinMid.Orientation.Raw(2,i) = orientx;
        data.SpinMid.Orientation.Raw(3,i) = orienty;
        data.SpinMid.Orientation.Raw(4,i) = orientz;
    
    case 3
        data.Neck.Orientation.Raw(1,i) = orientw;
        data.Neck.Orientation.Raw(2,i) = orientx;
        data.Neck.Orientation.Raw(3,i) = orienty;
        data.Neck.Orientation.Raw(4,i) = orientz;
        
        case 4
        data.Head.Orientation.Raw(1,i) = orientw;
        data.Head.Orientation.Raw(2,i) = orientx;
        data.Head.Orientation.Raw(3,i) = orienty;
        data.Head.Orientation.Raw(4,i) = orientz;
        
        case 5
        data.ShoulderLeft.Orientation.Raw(1,i) = orientw;
        data.ShoulderLeft.Orientation.Raw(2,i) = orientx;
        data.ShoulderLeft.Orientation.Raw(3,i) = orienty;
        data.ShoulderLeft.Orientation.Raw(4,i) = orientz;
        
        case 6
        data.ElbowLeft.Orientation.Raw(1,i) = orientw;
        data.ElbowLeft.Orientation.Raw(2,i) = orientx;
        data.ElbowLeft.Orientation.Raw(3,i) = orienty;
        data.ElbowLeft.Orientation.Raw(4,i) = orientz;
        
        
        case 7
        data.WristLeft.Orientation.Raw(1,i) = orientw;
        data.WristLeft.Orientation.Raw(2,i) = orientx;
        data.WristLeft.Orientation.Raw(3,i) = orienty;
        data.WristLeft.Orientation.Raw(4,i) = orientz;
        
        case 8
        data.HandLeft.Orientation.Raw(1,i) = orientw;
        data.HandLeft.Orientation.Raw(2,i) = orientx;
        data.HandLeft.Orientation.Raw(3,i) = orienty;
        data.HandLeft.Orientation.Raw(4,i) = orientz;
        
        case 9
        data.ShoulderRight.Orientation.Raw(1,i) = orientw;
        data.ShoulderRight.Orientation.Raw(2,i) = orientx;
        data.ShoulderRight.Orientation.Raw(3,i) = orienty;
        data.ShoulderRight.Orientation.Raw(4,i) = orientz;
        
        case 10
        data.ElbowRight.Orientation.Raw(1,i) = orientw;
        data.ElbowRight.Orientation.Raw(2,i) = orientx;
        data.ElbowRight.Orientation.Raw(3,i) = orienty;
        data.ElbowRight.Orientation.Raw(4,i) = orientz;
        
        case 11
        data.WristRight.Orientation.Raw(1,i) = orientw;
        data.WristRight.Orientation.Raw(2,i) = orientx;
        data.WristRight.Orientation.Raw(3,i) = orienty;
        data.WristRight.Orientation.Raw(4,i) = orientz;
        
        case 12
        data.HandRight.Orientation.Raw(1,i) = orientw;
        data.HandRight.Orientation.Raw(2,i) = orientx;
        data.HandRight.Orientation.Raw(3,i) = orienty;
        data.HandRight.Orientation.Raw(4,i) = orientz;
        
        case 13
        data.HipLeft.Orientation.Raw(1,i) = orientw;
        data.HipLeft.Orientation.Raw(2,i) = orientx;
        data.HipLeft.Orientation.Raw(3,i) = orienty;
        data.HipLeft.Orientation.Raw(4,i) = orientz;
        
        
        case 14
        data.KneeLeft.Orientation.Raw(1,i) = orientw;
        data.KneeLeft.Orientation.Raw(2,i) = orientx;
        data.KneeLeft.Orientation.Raw(3,i) = orienty;
        data.KneeLeft.Orientation.Raw(4,i) = orientz;
        
        case 15
        data.AnkelLeft.Orientation.Raw(1,i) = orientw;
        data.AnkelLeft.Orientation.Raw(2,i) = orientx;
        data.AnkelLeft.Orientation.Raw(3,i) = orienty;
        data.AnkelLeft.Orientation.Raw(4,i) = orientz;
        
        case 16
        data.FootLeft.Orientation.Raw(1,i) = orientw;
        data.FootLeft.Orientation.Raw(2,i) = orientx;
        data.FootLeft.Orientation.Raw(3,i) = orienty;
        data.FootLeft.Orientation.Raw(4,i) = orientz;
        
        case 17
        data.HipRight.Orientation.Raw(1,i) = orientw;
        data.HipRight.Orientation.Raw(2,i) = orientx;
        data.HipRight.Orientation.Raw(3,i) = orienty;
        data.HipRight.Orientation.Raw(4,i) = orientz;
        
        case 18
        data.KneeRight.Orientation.Raw(1,i) = orientw;
        data.KneeRight.Orientation.Raw(2,i) = orientx;
        data.KneeRight.Orientation.Raw(3,i) = orienty;
        data.KneeRight.Orientation.Raw(4,i) = orientz;
         
        case 19
        data.AnkelRight.Orientation.Raw(1,i) = orientw;
        data.AnkelRight.Orientation.Raw(2,i) = orientx;
        data.AnkelRight.Orientation.Raw(3,i) = orienty;
        data.AnkelRight.Orientation.Raw(4,i) = orientz;
        
        case 20
        data.FootRight.Orientation.Raw(1,i) = orientw;
        data.FootRight.Orientation.Raw(2,i) = orientx;
        data.FootRight.Orientation.Raw(3,i) = orienty;
        data.FootRight.Orientation.Raw(4,i) = orientz;
        
        case 21
        data.SpinShoulder.Orientation.Raw(1,i) = orientw;
        data.SpinShoulder.Orientation.Raw(2,i) = orientx;
        data.SpinShoulder.Orientation.Raw(3,i) = orienty;
        data.SpinShoulder.Orientation.Raw(4,i) = orientz;
        
        case 22
        data.HandTipLeft.Orientation.Raw(1,i) = orientw;
        data.HandTipLeft.Orientation.Raw(2,i) = orientx;
        data.HandTipLeft.Orientation.Raw(3,i) = orienty;
        data.HandTipLeft.Orientation.Raw(4,i) = orientz;
        
        case 23
        data.ThumbLeft.Orientation.Raw(1,i) = orientw;
        data.ThumbLeft.Orientation.Raw(2,i) = orientx;
        data.ThumbLeft.Orientation.Raw(3,i) = orienty;
        data.ThumbLeft.Orientation.Raw(4,i) = orientz;
        
        case 24
        data.HandTipRight.Orientation.Raw(1,i) = orientw;
        data.HandTipRight.Orientation.Raw(2,i) = orientx;
        data.HandTipRight.Orientation.Raw(3,i) = orienty;
        data.HandTipRight.Orientation.Raw(4,i) = orientz;
        
        case 25
        data.ThumbRight.Orientation.Raw(1,i) = orientw;
        data.ThumbRight.Orientation.Raw(2,i) = orientx;
        data.ThumbRight.Orientation.Raw(3,i) = orienty;
        data.ThumbRight.Orientation.Raw(4,i) = orientz;
        
        
    otherwise
            erro('Joint Number invalide');
        
end
end
%% Returns Polar 
function data = GetJointPolar (data,i,joint_no , posx , ...
    posy , posz)
 

switch joint_no 
    case 1
        data.SpineBase.Polar.Raw(1,i) = posx;
        data.SpineBase.Polar.Raw(2,i) = posy;
        data.SpineBase.Polar.Raw(3,i) = posz;
        
    case 2
        data.SpinMid.Polar.Raw(1,i) = posx;
        data.SpinMid.Polar.Raw(2,i) = posy;
        data.SpinMid.Polar.Raw(3,i) = posz;
        
    case 3
        data.Neck.Polar.Raw(1,i) = posx;
        data.Neck.Polar.Raw(2,i) = posy;
        data.Neck.Polar.Raw(3,i) = posz;
        
        case 4
        data.Head.Polar.Raw(1,i) = posx;
        data.Head.Polar.Raw(2,i) = posy;
        data.Head.Polar.Raw(3,i) = posz;
        
        case 5
        data.ShoulderLeft.Polar.Raw(1,i) = posx;
        data.ShoulderLeft.Polar.Raw(2,i) = posy;
        data.ShoulderLeft.Polar.Raw(3,i) = posz;
        
        case 6
        data.ElbowLeft.Polar.Raw(1,i) = posx;
        data.ElbowLeft.Polar.Raw(2,i) = posy;
        data.ElbowLeft.Polar.Raw(3,i) = posz;
        
        case 7
        data.WristLeft.Polar.Raw(1,i) = posx;
        data.WristLeft.Polar.Raw(2,i) = posy;
        data.WristLeft.Polar.Raw(3,i) = posz;
        
        case 8
        data.HandLeft.Polar.Raw(1,i) = posx;
        data.HandLeft.Polar.Raw(2,i) = posy;
        data.HandLeft.Polar.Raw(3,i) = posz;
        
        case 9
        data.ShoulderRight.Polar.Raw(1,i) = posx;
        data.ShoulderRight.Polar.Raw(2,i) = posy;
        data.ShoulderRight.Polar.Raw(3,i) = posz;
        
        case 10
        data.ElbowRight.Polar.Raw(1,i) = posx;
        data.ElbowRight.Polar.Raw(2,i) = posy;
        data.ElbowRight.Polar.Raw(3,i) = posz;
        
        
        case 11
        data.WristRight.Polar.Raw(1,i) = posx;
        data.WristRight.Polar.Raw(2,i) = posy;
        data.WristRight.Polar.Raw(3,i) = posz;
        
        case 12
        data.HandRight.Polar.Raw(1,i) = posx;
        data.HandRight.Polar.Raw(2,i) = posy;
        data.HandRight.Polar.Raw(3,i) = posz;
        
        case 13
        data.HipLeft.Polar.Raw(1,i) = posx;
        data.HipLeft.Polar.Raw(2,i) = posy;
        data.HipLeft.Polar.Raw(3,i) = posz;
        
        case 14
        data.KneeLeft.Polar.Raw(1,i) = posx;
        data.KneeLeft.Polar.Raw(2,i) = posy;
        data.KneeLeft.Polar.Raw(3,i) = posz;
        
        case 15
        data.AnkelLeft.Polar.Raw(1,i) = posx;
        data.AnkelLeft.Polar.Raw(2,i) = posy;
        data.AnkelLeft.Polar.Raw(3,i) = posz;
        
        case 16
        data.FootLeft.Polar.Raw(1,i) = posx;
        data.FootLeft.Polar.Raw(2,i) = posy;
        data.FootLeft.Polar.Raw(3,i) = posz;
        
        case 17
        data.HipRight.Polar.Raw(1,i) = posx;
        data.HipRight.Polar.Raw(2,i) = posy;
        data.HipRight.Polar.Raw(3,i) = posz;
        
        case 18
        data.KneeRight.Polar.Raw(1,i) = posx;
        data.KneeRight.Polar.Raw(2,i) = posy;
        data.KneeRight.Polar.Raw(3,i) = posz;       
        
        case 19
        data.AnkelRight.Polar.Raw(1,i) = posx;
        data.AnkelRight.Polar.Raw(2,i) = posy;
        data.AnkelRight.Polar.Raw(3,i) = posz;
        
        case 20
        data.FootRight.Polar.Raw(1,i) = posx;
        data.FootRight.Polar.Raw(2,i) = posy;
        data.FootRight.Polar.Raw(3,i) = posz;
        
        case 21
        data.SpinShoulder.Polar.Raw(1,i) = posx;
        data.SpinShoulder.Polar.Raw(2,i) = posy;
        data.SpinShoulder.Polar.Raw(3,i) = posz;
        
        case 22
        data.HandTipLeft.Polar.Raw(1,i) = posx;
        data.HandTipLeft.Polar.Raw(2,i) = posy;
        data.HandTipLeft.Polar.Raw(3,i) = posz;
        
        case 23
        data.ThumbLeft.Polar.Raw(1,i) = posx;
        data.ThumbLeft.Polar.Raw(2,i) = posy;
        data.ThumbLeft.Polar.Raw(3,i) = posz;
        
        case 24
        data.HandTipRight.Polar.Raw(1,i) = posx;
        data.HandTipRight.Polar.Raw(2,i) = posy;
        data.HandTipRight.Polar.Raw(3,i) = posz;
        
        case 25
        data.ThumbRight.Polar.Raw(1,i) = posx;
        data.ThumbRight.Polar.Raw(2,i) = posy;
        data.ThumbRight.Polar.Raw(3,i) = posz;
        
    otherwise
            erro('Joint Number invalide > Polar Convertion');
        
end
end

%% Returns Polar 
function data = GetJointSpherical (data,i,joint_no , posx , ...
    posy , posz)
 

switch joint_no 
    case 1
        data.SpineBase.Spherical.Raw(1,i) = posx;
        data.SpineBase.Spherical.Raw(2,i) = posy;
        data.SpineBase.Spherical.Raw(3,i) = posz;
        
    case 2
        data.SpinMid.Spherical.Raw(1,i) = posx;
        data.SpinMid.Spherical.Raw(2,i) = posy;
        data.SpinMid.Spherical.Raw(3,i) = posz;
        
    case 3
        data.Neck.Spherical.Raw(1,i) = posx;
        data.Neck.Spherical.Raw(2,i) = posy;
        data.Neck.Spherical.Raw(3,i) = posz;
        
        case 4
        data.Head.Spherical.Raw(1,i) = posx;
        data.Head.Spherical.Raw(2,i) = posy;
        data.Head.Spherical.Raw(3,i) = posz;
        
        case 5
        data.ShoulderLeft.Spherical.Raw(1,i) = posx;
        data.ShoulderLeft.Spherical.Raw(2,i) = posy;
        data.ShoulderLeft.Spherical.Raw(3,i) = posz;
        
        case 6
        data.ElbowLeft.Spherical.Raw(1,i) = posx;
        data.ElbowLeft.Spherical.Raw(2,i) = posy;
        data.ElbowLeft.Spherical.Raw(3,i) = posz;
        
        case 7
        data.WristLeft.Spherical.Raw(1,i) = posx;
        data.WristLeft.Spherical.Raw(2,i) = posy;
        data.WristLeft.Spherical.Raw(3,i) = posz;
        
        case 8
        data.HandLeft.Spherical.Raw(1,i) = posx;
        data.HandLeft.Spherical.Raw(2,i) = posy;
        data.HandLeft.Spherical.Raw(3,i) = posz;
        
        case 9
        data.ShoulderRight.Spherical.Raw(1,i) = posx;
        data.ShoulderRight.Spherical.Raw(2,i) = posy;
        data.ShoulderRight.Spherical.Raw(3,i) = posz;
        
        case 10
        data.ElbowRight.Spherical.Raw(1,i) = posx;
        data.ElbowRight.Spherical.Raw(2,i) = posy;
        data.ElbowRight.Spherical.Raw(3,i) = posz;
        
        
        case 11
        data.WristRight.Spherical.Raw(1,i) = posx;
        data.WristRight.Spherical.Raw(2,i) = posy;
        data.WristRight.Spherical.Raw(3,i) = posz;
        
        case 12
        data.HandRight.Spherical.Raw(1,i) = posx;
        data.HandRight.Spherical.Raw(2,i) = posy;
        data.HandRight.Spherical.Raw(3,i) = posz;
        
        case 13
        data.HipLeft.Spherical.Raw(1,i) = posx;
        data.HipLeft.Spherical.Raw(2,i) = posy;
        data.HipLeft.Spherical.Raw(3,i) = posz;
        
        case 14
        data.KneeLeft.Spherical.Raw(1,i) = posx;
        data.KneeLeft.Spherical.Raw(2,i) = posy;
        data.KneeLeft.Spherical.Raw(3,i) = posz;
        
        case 15
        data.AnkelLeft.Spherical.Raw(1,i) = posx;
        data.AnkelLeft.Spherical.Raw(2,i) = posy;
        data.AnkelLeft.Spherical.Raw(3,i) = posz;
        
        case 16
        data.FootLeft.Spherical.Raw(1,i) = posx;
        data.FootLeft.Spherical.Raw(2,i) = posy;
        data.FootLeft.Spherical.Raw(3,i) = posz;
        
        case 17
        data.HipRight.Spherical.Raw(1,i) = posx;
        data.HipRight.Spherical.Raw(2,i) = posy;
        data.HipRight.Spherical.Raw(3,i) = posz;
        
        case 18
        data.KneeRight.Spherical.Raw(1,i) = posx;
        data.KneeRight.Spherical.Raw(2,i) = posy;
        data.KneeRight.Spherical.Raw(3,i) = posz;       
        
        case 19
        data.AnkelRight.Spherical.Raw(1,i) = posx;
        data.AnkelRight.Spherical.Raw(2,i) = posy;
        data.AnkelRight.Spherical.Raw(3,i) = posz;
        
        case 20
        data.FootRight.Spherical.Raw(1,i) = posx;
        data.FootRight.Spherical.Raw(2,i) = posy;
        data.FootRight.Spherical.Raw(3,i) = posz;
        
        case 21
        data.SpinShoulder.Spherical.Raw(1,i) = posx;
        data.SpinShoulder.Spherical.Raw(2,i) = posy;
        data.SpinShoulder.Spherical.Raw(3,i) = posz;
        
        case 22
        data.HandTipLeft.Spherical.Raw(1,i) = posx;
        data.HandTipLeft.Spherical.Raw(2,i) = posy;
        data.HandTipLeft.Spherical.Raw(3,i) = posz;
        
        case 23
        data.ThumbLeft.Spherical.Raw(1,i) = posx;
        data.ThumbLeft.Spherical.Raw(2,i) = posy;
        data.ThumbLeft.Spherical.Raw(3,i) = posz;
        
        case 24
        data.HandTipRight.Spherical.Raw(1,i) = posx;
        data.HandTipRight.Spherical.Raw(2,i) = posy;
        data.HandTipRight.Spherical.Raw(3,i) = posz;
        
        case 25
        data.ThumbRight.Spherical.Raw(1,i) = posx;
        data.ThumbRight.Spherical.Raw(2,i) = posy;
        data.ThumbRight.Spherical.Raw(3,i) = posz;
        
    otherwise
            error('Joint Number invalide > Spherical Convertion');
        
end
end