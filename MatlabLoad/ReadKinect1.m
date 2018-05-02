function [ DATACELL ] = ReadKinect1( address )
%% Public Constants and Variables
PacketLength = 322;
Joint_Packet_Length = 15;
Orientation_Packet_Length = 18;
Joints_No = 20;
Header_ID = 36; % $ sign
Footer_ID = 69; % E sign
JointPacket_ID = 37; % % sign
Joint_Types = { '1.Hip Center' 
    '2.Spine'
    '3.Shoulder Center'
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
    '20.Foot Right'};

%% Read File


fid = fopen(address,'r');
%data = fread(fid,inf);
%fclose(fid); 

%fid = fopen('Kinect Binary File2.bin','r');              % open file for reading
[data,~] = fread(fid,inf,'uint8=>uint8');
fclose(fid);                                % close file


TotalPackageNo = floor(length(data)/PacketLength);
n = 1;
for i=1:PacketLength:floor(length(data)/PacketLength)*PacketLength
    rows(n,:) = data(i:i+PacketLength-1);
   
    n = n + 1;
end


time = zeros(size(rows,1));
date = zeros(size(rows,1));
joint_pos = zeros(size(rows,1),Joints_No,4);

for i = 1:size(rows,1)
    
    % Check Header
    %header(i,:) = rows(i,1:3);
    for j= 1:2
        if rows(i,j) ~= Header_ID

            error('Header check failed');
        end
    end
    
    % Read Payload
    time(i,:)   = typecast(rows(i,3:6),'uint32');
    date(i,:) = typecast(rows(i,7:8),'uint16');
    Time_vec(i,:) = datevec(date(i) + 730486 + time(i) /  (60*1000*60*24));
    body_pos_x(i) = typecast(rows(i,9:12),'single');
    body_pos_y(i) = typecast(rows(i,13:16),'single');
    body_pos_z(i) = typecast(rows(i,17:20),'single');
    
    
    % Read Joints
    m = 1;
    for j=1:Joint_Packet_Length:(Joints_No)*Joint_Packet_Length
        if rows(i,20+j) ~= JointPacket_ID
            error('Joint check failed');
        end
        joint_stat(i,m) = rows(i,21+j);
        joint_pos_no(i,m) = rows(i,22+j);
        joint_pos_x(i,m) = typecast(rows(i,23+j:26+j),'single');
        joint_pos_y(i,m) = typecast(rows(i,27+j:30+j),'single');
        joint_pos_z(i,m) = typecast(rows(i,31+j:34+j),'single');
        
        m = m + 1;
    end
    
    
    
    
    for j= 321:322
        if rows(i,j) ~= Footer_ID
            error('Footer check failed');
        end
    end



end

fprintf('%s kinect binary file read.\n\n',address);

% Create Cell Data
    for i = 1:size(rows,1)
        
        
       DATACELL.BodyPosition.Raw(1,i) =  body_pos_x(i);
       DATACELL.BodyPosition.Raw(2,i) =  body_pos_y(i);
       DATACELL.BodyPosition.Raw(3,i) =  body_pos_z(i);
       
       DATACELL.Time.MATLAB(i)  = datetime(Time_vec(i,:));
       DATACELL.Time.RAW(i) = date(i) + time(i) /  (60*1000*60*24);
       DATACELL.Time.DATENUM(i) = datenum(Time_vec(i,:));
       
       
       for j=1:Joints_No
           DATACELL = GetJointPosition(DATACELL,i,joint_pos_no(i,j), ...
               joint_stat(i,j),joint_pos_x(i,j),joint_pos_y(i,j), ...
                joint_pos_z(i,j));
       end
       %disp('Kinect Joints Position generated.');
       
       
       
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
        data.HipCenter.Position.Raw(1,i) = posx;
        data.HipCenter.Position.Raw(2,i) = posy;
        data.HipCenter.Position.Raw(3,i) = posz;
        data.HipCenter.Stat{i} = GetJointStatus(joint_stat);
        
    case 2
        data.Spine.Position.Raw(1,i) = posx;
        data.Spine.Position.Raw(2,i) = posy;
        data.Spine.Position.Raw(3,i) = posz;
        data.Spine.Stat{i} = GetJointStatus(joint_stat);
        
    case 3
        data.ShoulderCenter.Position.Raw(1,i) = posx;
        data.ShoulderCenter.Position.Raw(2,i) = posy;
        data.ShoulderCenter.Position.Raw(3,i) = posz;
        data.ShoulderCenter.Stat{i} = GetJointStatus(joint_stat);
        
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
        
        
    otherwise
            erro('Joint Number invalide');
        
end
end

%% Returns Polar 
function data = GetJointPolar (data,i,joint_no , posx , ...
    posy , posz)
 

switch joint_no 
    case 1
        data.HipCenter.Polar.Raw(1,i) = posx;
        data.HipCenter.Polar.Raw(2,i) = posy;
        data.HipCenter.Polar.Raw(3,i) = posz;
        
    case 2
        data.Spine.Polar.Raw(1,i) = posx;
        data.Spine.Polar.Raw(2,i) = posy;
        data.Spine.Polar.Raw(3,i) = posz;
        
    case 3
        data.ShoulderCenter.Polar.Raw(1,i) = posx;
        data.ShoulderCenter.Polar.Raw(2,i) = posy;
        data.ShoulderCenter.Polar.Raw(3,i) = posz;
        
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
        
    otherwise
            error('Joint Number invalide > Polar Convertion');
        
end
end

%% Returns Polar 
function data = GetJointSpherical (data,i,joint_no , posx , ...
    posy , posz)
 

switch joint_no 
    case 1
        data.HipCenter.Spherical.Raw(1,i) = posx;
        data.HipCenter.Spherical.Raw(2,i) = posy;
        data.HipCenter.Spherical.Raw(3,i) = posz;
        
    case 2
        data.Spine.Spherical.Raw(1,i) = posx;
        data.Spine.Spherical.Raw(2,i) = posy;
        data.Spine.Spherical.Raw(3,i) = posz;
        
    case 3
        data.ShoulderCenter.Spherical.Raw(1,i) = posx;
        data.ShoulderCenter.Spherical.Raw(2,i) = posy;
        data.ShoulderCenter.Spherical.Raw(3,i) = posz;
        
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
       
    otherwise
            erro('Joint Number invalide > Spherical Convertion');
        
end
end



