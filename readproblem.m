function [N,C,R,T,M,NO,O,SO,NT] = readproblem(filename)
%READPROBLEM Read problem definition text file.
%   Params:
%         filename    - name of problem text file
%         N           - dimensions of map (1x2)
%         C           - collision threshold for map (1x1)
%         R           - robot start location (1x2)
%         T           - target trajectory (Mx2*NT)     EDITED
%         M           - map cell costs (PxQ where P = N(1), Q = N(2))

%  NO   - number of objects 
% O                -M x (NO) * 2 -- dynamic obstacle
%  SO  - size of objects [x y]
% NT - number of targets

FID = fopen(filename, 'r');

if(fgetl(FID) ~= 'N')
    fprintf('Error parsing problem file.')
    return;
end
N = fscanf(FID, '%d,%d')';

if(fgetl(FID) ~= 'C')
    fprintf('Error parsing problem file.')
    return;
end
C = fscanf(FID, '%d');

if(fgetl(FID) ~= 'R')
    fprintf('Error parsing problem file.')
    return;
end
R = fscanf(FID, '%f,%f')';

if(fgetl(FID) ~= 'G')
    fprintf('Error parsing problem file.NT')
    return;
end
NT = fscanf(FID, '%d');


if(fgetl(FID) ~= 'T')
    fprintf('Error parsing problem file.')
    return;
end
form = repmat('%f,%f,',1,NT);
T = textscan(FID, form, 'CollectOutput', true);
T = T{1};


if(fgetl(FID) ~= 'M')
    fprintf('Error parsing problem file.')
    return;
end
formatSpec = repmat(replace(cat(2,repmat('%f,', 1, N(2)), '\n'), ',\n', '\n'), 1, N(1));
M = reshape(fscanf(FID, formatSpec), N(2), N(1))';


if(fgetl(FID) ~= 'G')
    fprintf('Error parsing problem file. NO')
    return;
end
NO = fscanf(FID, '%d')';

if(fgetl(FID) ~= 'R')
    fprintf('Error parsing problem file. SO') %wtf this wont work reee
    return;
end
SO = fscanf(FID, '%f %f\n')';

if(fgetl(FID) ~= 'O')
    fprintf('Error parsing problem file. O')
    return;
end
steps = size(T,1);
formatSpec = '%f,';
sizeO = [NO*2 steps];
O = transpose(fscanf(FID,formatSpec,sizeO));

end