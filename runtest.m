function[caught, time, numofmoves, pathcost] = runtest(problemfile)

[mapdims, C, robotstart, targettraj, envmap, numObs, Obs, sizeObs,numTar] = readproblem(problemfile);

close all;

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(envmap'); axis square; colorbar; colormap jet; hold on;

%current positions of the target and robot
time = 0;
robotpos = robotstart;
targetpos = targettraj(1,:);

%now comes the main loop
hr = -1;
ht = -1;
numofmoves = 0;
caught = false;
pathcost = 0;

%draw the positions
if (hr ~= -1)
    delete(hr);
    delete(ht);
end
hr = text(robotpos(1), robotpos(2), 'R', 'Color', 'g', 'FontWeight', 'bold');
hr = scatter(robotpos(1), robotpos(2), 10, 'g', 'filled');
for i = 1:numTar
    curX = targetpos(1+2*(i-1));
    curY = targetpos(2+2*(i-1));
    ht = text(curX, curY, 'T', 'Color', 'm', 'FontWeight', 'bold');
    ht = scatter(curX, curY, 10, 'm', 'filled');
end

%gotta detect caught states
caught = zeros(1,numTar);

pause(1.0);

% robot can take at most as many steps as target takes
while (true)
    
    %imagesc(envmap'); axis square; colorbar; colormap jet;
    
    % call robot planner to find what they want to do
    t0 = clock;
    newrobotpos = robotplanner(envmap, robotpos, targettraj, targetpos, time, C, Obs, sizeObs,numTar, caught);
    if (size(newrobotpos, 1) > size(targettraj, 1) || size(newrobotpos, 2) ~= 2)
        fprintf(1, 'ERROR: invalid action\n');
        fprintf(1, '\t newrobotpos must be M x %d with M <= %d.\n', 2, size(targettraj, 1));
        return;
    end

    newrobotpos = cast(newrobotpos, 'like', robotpos);
    if (newrobotpos(1) < 1 || newrobotpos(1) > size(envmap, 1) || ...
            newrobotpos(2) < 1 || newrobotpos(2) > size(envmap, 2))
        fprintf(1, 'ERROR: out-of-map robot position commanded\n');
        return;
    elseif (envmap(newrobotpos(1), newrobotpos(2)) >= C)
        fprintf(1, 'ERROR: planned action leads to collision\n');
        return;
    elseif (abs(newrobotpos(1)-robotpos(1)) > 1 || abs(newrobotpos(2)-robotpos(2)) > 1)
        fprintf(1, 'ERROR: invalid action commanded. robot must move on 8-connected grid.\n');
        return;
    end
    
    % compute movetime for the target
    movetime = max(1, ceil(etime(clock,t0)));
    
    if (newrobotpos(1) == robotpos(1) && newrobotpos(2) == robotpos(2))
        numofmoves = numofmoves - 1;
    end
    
    time = time + movetime;
    
    if (time > size(targettraj, 1))
        delete(ht);
        time = time - 1;
        break;
    end
    
    targetpos = targettraj(time, :);
    numofmoves = numofmoves + 1;
    % old cost calculation
%     pathcost = pathcost + envmap(robotpos(1), robotpos(2));
    % add cost proportional to time spent planning
    pathcost = pathcost + movetime*envmap(robotpos(1), robotpos(2));
    robotpos = newrobotpos;
    
    %draw moving obs here
    for i = 1:numObs
        xO = Obs(time,1+(i-1)*2);
        yO = Obs(time,2+(i-1)*2);
        x1 = xO-sizeObs(1)/2;
        x2 = xO+sizeObs(1)/2;
        y1 = yO-sizeObs(2)/2;
        y2 = yO+sizeObs(2)/2;
        x = [x1, x2, x2, x1, x1];
        y = [y1, y1, y2, y2, y1];
        plot(x, y, 'b-');
        if(robotpos(1) > x1 && robotpos(1) < x2 && robotpos(2) > y1 && robotpos(2) < y2)
            fprintf('Collision with moving object!\n');
            return;
        end
    end

    % draw the positions
    if (hr ~= -1)
        delete(hr);
        delete(ht);
    end
    hr = text(robotpos(1), robotpos(2), 'R', 'Color', 'g', 'FontWeight', 'bold');
    hr = scatter(robotpos(1), robotpos(2), 10, 'g', 'filled');
    
    for i = 1:numTar
        curX = targetpos(1+2*(i-1));
        curY = targetpos(2+2*(i-1));
        ht = text(curX, curY, 'T', 'Color', 'm', 'FontWeight', 'bold');
        ht = scatter(curX, curY, 10, 'm', 'filled');
    end

    
    pause(0.0001);
    
    % check if target is caught
    thresh = 0.5;
    
    for i = 1:numTar
        curX = targetpos(1+2*(i-1));
        curY = targetpos(2+2*(i-1));
        if (abs(robotpos(1)-curX) <= thresh && abs(robotpos(2)-curY) <= thresh)
            caught(i) = 1;
        end
    end
    if(sum(caught) == numTar)
        break;
    end
end

fprintf(1, '\nRESULT:\n');
fprintf(1, '\t target caught = %d\n', caught);
fprintf(1, '\t time taken (s) = %d\n', time);
fprintf(1, '\t moves made = %d\n', numofmoves);
fprintf(1, '\t path cost = %d\n', pathcost);