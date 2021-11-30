/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <time.h>
#include <mex.h>
#include <iostream>
#include <queue>
#include <vector>
#include <stack>
#include <unordered_map>
#include <queue>

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]

#define NUMBER_OBJECTS          prhs[6]
#define OBJECT_TRAJ             prhs[7]
#define OBJECT_SIZE             prhs[8]
#define NUMBER_TARGET           prhs[9]
#define CAUGHT                  prhs[10]

/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

struct cell2d{

    pair<int,int> parent;

    int f, g, h;
    cell2d()
        : parent(-1, -1)
        , f(-1)
        , g(-1)
        , h(-1)
    {
    }
};

struct cell{

    vector<int> parent;
    int f, g, h;

    cell()
        : parent({-1, -1, -1})
        , f(-1)
        , g(-1)
        , h(-1)
    {
    }
};

typedef pair<int, vector<int>> listPair;
typedef pair<int, pair<int, int>> listPair2d;

queue<pair<int,int>> Path2d;
queue<pair<int,int>> Path;
bool have_path = false, better_2dpath = false;
vector<vector<cell2d>> grid2d;
int path_size_2d = INT_MAX;
int cost_2d = INT_MAX;
int trace_idx = 0;

bool got_goal = false;
queue<pair<int, int>> goal_poses;

bool collCheck(double* object_traj, int num_obj, int x, int y, int num, double* obj_size, int t, int steps) //return true if given pose hits dyn object
{   
    for (int i = 0; i < num_obj; i++){
        int* objPose = new int[2];
        objPose[0] = (int)object_traj[t+2*i*steps];
        objPose[1] = (int)object_traj[t+(2*i+1)*steps];   
        int szX = int(obj_size[0]);
        int szY = int(obj_size[1]);
        int curObX;
        int curObY;
        for (int i = 0; i < num; i++)
        {
            curObX = int(objPose[2 * i]);
            curObY = int(objPose[2 * i + 1]); //pass object position
            if(x > (curObX - szX/2)
                && x < (curObX + szX/2)
                && y > (curObY - szY / 2)
                && y < (curObY + szY / 2))
                return true;
        }
    }
    return false;
}

struct ArrayHasher {
    std::size_t operator()(const std::array<int, 3>& a) const {
        std::size_t h = 0;
        for (auto e : a) {
            h ^= std::hash<int>{}(e)  + 0x9e3779b9 + (h << 6) + (h >> 2); 
        }
        return h;
    }   
};

bool isValid(int x, int y, int x_size, int y_size, double* map, int collision_thresh){
    if(((int)map[GETMAPINDEX(x,y,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(x,y,x_size,y_size)] < collision_thresh)){
        return true;
    }
    return false;
}

vector<int> getPath2d(vector<vector<cell2d>> &grid, int goalposeX, int goalposeY)
{
    int row = goalposeX;
    int col = goalposeY;
    path_size_2d = 0;
 
    while (!(grid[row][col].parent.first == row
             && grid[row][col].parent.second == col)) {
        Path2d.push(make_pair(row, col));
        int temp_row = grid[row][col].parent.first;
        int temp_col = grid[row][col].parent.second;
        row = temp_row;
        col = temp_col;
        path_size_2d++;
    }
    Path2d.push(make_pair(row, col));
 
    pair<int, int> p = Path2d.front();
    return {p.first, p.second};
}

vector<int> getPath(unordered_map<array<int,3> , cell, ArrayHasher >& grid, int goalposeX, int goalposeY, int goalT, stack<pair<int,int>>& Path)
{
    int row = goalposeX;
    int col = goalposeY;
    int t = goalT;
 
    while (!(grid[{row,col,t}].parent[0] == row
             && grid[{row,col,t}].parent[1] == col && grid[{row,col,t}].parent[2] == t))
    {
        Path.push(make_pair(row, col));
        int temp_row = grid[{row,col,t}].parent[0];
        int temp_col = grid[{row,col,t}].parent[1];
        int temp_time = grid[{row,col,t}].parent[2];
        row = temp_row;
        col = temp_col;
        t = temp_time;
    }
    mexPrintf("\n sub-path found");
    pair<int, int> p = Path.top();
    return {p.first, p.second};
}

vector<int> search_2d(
    double* map, 
    int x_size, 
    int y_size, 
    int collision_thresh, 
    int robotposeX, 
    int robotposeY, 
    int curr_time,  
    int targetposeX, 
    int targetposeY,
    double* targetposeV,
    int num_obj,
    double* object_traj, 
    double* obj_size,
    int target_steps
    )
{

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    int i, j, epsilon=1;

    int goalposeX = robotposeX;
    int goalposeY = robotposeY;

    vector<vector<bool>> closed(x_size+1, vector<bool> (y_size+1, false));
    vector<vector<bool>> opened(x_size+1, vector<bool> (y_size+1, false));
    grid2d.clear();
    grid2d.resize(x_size+1, vector<cell2d>(y_size+1));
    for (i = 1; i <=x_size; i++) {
        for (j = 1; j <=y_size; j++) {
            grid2d[i][j].f = INT_MAX;
            grid2d[i][j].g = INT_MAX;
            grid2d[i][j].h = INT_MAX;
            grid2d[i][j].parent = make_pair(-1, -1);
        }
    }

    robotposeX = (int) targetposeX; 
    robotposeY = (int) targetposeY;

    i=robotposeX, j=robotposeY;
    // set<listPair2d> open2d;
    priority_queue<listPair2d, vector<listPair2d>, greater<listPair2d>> open2d;

    // all potential goal poses (from halfway to end) have g=0 (starting state for backwards search)
    grid2d[targetposeX][targetposeY].f = 0;
    grid2d[targetposeX][targetposeY].g = 0;
    grid2d[targetposeX][targetposeY].h = 0;
    grid2d[targetposeX][targetposeY].parent = make_pair(i, j);
    open2d.push(make_pair(0, make_pair(targetposeX, targetposeY)));
    opened[targetposeX][targetposeY] = true;

    int newx, newy;
    vector<int> new_pose={robotposeX, robotposeY};
    bool found_path = false;
    if(robotposeX!=goalposeX || robotposeY!=goalposeY){
        while (!open2d.empty()) {

            bool next = false;
            while(!next){           
                listPair2d curr = open2d.top();
                open2d.pop();                                                                                   // remove s from OPEN
                i = curr.second.first;
                j = curr.second.second;
                if(closed[i][j]==false){
                    next=true;
                }
            }
            closed[i][j] = true;                                                                                // insert s into CLOSED
            int gNew, hNew, fNew;
            if (i==goalposeX && j==goalposeY)                                                                   // if new pose is the goal pose
            {
                cost_2d=grid2d[i][j].g;
                new_pose= getPath2d(grid2d, goalposeX, goalposeY);
                found_path=true;
            }
            for(int dir = 0; dir < NUMOFDIRS; dir++)
            {
                newx = i + dX[dir];
                newy = j + dY[dir];

                if (newx >= 1 && newx <=x_size && newy >= 1 && newy <=y_size)                                   //if new pose is within the map
                {   
                    if(closed[newx][newy]==false && isValid(newx,newy,x_size,y_size,map,collision_thresh) && !collCheck(object_traj,num_obj,newx,newy,num_obj,obj_size,curr_time,target_steps))      // if new pose is not in CLOSED and is valid
                    {
                        gNew = grid2d[i][j].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                        hNew = 0;                                                                               // Uninformed search for max coverage
                        fNew = gNew + hNew;
                        if (grid2d[newx][newy].g == INT_MAX || grid2d[newx][newy].g > gNew)                     // if g(s')>g(s)+c(s,s')
                        {
                            open2d.push(make_pair(fNew, make_pair(newx,newy)));
                            grid2d[newx][newy].f = fNew;  
                            grid2d[newx][newy].g = gNew;                                                       // update g(s')
                            grid2d[newx][newy].h = hNew;
                            grid2d[newx][newy].parent = make_pair(i, j);
                        }
                    }
                }
            }
        }
    }
    mexPrintf("\n 2d Search ended");
    return new_pose;
}

static void planner(
    double* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    double* target_traj,
    double* targetposeV,
    int curr_time,
    double* action_ptr,
    int num_obj,
    double* object_traj_set, 
    double* obj_size,
    int num_tar,
    double* caught
    )
{    
    if(curr_time==0){                                                                                               // initialize variables for new map
        have_path=false;
        Path = queue<pair<int,int>>();
        Path2d = queue<pair<int,int>>();
        path_size_2d=INT_MAX;
        cost_2d=INT_MAX;
        better_2dpath = false;
        trace_idx = target_steps-1;
    }
    if(have_path){
        if(!better_2dpath)
        {
            if(Path.size()>1){
                Path.pop();
                pair<int, int> p = Path.front();
                // mexPrintf("\n next goal is %d,%d", p.first, p.second);
                //     mexPrintf("\n curr_time is %d", curr_time);
                action_ptr[0] = p.first;
                action_ptr[1] = p.second;
                if(collCheck(object_traj_set,num_obj,p.first,p.second,num_obj,obj_size,curr_time,target_steps))
                {
                    mexPrintf("\n next goal is %d,%d", p.first, p.second);
                    mexPrintf("\n curr_time is %d", curr_time);
                }
                // mexPrintf("\n 2d next goal is %d,%d", p.first, p.second);
                // mexPrintf("\n curr_time is %d", curr_time);
            }else{
                action_ptr[0] = target_traj[trace_idx];
                action_ptr[1] = target_traj[trace_idx+target_steps];
                trace_idx--;
            }
            return;
        }
    }
    // 9-connected grid (for 3D)
    int dX[NUMOFDIRS+1] = {0, -1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS+1] = {0, -1,  0,  1, -1,  1, -1, 0, 1};
    int dT = 1;
    double epsilon = 2;
    int buffer_time = (int) 5*(double)MAX(x_size,y_size)/200;

    if(!got_goal){
        for(int i=num_tar-1;i>=0;i--){
            goal_poses.push(make_pair(int(target_traj[2*i*target_steps+target_steps-1]), int(target_traj[(2*i+1)*target_steps+target_steps-1])));
        }
        got_goal = true;
    }
    int goalposeX = (int) goal_poses.front().first;
    int goalposeY = (int) goal_poses.front().second;
    vector<int> new_pose={robotposeX, robotposeY};

    clock_t tStart = clock();
    int num_expanded = 0;
    while(!goal_poses.empty()){
        mexPrintf("\n new run");
        double delta=0.1;
        bool found_path = false;

        stack<pair<int,int>> Path_dyn;
        vector<int> new_pose2d={robotposeX, robotposeY};
        goalposeX = (int) goal_poses.front().first;
        goalposeY = (int) goal_poses.front().second;
        mexPrintf("\n targets left: %d", goal_poses.size());
        mexPrintf("\n robotpose is %d,%d", robotposeX, robotposeY);
        mexPrintf("\n goalpose is %d,%d", goalposeX, goalposeY);
        new_pose2d = search_2d(map,x_size,y_size,collision_thresh,robotposeX,robotposeY,curr_time,goalposeX,goalposeY,targetposeV,num_obj,object_traj_set,obj_size,target_steps);

        unordered_map<array<int,3> , cell, ArrayHasher >  grid;
        unordered_map<array<int,3> , bool, ArrayHasher >  closed;
        unordered_map<array<int,3> , bool, ArrayHasher >  opened;
        // set<listPair> open;
        priority_queue<listPair, vector<listPair>, greater<listPair>> open;

        int i, j, k;

        mexPrintf("\nPath size: %d",Path.size());
        mexPrintf("\ntarget_steps: %d",target_steps);
        i=robotposeX, j=robotposeY, k=curr_time+Path.size();
        // mexPrintf("\nk: %d",k);

        cell c = {};
        c.parent = vector<int>{i, j, k};
        c.g = 0;
        c.f = 0;
        c.h = 0;
        grid[{i,j,k}] = c;
        
        open.push(make_pair(0, vector<int> {i,j,k}));
        int newx, newy, newt;
        while (!open.empty())
        {
            // listPair curr = *open.begin();                                                                                  // remove s with the smallest f(s) from OPEN;
            // open.erase(open.begin());
            bool next = false;
            // mexPrintf("open size: %d \n",open.size());
            while(!next){                                                                              // check for duplicate elements in pq
                listPair curr = open.top();
                open.pop();                                                                            // remove s from OPEN
                i = curr.second[0];
                j = curr.second[1];
                k = curr.second[2];
                if((closed.find({i,j,k}) == closed.end() || closed[{i,j,k}]==false)){
                    next=true;
                }
            }
            closed[{i,j,k}] = true;                                                                                         // insert s into CLOSED

            // int time_elapsed = buffer_time + (int)((clock() - tStart)/CLOCKS_PER_SEC);
            int time_elapsed = (int)((clock() - tStart)/CLOCKS_PER_SEC);
            int gNew, hNew, fNew;
            num_expanded++;
            // mexPrintf("target_x: %d, target_y: %d \n",target_x, target_y);
            // mexPrintf("time: %d \n",curr_time+k+time_elapsed);
            if (i==goalposeX && j==goalposeY && curr_time+k+time_elapsed<=target_steps)                                       // if goal pose is expanded
            {
                mexPrintf("\ntarget i: %d, j: %d",i, j);
                new_pose= getPath(grid, goalposeX, goalposeY, k, Path_dyn);
                found_path=true;
                break;
            }

            for(int dir = 0; dir < NUMOFDIRS; dir++)
            {
                newx = i + dX[dir];
                newy = j + dY[dir];
                newt = k + dT;
                if (newx >= 1 && newx <=x_size && newy >= 1 && newy <=y_size && curr_time+newt+time_elapsed<=target_steps)  //if new pose is within the map
                {                      
                    // if new pose is not in CLOSED and is valid
                    if( (closed.find({newx,newy,newt}) == closed.end() || closed[{newx,newy,newt}]==false) && isValid(newx,newy,x_size,y_size,map,collision_thresh) && !collCheck(object_traj_set,num_obj,newx,newy,num_obj,obj_size,curr_time+k+time_elapsed,target_steps)) 
                    {
                        gNew = grid[{i,j,k}].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                        hNew = (int) epsilon*grid2d[newx][newy].g;                                                                // use heuristic from 2D backward djikstra
                        fNew = gNew + hNew;

                        if(grid.find({newx,newy,newt})==grid.end())                                                         // if node is not in grid cosnstruct graph
                        {
                            cell c = {};
                            c.parent = vector<int>{-1,-1,-1};
                            c.g = INT_MAX;
                            c.h = INT_MAX;
                            c.f = INT_MAX;
                            grid[{newx,newy,newt}] = c;
                                
                        }  
                        if (grid[{newx,newy,newt}].g == INT_MAX || grid[{newx,newy,newt}].g > gNew)                         // if g(s')>g(s)+c(s,s')
                        {    
                            open.push(make_pair(fNew, vector<int> {newx,newy,newt}));                                       // update f in OPEN
                            grid[{newx,newy,newt}].f = fNew;  
                            grid[{newx,newy,newt}].g = gNew;                                                                // update g(s')
                            grid[{newx,newy,newt}].h = hNew;
                            grid[{newx,newy,newt}].parent =  vector<int> {i,j,k};
                        }
                    }
                }
            }  
        }
        mexPrintf("\n end run: %d", found_path);
        if(!found_path){
            new_pose = search_2d(map,x_size,y_size,collision_thresh,robotposeX,robotposeY,curr_time,goalposeX,goalposeY,targetposeV,num_obj,object_traj_set,obj_size,target_steps);
            found_path=true;
            better_2dpath=true;
        }
        if(found_path==true){
            goal_poses.pop();
            found_path = false;
            robotposeX = goalposeX;
            robotposeY = goalposeY;
            mexPrintf("\n ready for new run \n");
        }
        while(!Path_dyn.empty()){
            Path.push(Path_dyn.top());
            Path_dyn.pop();
        }
    }
    
    have_path=true;
    mexPrintf("\n robot: %d %d", robotposeX, robotposeY);
    mexPrintf("\n next goal is %d,%d \n", new_pose[0], new_pose[1]);
    robotposeX = Path.front().first;
    robotposeY = Path.front().second;
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;    
    mexPrintf("\nTime taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 11) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                 "Check number of input args.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 )//|| targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 ){//|| targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    //int targetposeX = (int)targetposeV[0];
    //int targetposeY = (int)targetposeV[1]; 
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    //Get the new stuff
    int num_obj = mxGetScalar(NUMBER_OBJECTS);
    int num_tar = mxGetScalar(NUMBER_TARGET);
    double* object_traj_set = mxGetPr(OBJECT_TRAJ);
    double* obj_size = mxGetPr(OBJECT_SIZE);
    double* caught = mxGetPr(OBJECT_SIZE);


    /* Do the actual planning in a subroutine */
    planner(map, 
            collision_thresh, 
            x_size, 
            y_size, 
            robotposeX, robotposeY, 
            target_steps, 
            targettrajV, 
            targetposeV, 
            curr_time, 
            &action_ptr[0], 
            num_obj, 
            object_traj_set,
            obj_size,
            num_tar,
            caught);
    // printf("DONE PLANNING!\n");
    return;   
}
