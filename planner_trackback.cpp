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
#include <set>
#include <unordered_map>

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


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

stack<pair<int,int>> Path;
queue<pair<int,int>> Path2d;
bool have_path = false, better_2dpath = false;
vector<vector<cell2d>> grid2d;
int path_size_2d = INT_MAX;
int cost_2d = INT_MAX;
int trace_idx = 0;

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
        mexPrintf("%d\n", Path2d.size());
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

vector<int> getPath(unordered_map<array<int,3> , cell, ArrayHasher >& grid, int goalposeX, int goalposeY, int goalT)
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
 
    pair<int, int> p = Path.top();
    return {p.first, p.second};
}

vector<int> search_2d(double* map, int x_size, int y_size, int collision_thresh, int robotposeX, int robotposeY, int curr_time, int target_steps, double* target_traj, int targetposeX, int targetposeY){

    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    int i, j, epsilon=10;
    // int goalposeX = (int) target_traj[target_steps-1];
    // int goalposeY = (int) target_traj[target_steps-1+target_steps];


    int goalposeX = robotposeX;
    int goalposeY = robotposeY;
    // int goalposeX = (int) targetposeX;
    // int goalposeY = (int) targetposeY; 

    vector<vector<bool>> closed(x_size, vector<bool> (y_size, false));
    vector<vector<bool>> opened(x_size, vector<bool> (y_size, false));
    grid2d.clear();
    grid2d.resize(x_size, vector<cell2d>(y_size));
    for (i = 0; i < x_size; i++) {
        for (j = 0; j < y_size; j++) {
            grid2d[i][j].f = INT_MAX;
            grid2d[i][j].g = INT_MAX;
            grid2d[i][j].h = INT_MAX;
            grid2d[i][j].parent = make_pair(-1, -1);
        }
    }
    robotposeX = (int) target_traj[target_steps-1];
    robotposeY = (int) target_traj[target_steps-1+target_steps];

    i=robotposeX, j=robotposeY;

    grid2d[i][j].f = 0;
    grid2d[i][j].g = 0;
    grid2d[i][j].h = 0;
    grid2d[i][j].parent = make_pair(i, j);

    set<listPair2d> open2d;
    open2d.insert(make_pair(0, make_pair(i, j)));
    
    int newx, newy;
    vector<int> new_pose={robotposeX, robotposeY};
    bool found_path = false;
    if(robotposeX!=goalposeX || robotposeY!=goalposeY){
        while (!open2d.empty()) {
            listPair2d curr = *open2d.begin(); // remove s with the smallest f(s) from open2d;
            open2d.erase(open2d.begin()); // remove s from open2d
            i = curr.second.first;
            j = curr.second.second;
            closed[i][j] = true; //insert s into CLOSED
            int gNew, hNew, fNew;
            // mexPrintf("\n open size: %d", open2d.size());
            for(int dir = 0; dir < NUMOFDIRS; dir++)
            {
                newx = i + dX[dir];
                newy = j + dY[dir];

                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)  //if new pose is within the map
                {   
                    if (newx==goalposeX && newy==goalposeY)  //if new pose is the goal pose
                    {
                        // mexPrintf("\n 3");
                        grid2d[newx][newy].parent.first = i;
                        grid2d[newx][newy].parent.second = j;
                        cost_2d=grid2d[i][j].g  + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];;
                        new_pose= getPath2d(grid2d, goalposeX, goalposeY);
                        mexPrintf("\n cost_2d: %d", cost_2d);
                        found_path=true;
                    }
                    else if(closed[newx][newy]==false && isValid(newx,newy,x_size,y_size,map,collision_thresh)) // if new pose is not in CLOSED and is valid
                    {
                        gNew = grid2d[i][j].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                        hNew = (int) epsilon*(sqrt(2)*MIN(abs(newx-goalposeX),abs(newy-goalposeY))+(MAX(abs(newx-goalposeX),abs(newy-goalposeY))-MIN(abs(newx-goalposeX),abs(newy-goalposeY))));
                        // hNew = 0;
                        fNew = gNew + hNew;
                        // mexPrintf("\n 4");
                        if (grid2d[newx][newy].g == INT_MAX || grid2d[newx][newy].g > gNew) //if g(s')>g(s)+c(s,s')
                        {
                            if(opened[newx][newy]==false){
                                open2d.insert(make_pair(fNew, make_pair(newx, newy))); // insert s' in open2d
                                opened[newx][newy]=true;
                            }else{
                                open2d.erase(make_pair(grid2d[newx][newy].f, make_pair(newx, newy)));
                                open2d.insert(make_pair(fNew, make_pair(newx, newy)));
                            }
                            // mexPrintf("\n 5");
                            grid2d[newx][newy].f = fNew;  
                            grid2d[newx][newy].g = gNew;  // update g(s')
                            grid2d[newx][newy].h = hNew;
                            grid2d[newx][newy].parent = make_pair(i, j);
                        }
                    }
                }
            }
            if(found_path) break;   
        }
    }
    return new_pose;
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{    
    if(curr_time==0){
        have_path=false;
        Path = stack<pair<int,int>>();
        Path2d = queue<pair<int,int>>();
        path_size_2d=INT_MAX;
        cost_2d=INT_MAX;
        better_2dpath = false;
        trace_idx = target_steps-1;
    }
    if(have_path){
        if(!better_2dpath)
        {
            // mexPrintf("\nPath empty: %d", Path.size());
            if(Path.size()>1){
                Path.pop();
                pair<int, int> p = Path.top();
                action_ptr[0] = p.first;
                action_ptr[1] = p.second;
            }else{
                action_ptr[0] = robotposeX;
                action_ptr[1] = robotposeY;
            }
            // mexPrintf("\n Next pose: %.2f,%.2f", action_ptr[0], action_ptr[1]);
            return;
        }else{
            mexPrintf("\nPath empty: %d", Path2d.size());
            if(Path2d.size()>1){
                Path2d.pop();
                pair<int, int> p = Path2d.front();
                action_ptr[0] = p.first;
                action_ptr[1] = p.second;
            }else{
                action_ptr[0] = target_traj[trace_idx];
                action_ptr[1] = target_traj[trace_idx+target_steps];
                trace_idx--;
            }
            // mexPrintf("\n Next pose: %.2f,%.2f", action_ptr[0], action_ptr[1]);
            return;
        }
    }

    vector<int> new_pose2d={robotposeX, robotposeY};
    // new_pose2d = search_2d(map,x_size,y_size,collision_thresh,robotposeX,robotposeY,curr_time,target_steps,target_traj,targetposeX,targetposeY);


    // 8-connected grid
    int dX[NUMOFDIRS+1] = {0, -1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS+1] = {0, -1,  0,  1, -1,  1, -1, 0, 1};
    int dT = 1;

    vector<int> new_pose={robotposeX, robotposeY};
    clock_t tStart = clock();
    double delta=0.1, epsilon=50;
    int buffer_time = 10;
    // double dist_to_object = sqrt(((robotposeX-targetposeX)*(robotposeX-targetposeX) + (robotposeY-targetposeY)*(robotposeY-targetposeY)));
    
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];

    // mexPrintf("\n targetpose is %d,%d", targetposeX, targetposeY);
    // mexPrintf("\n goalpose is %d,%d", goalposeX, goalposeY);

    // vector<vector<vector<cell>>> grid(x_size, vector<vector<cell>> (y_size, vector<cell>));  // 3d grid to store the graph
    unordered_map<array<int,3> , cell, ArrayHasher >  grid;
    // unordered_map<vector<int>,cell> grid;
    unordered_map<array<int,3> , bool, ArrayHasher >  closed;
    unordered_map<array<int,3> , bool, ArrayHasher >  opened;
    // unordered_map<vector<int>,bool> closed;   // closed list
    set<listPair> open;

    int i, j, k;

    i=robotposeX, j=robotposeY, k=0;
    cell c = {};
    c.parent = vector<int>{i, j, k};
    c.g = 0;
    c.f = 0;
    c.h = 0;
    grid[{i,j,k}] = c;
    
    open.insert(make_pair(0, vector<int> {i,j,k}));
    int newx, newy, newt;
    bool found_path = false;
    int num_expanded = 0;
    while (!open.empty())
    {
        listPair curr = *open.begin(); // remove s with the smallest f(s) from OPEN;
        open.erase(open.begin()); // remove s from OPEN
        
        i = curr.second[0];
        j = curr.second[1];
        k = curr.second[2];
        int time_elapsed = buffer_time + (int)((clock() - tStart)/CLOCKS_PER_SEC);
        closed[{i,j,k}] = true; //insert s into CLOSED
        if((k+time_elapsed>=target_steps-curr_time-1 && path_size_2d+time_elapsed<=target_steps-curr_time) || (x_size>1000 && y_size>1000))
        {
            new_pose2d = search_2d(map,x_size,y_size,collision_thresh,robotposeX,robotposeY,curr_time,target_steps,target_traj,targetposeX,targetposeY);
            mexPrintf("\n found path 2d");
            new_pose = new_pose2d;
            have_path=true;
            better_2dpath=true;
            found_path=true;
            break;
        }
        int gNew, hNew, fNew;
        num_expanded++;
        // mexPrintf("\n num_expanded: %d", num_expanded);
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            newx = i + dX[dir];
            newy = j + dY[dir];
            newt = k + dT;      //use absolute time 
            // mexPrintf("\n child %d, %d", newx, newy);
            int target_x = (int) target_traj[curr_time+time_elapsed+newt];
            int target_y = (int) target_traj[curr_time+time_elapsed+newt+target_steps];
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && curr_time+newt+time_elapsed<=target_steps)  //if new pose is within the map
            {                      
                if (newx==target_x && newy==target_y)  //if new pose is the goal pose at that time
                {
                    mexPrintf("\n newt: %d, time_elapsed: %d", newt, time_elapsed);
                    mexPrintf("\n target_x: %d, target_y: %d", target_x, target_y);
                    grid[{newx,newy,newt}].parent = {i,j,k};
                    new_pose= getPath(grid, target_x, target_y, newt);
                    // new_pose= getPath(grid, goalposeX, goalposeY, newt);
                    found_path=true;
                    have_path=true;
                }
                else if( (closed.find({newx,newy,newt}) == closed.end() || closed[{newx,newy,newt}]==false) && isValid(newx,newy,x_size,y_size,map,collision_thresh)) // if new pose is not in CLOSED and is valid
                {
                    gNew = grid[{i,j,k}].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                    // gNew = grid[{i,j,k}].g + 1;
                    hNew = (int) epsilon*(sqrt(2)*MIN(abs(newx-target_x),abs(newy-target_y))+(MAX(abs(newx-target_x),abs(newy-target_y))-MIN(abs(newx-target_x),abs(newy-target_y))));
                    // hNew = grid2d[newx][newy].g;
                    fNew = gNew + hNew;
                    // mexPrintf("\n 4");
                    if(grid.find({newx,newy,newt})==grid.end())
                    {  // if node is not in grid construct graph
                            cell c = {};
                            c.parent = vector<int>{-1,-1,-1};
                            c.g = INT_MAX;
                            c.h = INT_MAX;
                            c.f = INT_MAX;
                            grid[{newx,newy,newt}] = c;
                            
                    }  
                    if (grid[{newx,newy,newt}].g == INT_MAX || grid[{newx,newy,newt}].g > gNew) //if g(s')>g(s)+c(s,s')
                    {
                        if(opened[{newx,newy,newt}]==false || (opened.find({newx,newy,newt}) == opened.end())){
                                open.insert(make_pair(fNew,  vector<int> {newx,newy,newt})); // insert s' in OPEN
                                opened[{newx,newy,newt}]=true;
                        }else{
                                open.erase(make_pair(grid[{newx,newy,newt}].f,  vector<int> {newx,newy,newt}));
                                open.insert(make_pair(fNew,  vector<int> {newx,newy,newt}));
                        }               
                        // mexPrintf("\n 5");
                        grid[{newx,newy,newt}].f = fNew;  
                        grid[{newx,newy,newt}].g = gNew;  // update g(s')
                        grid[{newx,newy,newt}].h = hNew;
                        grid[{newx,newy,newt}].parent =  vector<int> {i,j,k};
                    }
                }
            }
        }
        if(found_path) break;   
    }
    if(!found_path){
        new_pose = search_2d(map,x_size,y_size,collision_thresh,robotposeX,robotposeY,curr_time,target_steps,target_traj,targetposeX,targetposeY);
        have_path=true;
        better_2dpath=true;
        found_path=true;
    }

    // :::::::::::::::::::::: planner :::::::::::::::::::::::::::::::::::::::::::::::::
    // mexPrintf("\n found path 3d %d", found_path);
    // mexPrintf("\n robot: %d %d", robotposeX, robotposeY);
    // mexPrintf("\n next goal is %d,%d \n", new_pose[0], new_pose[1]);

    robotposeX = new_pose[0];
    robotposeY = new_pose[1];
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    mexPrintf("\nTime taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    mexPrintf("object: %d %d;\n", (int) target_traj[curr_time],(int) target_traj[curr_time+target_steps]);
    mexPrintf("num_expanded: %d ;\n", num_expanded);
    
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
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
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
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}