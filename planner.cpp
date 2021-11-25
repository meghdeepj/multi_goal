/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <mex.h>
#include <iostream>
#include <vector>
#include <stack>
#include <set>
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
#define NUMBER_TARGETS          prhs[9]
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

typedef pair<int, pair<int, int> > listPair;

struct cell {

    pair<int,int> parent;

    int f, g, h;
    cell()
        : parent(-1, -1)
        , f(-1)
        , g(-1)
        , h(-1)
    {
    }
};

bool got_goal = false;
queue<pair<int, int>> goal_poses;


bool collCheck(double* object_traj, int num_obj, int x, int y, int num, double* obj_size, int t, int steps) //return true if given pose hits dyn object
{   
    mexPrintf("\ncoll check");
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

// bool collCheck(int* objPose, int x, int y, int num, double* obj_size) //return true if given pose hits dyn object
// {
//     int szX = int(obj_size[0]);
//     int szY = int(obj_size[1]);
//     int curObX;
//     int curObY;
//     for (int i = 0; i < num; i++)
//     {
//         curObX = int(objPose[2 * i]);
//         curObY = int(objPose[2 * i + 1]); //pass object position
//         if(x > (curObX - szX/2)
//             && x < (curObX + szX/2)
//             && y > (curObY - szY / 2)
//             && y < (curObY + szY / 2))
//             return true;
//     }
//     return false;
// }

bool isValid(int x, int y, int x_size, int y_size, double* map, int collision_thresh){
    if(((int)map[GETMAPINDEX(x,y,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(x,y,x_size,y_size)] < collision_thresh)){
        return true;
    };
    return false;
}

vector<int> getPath(vector<vector<cell>> &grid, int goalposeX, int goalposeY)
{
    int row = goalposeX;
    int col = goalposeY;
 
    stack<pair<int,int>> Path;
 
    while (!(grid[row][col].parent.first == row
             && grid[row][col].parent.second == col)) {
        Path.push(make_pair(row, col));
        int temp_row = grid[row][col].parent.first;
        int temp_col = grid[row][col].parent.second;
        row = temp_row;
        col = temp_col;
    }
 
    // Path.push(make_pair(row, col));
    // Path.pop();
    pair<int, int> p = Path.top();
    return {p.first, p.second};
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
    double* object_traj, 
    double* obj_size,
    double* caught
    )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

    // TODO:  implement queue of goal poses
    if(!got_goal){
        for(int i=2;i>=0;i--){
            goal_poses.push(make_pair(int(target_traj[2*i*target_steps+target_steps-1]), int(target_traj[(2*i+1)*target_steps+target_steps-1])));
        }
        got_goal = true;
    }

    double delta=0.1;

    // double dist_to_object = sqrt(((robotposeX-targetposeX)*(robotposeX-targetposeX) + (robotposeY-targetposeY)*(robotposeY-targetposeY)));

    mexPrintf("\n target steps: %d", target_steps);
    mexPrintf("\n curr_time: %d", curr_time);
    // mexPrintf("\n dist2obj: %f", dist_to_object);
    // mexPrintf("\n delta*dist: %f", delta*dist_to_object);

    // int goalposeX = (int) target_traj[curr_time+(int)(delta*dist_to_object)];
    // int goalposeY = (int) target_traj[curr_time+target_steps+(int)(delta*dist_to_object)];
    if(goal_poses.empty()){
        mexPrintf("\n %d, goal_poses is empty");
        return;
    }
     mexPrintf("\n %d caught?",caught[goal_poses.size()-1]);
    if(caught[goal_poses.size()-1] == 1) goal_poses.pop();
    int goalposeX = (int) goal_poses.front().first;
    int goalposeY = (int) goal_poses.front().second;


    // int goalposeX = targetposeX;
    // int goalposeY = targetposeY;

    // if(dist_to_object<20 || (int)(*(&target_traj + 1) - target_traj)==0){
    //     int goalposeX = targetposeX;
    //     int goalposeY = targetposeY;
    // }

    // mexPrintf("\n targetpose is %d,%d", targetposeX, targetposeY);
    mexPrintf("\n goalpose is %d,%d", goalposeX, goalposeY);

    vector<vector<bool>> closed(x_size, vector<bool> (y_size, false));
    
    int i, j;

    vector<vector<cell>> grid(x_size, vector<cell>(y_size));

    for (i = 0; i < x_size; i++) {
        for (j = 0; j < y_size; j++) {
            grid[i][j].f = INT_MAX;
            grid[i][j].g = INT_MAX;
            grid[i][j].h = INT_MAX;
            grid[i][j].parent = make_pair(-1, -1);
        }
    }

    i=robotposeX, j=robotposeY;

    grid[i][j].f = 0;
    grid[i][j].g = 0;
    grid[i][j].h = 0;
    grid[i][j].parent = make_pair(i, j);

    set<listPair> open;
    open.insert(make_pair(0, make_pair(i, j)));
    
    int newx, newy;
    vector<int> new_pose={robotposeX, robotposeY};
    bool found_path = false;
    if(robotposeX!=goalposeX || robotposeY!=goalposeY){
        while (!open.empty()) {
            listPair curr = *open.begin(); // remove s with the smallest f(s) from OPEN;
            open.erase(open.begin()); // remove s from OPEN
            i = curr.second.first;
            j = curr.second.second;
            closed[i][j] = true; //insert s into CLOSED
            // mexPrintf("\n child 2D %d, %d", i, j);
            int gNew, hNew, fNew;
            for(int dir = 0; dir < NUMOFDIRS; dir++)
            {
                newx = i + dX[dir];
                newy = j + dY[dir];
                // mexPrintf("\n child %d, %d", newx, newy);
                if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)  //if new pose is within the map
                {   
                    // mexPrintf("\n opensize %d", open.size());   
                    // mexPrintf("\n goalpose is %d,%d", goalposeX, goalposeY);
                    // mexPrintf("\n robotpose is %d,%d", newx, newy);
                    // mexPrintf("collision? %d",collCheck(object_traj,num_obj,newx,newy,num_obj,obj_size,curr_time,target_steps));
                    if (newx==goalposeX && newy==goalposeY)  //if new pose is the goal pose
                    {
                        grid[newx][newy].parent.first = i;
                        grid[newx][newy].parent.second = j;
                        new_pose= getPath(grid, goalposeX, goalposeY);
                        found_path=true;
                    }
                    else if(closed[newx][newy]==false && isValid(newx,newy,x_size,y_size,map,collision_thresh) && !collCheck(object_traj,num_obj,newx,newy,num_obj,obj_size,curr_time,target_steps)) // if new pose is not in CLOSED and is valid
                    {
                        gNew = grid[i][j].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                        hNew = (int)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                        fNew = gNew + hNew;
                        if (grid[newx][newy].g == INT_MAX || grid[newx][newy].g > gNew) //if g(s')>g(s)+c(s,s')
                        {
                            open.insert(make_pair(fNew, make_pair(newx, newy))); // insert s' in OPEN
                            // mexPrintf("\n 5");
                            grid[newx][newy].f = fNew;  
                            grid[newx][newy].g = gNew;  // update g(s')
                            grid[newx][newy].h = hNew;
                            grid[newx][newy].parent = make_pair(i, j);
                        }
                    }
                }
            }
            if(found_path) break;   
        }
    }
    

    // :::::::::::::::::::::: planner :::::::::::::::::::::::::::::::::::::::::::::::::
    mexPrintf("\n found path %d", found_path);
    mexPrintf("\n robot: %d %d", robotposeX, robotposeY);
    mexPrintf("\n next goal is %d,%d \n", new_pose[0], new_pose[1]);

    robotposeX = new_pose[0];
    robotposeY = new_pose[1];
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;

    // mexPrintf("object: %d %d;\n", (int) target_traj[curr_time],(int) target_traj[curr_time+target_steps]);
    // mexPrintf("dist_to_object: %d ;\n", dist_to_object);
    
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
                "Invalid # of args.");
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
    int num_tar = mxGetScalar(NUMBER_TARGETS);
    double* object_traj_set = mxGetPr(OBJECT_TRAJ);
    double* obj_size = mxGetPr(OBJECT_SIZE);
    double* caught = mxGetPr(CAUGHT);

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
            caught);
    // printf("DONE PLANNING!\n");
    return;   
}