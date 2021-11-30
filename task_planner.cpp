#include <iostream>
#include <regex>
#include <set>
#include <queue>
#include <vector>
#include <time.h>
#include <math.h>

using namespace std;

#include "task_planner.hpp"


int main()

{   
    vector<int> start_point={1,1};
    queue<pair<int,int>> goal_pose;
    vector<pair<int,int>> results;
    goal_pose.push (make_pair(1, 2));
    goal_pose.push (make_pair(3, 1));
    goal_pose.push (make_pair(3, 3));

    

    Taskplanner task(start_point,goal_pose);
    task.euclidean();
    results=task.computeorder();
    for(int i=0;i<results.size();i++)
    {
        cout<<results[i].first<<" "<<results[i].second<<endl;
    }
    //cout<<"the order of the point coordiante is "<<  <<endl;
    return 0;
}
