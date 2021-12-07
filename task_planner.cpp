#include "task_planner.h"
//#include "task_planner_2d_search.h"
using namespace std;

int main()

{   
    pair<int,int> start_point={1,1};
    queue<pair<int,int>> goal_pose;
    vector<pair<int,int>> results;
    goal_pose.push (make_pair(1, 2));
    goal_pose.push (make_pair(3, 1));
    goal_pose.push (make_pair(3, 3));

    Taskplanner task(start_point,goal_pose);
    task.queuepreprocess();
    //results=task.computeorder();
    results=task.computeorder_maxtomin();
    for(int i=0;i<results.size();i++)
    {
        cout<<results[i].first<<" "<<results[i].second<<endl;
    }
    //cout<<"the order of the point coordiante is "<<  <<endl;
    return 0;
}