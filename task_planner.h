#include <iostream>
#include <regex>
#include <set>
#include <queue>
#include <vector>
#include <time.h>
#include <math.h>

using namespace std;
class Taskplanner
{   
    pair<int,int> start_point;
    queue<pair<int,int>> goal_pose;

    public:
 
        vector<pair<int,int>> goals;
        vector<pair<int,int>> results;
        

        Taskplanner(pair<int,int> start_point, queue<pair<int,int>> goal_pose)
        {
            this->start_point = start_point;
            this->goal_pose = goal_pose;
        }

        // double calc1 (pair<int,int> in1,pair<int,int> in2)
        // {
        //     return sqrt(pow((in1.first-in2.first),2)+pow((in1.second-in2.second),2));
        // }

        double calc1 (pair<int,int> in1,pair<int,int> in2)
        {
            // return search_2d(map,x_size,y_size,collision_thresh,in2.first,in2.second,curr_time,in1.first,in1.second,targetposeV,num_obj,object_traj_set,obj_size,target_steps);

            return sqrt(pow((in1.first-in2.first),2)+pow((in1.second-in2.second),2));
        }

        //input set of point coordinate goal_coor(vector of pairs) and start_point(pair)...
        void queuepreprocess()//pair start,quene<pair<int,int>> goal_pose);
        {   
            int temp = goal_pose.size();
            for (int i=0;i<temp;i++)
            {
                goals.push_back(goal_pose.front());  
                goal_pose.pop();            
            }
            cout<<"preprocess complete"<<endl;
        }

        
        vector<pair<int,int>> computeorder()
        {   
            vector<pair<int,int>> sequence;
            int loop = goals.size();
            pair<int,int> prev_vec = start_point;
            for(int i=0;i<loop;i++)
            {
                int min_val=numeric_limits<int>::max();
                pair<int,int> min_vec;
                int min_index;
                //cout<<"debug"<<endl;
                for(int j=0;j<goals.size();j++)
                {
                    if(calc1(prev_vec,goals[j])<=min_val)
                    {
                        min_val=calc1(prev_vec,goals[j]);
                        min_vec=goals[j];
                        min_index=j;
                    }
                }
                sequence.push_back(min_vec);
                goals.erase(goals.begin()+min_index);
                prev_vec = min_vec;
            }
            return sequence;
        }

        vector<pair<int,int>> computeorder_maxtomin()
        {   
            vector<pair<int,int>> sequence;
            int loop = goals.size();
            pair<int,int> prev_vec = start_point;
            for(int i=0;i<loop;i++)
            {
                int max_val=0;
                pair<int,int> max_vec;
                int max_index;
                //cout<<"debug"<<endl;
                for(int j=0;j<goals.size();j++)
                {
                    if(calc1(prev_vec,goals[j])>=max_val)
                    {
                        max_val=calc1(prev_vec,goals[j]);
                        max_vec=goals[j];
                        max_index=j;
                    }
                }
                sequence.push_back(max_vec);
                goals.erase(goals.begin()+max_index);
                prev_vec = max_vec;
            }
            return sequence;
        }
        
        
};