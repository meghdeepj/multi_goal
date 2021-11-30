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
    vector<int> start_point;
    queue<pair<int,int>> goal_pose;

    public:
        double distance_Str_G1;
        double distance_Str_G2;
        double distance_Str_G3;
        double G1_G2;
        double G1_G3;
        double G2_G3;
 
        vector<pair<int,int>> goals;
        vector<pair<int,int>> results;
        

        Taskplanner(vector<int> start_point, queue<pair<int,int>> goal_pose)
        {
            this->start_point = start_point;
            this->goal_pose = goal_pose;
        }

        double calc1 (vector<int> in1,pair<int,int> in2)
        {
            return sqrt(pow((in1[0]-in2.first),2)+pow((in1[1]-in2.second),2));
        }

        double calc2 (pair<int,int> in1,pair<int,int> in2)
        {
            return sqrt(pow((in1.first-in2.first),2)+pow((in1.second-in2.second),2));
        }


        //input set of point coordinate goal_coor(vector of pairs) and start_point(pair)...
        void euclidean()//pair start,quene<pair<int,int>> goal_pose);
        {   
            int temp = goal_pose.size();
            for (int i=0;i<temp;i++)
            {
                goals.push_back(goal_pose.front());
                cout<<goal_pose.front().first<<goal_pose.front().second<<endl;
                goal_pose.pop();            
            }
            distance_Str_G1 = calc1(start_point,goals[0]); 
            distance_Str_G2 = calc1(start_point,goals[1]);
            distance_Str_G3 = calc1(start_point,goals[2]);

            G1_G2 = calc2(goals[0],goals[1]); 
            G1_G3 = calc2(goals[0],goals[2]);
            G2_G3 = calc2(goals[1],goals[2]);

        }

        vector<pair<int,int>> computeorder()
        {
            if (distance_Str_G1<=distance_Str_G2 && distance_Str_G1<=distance_Str_G3)
            {
                if(G1_G2<=G1_G3)
                {
                    results.push_back(goals[0]);
                    results.push_back(goals[1]);
                    results.push_back(goals[2]);
                }
                else
                {
                    results.push_back(goals[0]);
                    results.push_back(goals[2]);
                    results.push_back(goals[1]);
                }
            }
            else if (distance_Str_G2<=distance_Str_G1 && distance_Str_G2<=distance_Str_G3)
            {
                if(G1_G2<=G2_G3)
                {
                    results.push_back(goals[1]);
                    results.push_back(goals[0]);
                    results.push_back(goals[2]);
                }
                else
                {
                    results.push_back(goals[1]);
                    results.push_back(goals[2]);
                    results.push_back(goals[0]);
                }
            }
            else
            {
                if(G1_G3<=G2_G3)
                {
                    results.push_back(goals[2]);
                    results.push_back(goals[0]);
                    results.push_back(goals[1]);
                }
                else
                {
                    results.push_back(goals[2]);
                    results.push_back(goals[1]);
                    results.push_back(goals[0]);
                }
            }

            cout<<"goal sequence generated "<< endl;
            
            return results;
        }
        
};