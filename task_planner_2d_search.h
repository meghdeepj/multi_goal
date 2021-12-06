#include <iostream>
#include <regex>
#include <set>
#include <queue>
#include <vector>
#include <time.h>
#include <math.h>

using namespace std;
class Taskplanner_2d_search
{   
    double*	map;
    int collision_thresh;
    int x_size;     //Number of columns
    int y_size;     //Number of rows
    pair<int,int> start_point;
    queue<pair<int,int>> goal_pose;
    
    


    public:
 
        vector<pair<int,int>> goals;
        vector<pair<int,int>> results;
        

        Taskplanner_2d_search(double* map,int collision_thresh,int x_size,int y_size, pair<int,int> start_point, queue<pair<int,int>> goal_pose)
        {
            this->map = map;
            this->collision_thresh = collision_thresh;
            this->x_size = x_size;
            this->y_size = y_size;
            this->start_point = start_point;
            this->goal_pose = goal_pose;
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

        int xyToindex(int x, int y)
        {
            return (y-1)*x_size + (x-1);
        }

        pair<int, int> indexToXY(int index)
        {
            return (make_pair((index % x_size) + 1, (index / x_size) + 1));
        }

        int gridvalid(int index)
        {
                if (map[index] < collision_thresh)
            {
                if (indexToXY(index).first > 1 && indexToXY(index).first < x_size && indexToXY(index).second > 1 && indexToXY(index).second < y_size)
                {
                    return 1;
                }
            }
            return 0;
        }


        //2d dijikstra
        int dij_search(pair<int,int> prev,int goalposeX,int goalposeY)
        {
            int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
            int dy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
            int startposeX = prev.first;
            int startposeY = prev.second;
            
            priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> openList;
            unordered_map<int, bool> closedList;     //closedList record if in the closed list by index
            unordered_map<int, int> nodeInfo;  //Stores info of each grid based on the map
            bool reached = false;
            nodeInfo[xyToindex(startposeX, startposeY)] = 0;  
            openList.push(make_pair(0 , xyToindex(startposeX, startposeY)));
            double path_cost;
            while(!openList.empty()&&!reached)//&&num_goals<=target_steps
            {
                
                //Pop out the cell with lowest f value
                pair<int, int> node = openList.top();     //f-value, cell index
                openList.pop();//Remove it 

                closedList[node.second] = true;     //cell index, closed Add to the closedList
                //Iterate Over the dx dy
                for(int i=0 ; i<8 ; i++)
                {              
                    pair<int, int> successorXY = indexToXY(node.second);
                    int successorX = successorXY.first + dx[i];
                    int successorY = successorXY.second + dy[i];
                    
                    int successor_index = xyToindex(successorX, successorY);
                    if (gridvalid(successor_index)==false) // ||closedList[successor_index] == true   if it is an obsticle or already in the closed list
                        continue;
                    double h_val = 0;
                    
                    if (nodeInfo[successor_index] > h_val+nodeInfo[node.second] + map[successor_index]) //If F of new > g of parent + cost of new   
                    {      
                        nodeInfo[successor_index] = h_val+nodeInfo[node.second] + map[successor_index];  //Set f of new = g + c
                        openList.push(make_pair(nodeInfo[successor_index]  , successor_index));
                        
                    }
                    if (successorX==goalposeX&&successorY==goalposeY)
                    {
                        reached = true;
                        path_cost = nodeInfo[successor_index];
                        continue;
                    }
                }

            }
            return path_cost;

        }

        vector<pair<int,int>> computeorder_2d()
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
                    int temp_dij=dij_search(prev_vec,goals[j].first,goals[j].second);
                    if(temp_dij<=min_val)
                    {
                        min_val=temp_dij;
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

        
        
};