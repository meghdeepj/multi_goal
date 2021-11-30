#include <math.h>
#include <time.h>
#include <iostream>
#include <queue>
#include <vector>
#include <stack>
#include <algorithm>
#include <unordered_map>
#include <queue>

using namespace std;

class TrajectoryPredictor {
  public:
    vector<vector<pair<int,int>>> obstacle_traj;
    vector<vector<pair<int,int>>> observation;

    void init(double *object_traj, int steps, int num_obj, int horzn, double* obj_size)
    {
      num_obs = num_obj;
      horizon = horzn;
      obs_size = obj_size;
      obstacle_traj = vector<vector<pair<int,int>>>(num_obs, vector<pair<int,int>>(horizon, make_pair(0,0)));
      observation = vector<vector<pair<int,int>>>(num_obs, vector<pair<int,int>>(2, make_pair(0,0)));
      for(int i = 0; i < num_obs; i++)
      {
        observation[i][0].first = (int)object_traj[2*i*steps];
        observation[i][0].second = (int)object_traj[(2*i+1)*steps];
        observation[i][1].first = (int)object_traj[2*i*steps];
        observation[i][1].second = (int)object_traj[(2*i+1)*steps];
        obstacle_traj[i] = vector<pair<int,int>>(horizon, observation[i][0]);
      }
    }

    void update(double *object_traj, int steps, int t)
    {
      for(int i = 0; i < num_obs; i++)
      {
        observation[i][0].first = (int) object_traj[t-1+2*i*steps];
        observation[i][0].second = (int) object_traj[t-1+(2*i+1)*steps];
        observation[i][1].first = (int) object_traj[t+2*i*steps];
        observation[i][1].second = (int) object_traj[t+(2*i+1)*steps];
        // mexPrintf("obs %d %d %d %d\n", observation[0][0].first, observation[0][0].second, observation[0][1].first, observation[0][1].second);

      }


      for(int i = 0; i < num_obs; i++)
      {
        int dX = (observation[i][1].first - observation[i][0].first);
        int dY = (observation[i][1].second - observation[i][0].second);

        mexPrintf("dx dy %d %d\n", dX, dY);
        int x_f=1, y_f=1;
        if(dX<0) x_f=-1;
        if(dY<0) y_f=-1;
        if(dX != 0 && dY != 0)
        {
          int x = observation[i][1].first;
          int y = observation[i][0].second + (x - observation[i][0].first)/(observation[i][1].first - observation[i][0].first)* (observation[i][1].second - observation[i][0].second); 
          for(int j = 0; j < horizon; j++)
          {
            obstacle_traj[i][j] = make_pair(x, y);
            x += x_f;
          }
        }
        else if(dX == 0 && dY != 0)
        {
          int y = observation[i][1].second;
          for(int j = 0; j < horizon; j++)
          {
            obstacle_traj[i][j] = make_pair(observation[i][1].first, y);
            y += y_f;
          }
        }
        else if(dX != 0 && dY == 0)
        {
          int x = observation[i][1].first;
          for(int j = 0; j < horizon; j++)
          {
            obstacle_traj[i][j] = make_pair(x, observation[i][1].second);
            x += x_f;
          }
        }
        else
        {
          obstacle_traj[i] = vector<pair<int,int>>(horizon, observation[i][1]);
        }
      }
    // mexPrintf("obj traj %d %d\n", obstacle_traj[0][0].first, obstacle_traj[0][0].second);
    }

    bool check_plan(vector<pair<int,int>> plan, int idx)
    {
      // int lookahead = horizon + (int) max(obs_size[0], obs_size[1]);
      int lookahead = horizon;
      for(int i = idx; i < min(idx+lookahead, (int)plan.size()); i++)
      {
        if(coll_check(plan[i].first, plan[i].second))
          return false;
      }
      return true;
    }

    bool coll_check(int x, int y) //return true if given pose hits dyn object
    {   
      int szX = int(obs_size[0]);
      int szY = int(obs_size[1]);
      int curObX;
      int curObY;
      // mexPrintf("obj traj %d %d\n", obstacle_traj[0][0].first, obstacle_traj[0][0].second);

      for (int i = 0; i < num_obs; i++)
      {
        for(int j = 0; j < horizon; j++)
        {
          curObX = (int)obstacle_traj[i][j].first;
          curObY = (int)obstacle_traj[i][j].second; //pass object position
          if(x >= (curObX - szX/2)
            && x <= (curObX + szX/2)
            && y >= (curObY - szY / 2)
            && y <= (curObY + szY / 2))
            return true;
        }
      }
    // mexPrintf("\ndetect %d %d %d %d %d %d", x, y, szX, szY, curObX, curObY);
    return false;
    }

  private:
    int m=0,c=0;
    int horizon = 20;
    int num_obs = 0, steps=0;
    double* obs_size;
    


};