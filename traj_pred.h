#include <math.h>
#include <time.h>
#include <iostream>
#include <queue>
#include <vector>
#include <stack>
#include <unordered_map>
#include <queue>

using namespace std;

class TrajectoryPredictor {
  public:
    vector<vector<pair<int,int>>> obstacle_traj;
    vector<vector<pair<int,int>>> observation;

    TrajectoryPredictor(int num_obj, int horzn, double* obj_size)
    {
      num_obs = num_obj;
      horizon = horzn;
      obs_size = obj_size;
      obstacle_traj = vector<vector<pair<int,int>>>(num_obs, vector<pair<int,int>>(horizon, make_pair(0,0)));
      observation = vector<vector<pair<int,int>>>(num_obs, vector<pair<int,int>>(2, make_pair(0,0)));
    }

    void init(double *object_traj, int steps)
    {
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
        observation[i][0].first = observation[i][1].first;
        observation[i][0].second = observation[i][1].second;
        observation[i][1].first = (int) object_traj[t+2*i*steps];
        observation[i][1].second = (int) object_traj[t+(2*i+1)*steps];
      }

      for(int i = 0; i < num_obs; i++)
      {
        int dX = (observation[i][1].first - observation[i][0].first);
        int dY = (observation[i][1].second - observation[i][0].second);
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
    }

    bool check_plan()

    bool coll_check(int x, int y, int num, double* obj_size, int t, int steps) //return true if given pose hits dyn object
    {   
      for(int i = 0; i < num_obs; i++)
      {
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

  private:
    int m=0,c=0;
    int horizon = 20;
    int num_obs = 0, steps=0;
    double* obs_size;
    


};w