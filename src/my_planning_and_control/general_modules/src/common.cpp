#include "common.h"

void Calculate_heading_and_kappa(std::shared_ptr<std::vector<PathPoint>> path)
{
    int n = path->size();
    std::vector<double> dx;
    std::vector<double> dy;
    std::vector<double> ds;
    dx.resize(n);
    dy.resize(n);
    ds.resize(n);
    for (int i = 0; i < (int)path->size(); i++)
    {
        if (i == 0)
        {
            dx[i] = path->at(i+1).x - path->at(i).x;
            dy[i] = path->at(i+1).y - path->at(i).y;
        }
        else if (i == (int)path->size()-1)
        {
            dx[i] = path->at(i).x - path->at(i-1).x;
            dy[i] = path->at(i).y - path->at(i-1).y;
        }
        else
        {
            dx[i] = (path->at(i+1).x - path->at(i-1).x)/2.0;
            dy[i] = (path->at(i+1).y - path->at(i-1).y)/2.0;
        }
        
        ds[i] = std::hypot(dx[i],dy[i]);
        path->at(i).heading = std::atan2(dy[i],dx[i]);
        
    }

    std::vector<double> dtheta;
    dtheta.resize(n);
    
    for (int i = 0; i < (int)path->size(); i++)
    {
        if (i == 0)
        {
            dtheta[i] = path->at(i+1).heading - path->at(i).heading;
        }
        else if (i == (int)path->size()-1)
        {
            dtheta[i] = path->at(i).heading - path->at(i-1).heading;
        }
        else
        {
            dtheta[i] = (path->at(i+1).heading - path->at(i-1).heading)/2.0;
        }
        
        //这里可能存在多值问题
        path->at(i).kappa = dtheta[i]/ds[i];
    }
    

    
    
}

