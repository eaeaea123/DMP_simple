#include "dmp.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <iomanip>
using namespace std;

/**********将demo的数据读入数组进行计算**********/
void Input_demo_data(dmp::DMPTraj &demo)
{
    ifstream infile_pos("../trajectory_data/demo_trajectory/sinx.txt");
    const int dims = 2;       //根据demo文件中的轨迹维度设定dims
    while (!infile_pos.eof())
    {
        string line;
        getline(infile_pos, line);
        if (!line.empty())
        {
            dmp::DMPPoint Point;
            double pos;
            stringstream dataa;
            dataa << line;
            for (int i = 0; i < dims; i++)
            {
                dataa >> pos;
                Point.positions.push_back(pos);
            }
            demo.points.push_back(Point);
        }
    }
    infile_pos.close();

    int n_pts = demo.points.size();
    double delta_t = 0.001;
    for (int i = 0; i < n_pts; i++)
    {
        double t = delta_t * i;
        demo.times.push_back(t);
    }
}

/**********将计算得到的plan的数据点输出到txt中***********/
void Output_plan_data(dmp::DMPTraj &plan)
{
    const int dims = 2;
    int count = plan.points.size();
    ofstream outfile_x;
    ofstream outfile_v;
    outfile_x.open("../trajectory_data/plan_trajectory/test_x.txt", ios::ate | ios::out);
    outfile_v.open("../trajectory_data/plan_trajectory/test_v.txt  ", ios::ate | ios::out);
    outfile_x << fixed;
    outfile_v << fixed;
    for (int j = 0; j < count; j++)
    {
        for (int i = 0; i < dims; i++)
        {
            outfile_x << plan.points[j].positions[i] << " ";
            outfile_v << plan.points[j].velocities[i] << " ";
        }
        outfile_x << "\n";
        outfile_v << "\n";
    }
    outfile_x.close();
    outfile_v.close();
    cout << "All the plan of "<< count << " points already have recorded !" << endl;
}

/*****主函数*****/
int main()
{
    dmp::DMPTraj demo ;
    dmp::DMPTraj plan;
    vector<dmp::DMPData> dmp_list;
    vector<double> k_gains = {1, 1};
    vector<double> d_gains = {0.1, 0.1};
    int num_bases = 200;

    vector<double> x_0 = {5.1 ,-8.004};            //规划轨迹的起点
    vector<double> x_dot_0 = { 0,0 };     //规划轨迹起点的速度
    double t_0 = 0;
    vector<double> goal = {15,-22.856422 };         //规划轨迹的终点
    vector<double> goal_thresh = { 0.1 };  //规划轨迹的重点的逼近精度，当规划到精度范围内停止规划，可设置一个方向的逼近精度，轨迹稳定性更高
    double seg_length = -1 ;
    double tau = 3 ;     //时间缩放系数，数值越大，规划轨迹点的数量越多，规划速度越小
    double total_dt = 0.001;
    int integrate_iter = 10 ;  //积分次数，改变大小并没有太大差别
    uint8_t at_goal;

    Input_demo_data(demo);
    dmp::learnFromDemo(demo, k_gains, d_gains, num_bases, dmp_list);
    dmp::generatePlan(dmp_list, x_0, x_dot_0, t_0, goal, goal_thresh, seg_length, tau, total_dt, integrate_iter, plan, at_goal);
    Output_plan_data(plan);
    return 0;
}