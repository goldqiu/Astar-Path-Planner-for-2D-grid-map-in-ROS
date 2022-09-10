#include <iostream>
#include <fstream>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <assert.h>
#include <list>
#include <iostream>
#include <queue>
#include "tic_toc.hpp"
#include <tf/transform_datatypes.h>

using namespace std;

#define GET_ARRAY_LEN(array) (sizeof(array)/sizeof(array[0]))

// ros related
ros::Subscriber _pts_sub,_map_2d_sub,fused_odom;
ros::Publisher  _grid_path_vis_pub, global_grid_map_pub;
ros::Publisher pubGlobalPath; // path is published in pose array format 

void rcvWaypointsCallback(const nav_msgs::Path & wp);

nav_msgs::OccupancyGrid temp_msg;
nav_msgs::OccupancyGrid temp2_msg;
uint16_t temp_width,temp_height;
uint32_t temp_size;
double origin_x,origin_y,gold_x,gold_y,start_x,start_y;
uint8_t flag_way_point = 0;
struct Point
{
    long x;		// 宽
    long y;		// 高
    Point(long tx = 0, long ty = 0) : x(tx), y(ty) {}
 
    // 两个坐标距离：横坐标距离 + 纵坐标距离
    long operator - (const Point& p)
    {
        return abs(x - p.x) + abs(y - p.y);
    }
 
    bool operator == (const Point& p)
    {
        return x == p.x && y == p.y;
    }
};
 
struct PointV : public Point
{
    long value;	// 0 :无障碍; 1:有障碍
    PointV(long nx = 0, long ny = 0, long v = 0) : Point(nx, ny), value(v) {}
};
 
struct PointAStart : public Point
{
    long f, g, h;
    bool visited;	// 是否被访问过，0：未被访问，1已经被访问
    Point parentNode;
 
    PointAStart(long tf = 0, long tg = 0, long th = 0, long tx = 0, long ny = 0) : Point(tx, ny), f(tf), g(tg), h(th), visited(false), parentNode() {};
    bool operator < (const PointAStart& t) { return f < t.f; }
    void SetFGH(long tf, long tg, long th) { f = tf; g = tg, h = th; }
};
 
// 重写仿函数, 优先队列元素大小比较
struct comp //重写仿函数
{
    bool operator() (PointAStart* a, PointAStart* b)
    {
        return a->f > b->f; //小顶堆
    }
};
vector<Point> result;
void visGridPath( vector<Point>& nodes);


class AStar
{
public:
    // arr 是一个二维数组 
    // s 起点; e 终点
    vector<Point> operator()(const vector<vector<int>>& arr, Point s, Point e)
    {
        if (arr.empty() || s == e)
            return {};
 
        int lenY = (int)arr.size() - 1;		    // 高
        int lenX = (int)arr[0].size() - 1;		// 宽
 
        if (s.x > lenX || s.y > lenY || e.x > lenX || e.y > lenY)
            return {};
 
        if (arr[s.y][s.x] != 0 || arr[e.y][e.x] != 0)
            return {};
 
        for (int i = 0; i < lenY; ++i)
            assert(lenX == (int)arr[i].size() - 1);
 
        vector<vector<PointAStart>> pArr(lenY + 1, vector<PointAStart>(lenX + 1));	// 父结点
        priority_queue<PointAStart*, vector<PointAStart*>, comp> openList; // by 2020/07/30 改用优先队列
 
        int g = 0, h = (s - e) * 10, f = g + h;
        PointAStart pt(f, g, h, s.x, s.y);
        pt.visited = true;
        pArr[s.y][s.x] = pt;
        openList.push(&pArr[s.y][s.x]);
 
        bool seek = true;
        const int dirs[8][3] = { {0,1,10},{1,1,14},{1,0,10},{1,-1,14},{0,-1,10},{-1,-1,14},{-1,0,10},{-1,1,14} };//8个移动方向（右，右上，上，左上，左，左下，下，右下）
        while (seek && !openList.empty())
        {
            PointAStart& p = *openList.top();
            openList.pop();
            p.visited = true;
 
            for (int i = 0; i < GET_ARRAY_LEN(dirs) && seek; ++i)
            {
                Point t(p.x + dirs[i][1], p.y + dirs[i][0]);
                // t 需要未被访问
                if (t.x < 0 || t.x > lenX || t.y < 0 || t.y > lenY || arr[t.y][t.x] == 1 || pArr[t.y][t.x].visited)
                    continue;
 
                // 找父节点
                g = p.g + dirs[i][2];
                h = (t - e) * 10;
                f = g + h;
                int minf = f;
                PointAStart newPoint(f, g, h, t.x, t.y);
                newPoint.visited = 1;
                newPoint.parentNode.x = p.x;
                newPoint.parentNode.y = p.y;
                for (int j = 0; j < GET_ARRAY_LEN(dirs); ++j)
                {
                    Point pp(t.x + dirs[j][1], t.y + dirs[j][0]);  //父节点Parent Point
                    // 父节点pp, 在需要已经被访问
                    if (pp.x < 0 || pp.x > lenX || pp.y < 0 || pp.y > lenY || arr[pp.y][pp.x] == 1 || !pArr[pp.y][pp.x].visited)
                        continue;
 
                    g = pArr[pp.y][pp.x].g + dirs[j][2];
                    f = g + h;
                    if (f <= minf)
                    {
                        minf = f;
                        f = g + h;
 
                        newPoint.SetFGH(f, g, h);
                        newPoint.parentNode = pp;
                    }
 
                }
 
                pArr[t.y][t.x] = newPoint;
                openList.push(&pArr[t.y][t.x]);
 
                if (t == e)
                    seek = false;
            }
        }
 
        if (!pArr[e.y][e.x].visited)
        {
            cout << "无法到达" << endl;
            return {};
        }
        else
        {
            vector<Point> path;
            path.push_back(e);
            Point p = pArr[e.y][e.x].parentNode;
            while (true)
            {
                if (!pArr[p.y][p.x].visited)
                {
                    cout << "无法到达" << endl;
                    return {};
                }
 
                path.push_back(p);
                if (p == s)
                    break;
 
                p = pArr[p.y][p.x].parentNode;
            }
 
            reverse(path.begin(), path.end());
            SetVisitedCount(pArr);	// 辅助测试，记录访问的结点数
 
            return path;
        }
    }

    // 辅助测试，用于获取访问的结点数
    void SetVisitedCount(const vector<vector<PointAStart>>& pArr)
    {
        visitCount = 0;
        for (int i = 0; i < pArr.size(); ++i)
        {
            for (int j = 0; j < pArr[i].size(); ++j)
            {
                if (pArr[i][j].visited)
                    ++visitCount;
            }
        }
    }
 
    int visitCount;
};

//
// 测试 用例 START
void test(const char* testName, const vector<vector<int>>& arr, Point s, Point e)
{
    AStar as;
    result = as(arr, s, e);
 
    cout << testName << "[" << as.visitCount << ", " << result.size() << "]";

    // for (long i = 0; i < result.size(); ++i)
    // {
    //     cout << ", (" << result[i].x << "," << result[i].y << ")";
    // }
    cout << endl;
}

void rcvWaypointsCallback(const nav_msgs::Path & wp)
{     
    // if( wp.poses[0].pose.position.z < 0.0 || _has_map == false )
    //     return;

    start_x = wp.poses[0].pose.position.x-origin_x;
    start_y = wp.poses[0].pose.position.y-origin_y;


    TicToc t_plan; //创建计时对象
    uint16_t j = 0,k = 0;
    vector<vector<int>> arr(temp_width,vector<int>(temp_height,1));
    
    // if(flag_way_point >= 1)
    {
            for (uint32_t i = 0; i < temp_size; i++)
        { 
            if(j >= temp_width) 
            {
                k++;
                j = 0;
            }
            if(temp_msg.data[i]>1)  arr[j][k] = 1; 
            else   arr[j][k] = 0;
            j++;
        }

        uint16_t x1 = (uint16_t)(gold_x);
        uint16_t y1 = (uint16_t)(gold_y);

        uint16_t x2 = (uint16_t)(start_x);
        uint16_t y2 = (uint16_t)(start_y);

        Point s(y2,x2);
        Point e(y1,x1);
        test("Test()", arr, s, e);  

        visGridPath(result);
        printf("plan time %f s \n", t_plan.toc()); //打印配准需要的时间
        flag_way_point = 2;
    }

    // start_x = wp.poses[0].pose.position.x;
    // start_y = wp.poses[0].pose.position.y;

}

void grid_map_Callback(const nav_msgs::OccupancyGrid& msg)
{     
    temp_msg = msg;
    temp_width = temp_msg.info.width;
    temp_height = temp_msg.info.height;
    temp_size = temp_msg.data.size();
    origin_x = temp_msg.info.origin.position.x;
    origin_y = temp_msg.info.origin.position.y;
    std::cout<<"origin_x=  "<<origin_x<<std::endl;
    std::cout<<"origin_y=  "<<origin_y<<std::endl;
    // temp_msg.info.origin.orientation.w = 1 ;
    // temp_msg.info.origin.orientation.x = 0 ;
    // temp_msg.info.origin.orientation.y = 0 ;
    // temp_msg.info.origin.orientation.z = 0 ;
    // temp_msg.info.origin.position.x = 0;
    // temp_msg.info.origin.position.y = 0;
    // temp_msg.info.origin.position.z = 0;    
    global_grid_map_pub.publish(temp_msg);

    TicToc t_plan; //创建计时对象
    uint16_t j = 0,k = 0;
    vector<vector<int>> arr(temp_width,vector<int>(temp_height,1));
    if(flag_way_point == 2)
    {
        for (uint32_t i = 0; i < temp_size; i++)
    { 
        if(j >= temp_width) 
        {
            k++;
            j = 0;
        }
        if(temp_msg.data[i]>1)  arr[j][k] = 1; 
        else   arr[j][k] = 0;
        j++;
    }

    uint16_t x1 = (uint16_t)(gold_x);
    uint16_t y1 = (uint16_t)(gold_y);

    uint16_t x2 = (uint16_t)(start_x);
    uint16_t y2 = (uint16_t)(start_y);

    Point s(y2,x2);
    Point e(y1,x1);
    test("Test()", arr, s, e);  
    visGridPath(result);
    nav_msgs::Path globalPath;
    globalPath.poses.clear();
    for (int i = 0; i < result.size(); i++){
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.pose.position.x = result[i].y + origin_x;
            pose.pose.position.y = result[i].x + origin_y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            globalPath.poses.push_back(pose);
    }
    globalPath.header.frame_id = "/map";
    globalPath.header.stamp = ros::Time::now();
    pubGlobalPath.publish(globalPath);
    printf("plan time %f s \n", t_plan.toc()); //打印配准需要的时间
    }
}

void fused_odom_Callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr)
{     
    gold_x = odom_msg_ptr->pose.pose.position.x - origin_x;
    gold_y = odom_msg_ptr->pose.pose.position.y - origin_y;
    if(flag_way_point == 0)  flag_way_point = 1;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo_node");
    ros::NodeHandle nh("~");
	std::string Map_topic;
	nh.param<std::string>("mapTopic",Map_topic,"/grid_map_global")
    _pts_sub  = nh.subscribe( "waypoints", 1, rcvWaypointsCallback );
    _map_2d_sub = nh.subscribe(Map_topic, 1, grid_map_Callback );
    fused_odom = nh.subscribe( "/laser_localization", 1, fused_odom_Callback );

    _grid_path_vis_pub            = nh.advertise<visualization_msgs::Marker>("grid_path_vis", 1);
    global_grid_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 100);
    pubGlobalPath = nh.advertise<nav_msgs::Path>("/global_path", 5);

    ros::Rate rate(1);
    bool status = ros::ok();
    // uint16_t j = 0,k = 0;
    // vector<vector<int>> arr =
    // {
    //     {0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,0,1,1,1,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,0,0},
    //     {0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,1,1,0,1,0,0,0,0,0,0,0,0},
    //     {0,1,1,1,1,0,0,0,1,0,0,0,0,1,0,1,0,0,1,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,1,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,1,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,0,0,1,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
    //     {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
    // };
 
    // origin_x = temp_msg.info.origin.position.x;
    // origin_y = temp_msg.info.origin.position.y;
    // temp2_msg.header.frame_id = "map";
    // temp2_msg.info.resolution = 1;
    // temp2_msg.info.origin.orientation.w = 1 ;
    // temp2_msg.info.origin.orientation.x = 0 ;
    // temp2_msg.info.origin.orientation.y = 0 ;
    // temp2_msg.info.origin.orientation.z = 0 ;
    // temp2_msg.info.origin.position.x = 0;
    // temp2_msg.info.origin.position.y = 0;
    // temp2_msg.info.origin.position.z = 0;
    // temp2_msg.info.width = arr.size();
    // temp2_msg.info.height = arr[0].size();
    // Point s(2, 7);
    // Point e(17, 5);

    // temp2_msg.data.resize(temp2_msg.info.width * temp2_msg.info.height);
    // temp2_msg.data.assign(temp2_msg.info.width * temp2_msg.info.height, 0);

    // test("Test11()", arr, s, e);
    //     for (uint32_t i = 0; i < 378; i++)
    // { 
    //     if(j >= 14) 
    //     {
    //         k++;
    //         j = 0;
    //     }
    //     if(arr[j][k] == 1)  temp2_msg.data[i] = 100;
    //     else temp2_msg.data[i] = 0;
    //     j++;
    // }

    while(status) 
    {
        ros::spinOnce();  
        // global_grid_map_pub.publish(temp2_msg);
        // visGridPath(result);
        status = ros::ok();
        rate.sleep();
    }

    return 0;
}

void visGridPath( vector<Point>& nodes)
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "map";
    node_vis.header.stamp = ros::Time::now();
    
    node_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
   
    node_vis.color.a = 1.0;
    node_vis.color.r = 0.5;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.8;
    
    node_vis.scale.x = 1;
    node_vis.scale.y = 1;
    node_vis.scale.z = 1;

    geometry_msgs::Point pt;
    for(int i = 0; i < nodes.size(); i++)
    {
        pt.x = nodes[i].y + origin_x;
        pt.y = nodes[i].x + origin_y;
        // pt.x = nodes[i].y ;
        // pt.y = nodes[i].x ;
        pt.z = 0;

        node_vis.points.push_back(pt);
    }

    _grid_path_vis_pub.publish(node_vis);
}


