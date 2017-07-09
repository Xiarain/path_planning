#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <path_planning/rrt.h>
#include <path_planning/obstacles.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>

#define success false
#define running true

using namespace rrt;

bool status = running;

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &finalPath)
{
    //init headers
	  sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = finalPath.header.frame_id    = "path_planner";
	  sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = finalPath.header.stamp       = ros::Time::now();
	  sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = finalPath.ns                 = "path_planner";
	  sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = finalPath.action             = visualization_msgs::Marker::ADD;
	  sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = finalPath.pose.orientation.w = 1.0;

    //setting id for each marker
    sourcePoint.id    = 0;
	  goalPoint.id      = 1;
	  randomPoint.id    = 2;
	  rrtTreeMarker.id  = 3;
    finalPath.id      = 4;

	  //defining types
	  rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	  finalPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	  sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

	  //setting scale
	  rrtTreeMarker.scale.x = 0.2;
	  finalPath.scale.x     = 1;
	  sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 2;
    sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 2;
    sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 1;

    //assigning colors
	  sourcePoint.color.r   = 1.0f;
	  goalPoint.color.g     = 1.0f;
    randomPoint.color.b   = 1.0f;

	  rrtTreeMarker.color.r = 0.8f;
	  rrtTreeMarker.color.g = 0.4f;

	  finalPath.color.r = 0.2f;
	  finalPath.color.g = 0.2f;
	  finalPath.color.b = 1.0f;

	  sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = finalPath.color.a = 1.0f;
}

vector< vector<geometry_msgs::Point> > getObstacles()
{
    obstacles obst;
    return obst.getObstacleArray();
}

void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{

    geometry_msgs::Point point;

    point.x = tempNode.posX;
    point.y = tempNode.posY;
    point.z = 0;
    rrtTreeMarker.points.push_back(point);

    RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

    point.x = parentNode.posX;
    point.y = parentNode.posY;
    point.z = 0;

    rrtTreeMarker.points.push_back(point);
}

/**
 * @brief  检测是否在地图边界里面
 * @param tempNode 当前节点
 * @return
 */
bool checkIfInsideBoundary(RRT::rrtNode &tempNode)
{
    // TODO， this boudary is contant data, it need to adjust automatically to adopt to the map that we load.
    if(tempNode.posX < 0 || tempNode.posY < 0  || tempNode.posX > 100 || tempNode.posY > 100 ) return false;
    else return true;
}

/**
 * @brief  检测是否在障碍物里面
 * @param obstArray 障碍物队列
 * @param tempNode 当前节点
 * @return
 */
bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, RRT::rrtNode &tempNode)
{
    // TODO， this checke of the obstacle is checking whether the node in the rectangle.
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        AB = (pow(obstArray[i][0].x - obstArray[i][1].x,2) + pow(obstArray[i][0].y - obstArray[i][1].y,2));
        AD = (pow(obstArray[i][0].x - obstArray[i][3].x,2) + pow(obstArray[i][0].y - obstArray[i][3].y,2));
        AMAB = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + (( tempNode.posY - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
        AMAD = (((tempNode.posX - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + (( tempNode.posY - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));

        // 如果在点M在长方形里面，则AM×AD小于AD×AD，AM×AB小于AB×AB且他们都是大于零的，否则在相反的方向。
        //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief  产生临时点
 * @param tempNode 当前节点号
 */
void generateTempPoint(RRT::rrtNode &tempNode)
{
    int x = rand() % 150 + 1;
    int y = rand() % 150 + 1;
    // std::cout<<"Random X: "<<x << " "<<"Random Y: "<<y<<endl;
    tempNode.posX = x;
    tempNode.posY = y;
}

/**
 * @brief  向RRT队列中添加新的节点
 * @param myRRT RRT队列
 * @param tempNode 当前节点
 * @param rrtStepSize RRT步长
 * @param obstArray 障碍物队列
 * @return
 */
bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, int rrtStepSize, vector< vector<geometry_msgs::Point> > &obstArray)
{
    int nearestNodeID = myRRT.getNearestNodeID(tempNode.posX,tempNode.posY);

    RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID);

    // 通过步长将当前节点转换为二维坐标系
    double theta = atan2(tempNode.posY - nearestNode.posY,tempNode.posX - nearestNode.posX);

    // 因为这个是最近节点加上一个随机步长，所以肯定是在已有的节点继续往下生长
    tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));
    tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));

    // 检测树的增长是否在障碍物里面和是否超过边界，如果不是，则把当前节添加到RRT队列中
    // 因为在上面进行了步长的转换，所以并不是直接将随机数和边界和障碍物进行比较
    if(checkIfInsideBoundary(tempNode) && checkIfOutsideObstacles(obstArray,tempNode))
    {
        //std::cout << "增长步长：" << (rrtStepSize * cos(theta)) << "   " << rrtStepSize * sin(theta) << endl;

        tempNode.parentID = nearestNodeID;
        tempNode.nodeID = myRRT.getTreeSize();
        myRRT.addNewNode(tempNode);
        return true;
    }
    else
        return false;
}

/**
 * @brief  检测节点是否靠近目标点
 * @param X 目标点的X
 * @param Y 目标点的Y
 * @param tempNode 当前节点
 * @return
 */
bool checkNodetoGoal(int X, int Y, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2));
    if(distance < 3)
    {
        return true;
    }
    return false;
}

/**
 * @brief  设置最后的路径数据
 * @param rrtPaths 路径
 * @param myRRT RRT队列
 * @param i 路径的编号
 * @param finalpath 用于显示的路径
 * @param goalX 目标点的X
 * @param goalY 目标点的Y
 */
void setFinalPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &finalpath, int goalX, int goalY)
{
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = 0;

        finalpath.points.push_back(point);
    }

    point.x = goalX;
    point.y = goalY;
    finalpath.points.push_back(point);
}

int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"rrt_node");
	  ros::NodeHandle n;

	  //defining Publisher
	  ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);

	  //defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker;
    visualization_msgs::Marker finalPath;

    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, finalPath);

    //setting source and goal
    sourcePoint.pose.position.x = 2;
    sourcePoint.pose.position.y = 2;

    goalPoint.pose.position.x = 95;
    goalPoint.pose.position.y = 95;

    rrt_publisher.publish(sourcePoint);
    rrt_publisher.publish(goalPoint);
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    srand (time(NULL));
    //initialize rrt specific variables

    //initializing rrtTree
    RRT myRRT(2.0,2.0);
    int goalX, goalY;
    goalX = goalY = 95;

    // RRT 步长
    int rrtStepSize = 3;

    vector< vector<int> > rrtPaths;
    vector<int> path;

    // RRT 路径寻找数量，找到一定数量后，进行比较得出最好的路径，也不一定是全局最优的
    int rrtPathLimit = 1;

    int shortestPathLength = 9999;
    int shortestPath = -1;

    RRT::rrtNode tempNode;

    vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles();

    bool addNodeResult = false, nodeToGoal = false;

    while(ros::ok() && status)
    {
        // 判断是否找到足够多的路径了
        if(rrtPaths.size() < rrtPathLimit)
        {
            // 产生临时点
            generateTempPoint(tempNode);
            //std::cout<<"tempnode generated"<<endl;
            addNodeResult = addNewPointtoRRT(myRRT,tempNode,rrtStepSize,obstacleList);
            if(addNodeResult)
            {
                // std::cout<<"tempnode accepted"<< tempNode.posX << " " << tempNode.posY << endl;
                addBranchtoRRTTree(rrtTreeMarker,tempNode,myRRT);
                // std::cout<<"tempnode printed"<<endl;
                nodeToGoal = checkNodetoGoal(goalX, goalY,tempNode);
                if(nodeToGoal)
                {
                    path = myRRT.getRootToEndPath(tempNode.nodeID);
                    rrtPaths.push_back(path);
                    std::cout<<"New Path Found. Total paths "<<rrtPaths.size()<<endl;
                    //ros::Duration(10).sleep();
                    //std::cout<<"got Root Path"<<endl;
                }
            }
        }
        else //if(rrtPaths.size() >= rrtPathLimit)
        {
            status = success;
            std::cout<<"Finding Optimal Path"<<endl;
            for(int i=0; i<rrtPaths.size();i++)
            {
                if(rrtPaths[i].size() < shortestPath)
                {
                    shortestPath = i;
                    shortestPathLength = rrtPaths[i].size();
                }
            }
            setFinalPathData(rrtPaths, myRRT, shortestPath, finalPath, goalX, goalY);
            rrt_publisher.publish(finalPath);
        }


        rrt_publisher.publish(sourcePoint);
        rrt_publisher.publish(goalPoint);
        rrt_publisher.publish(rrtTreeMarker);
        //rrt_publisher.publish(finalPath);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    return 1;
}
