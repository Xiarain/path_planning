#include <path_planning/obstacles.h>
#include <geometry_msgs/Point.h>

/**
 * @brief  添加障碍物（数据类型 geometry_msgs::Point）
 * @return 障碍物向量
 */
vector< vector<geometry_msgs::Point> > obstacles::getObstacleArray()
{
    vector<geometry_msgs::Point> obstaclePoint, obstaclePoint2;
    geometry_msgs::Point point;

    //first point
    point.x = 50;
    point.y = 50;
    point.z = 0;

    obstaclePoint.push_back(point);

    //second point
    point.x = 50;
    point.y = 70;
    point.z = 0;

    obstaclePoint.push_back(point);

    //third point
    point.x = 80;
    point.y = 70;
    point.z = 0;

    obstaclePoint.push_back(point);

    //fourth point
    point.x = 80;
    point.y = 50;
    point.z = 0;
    obstaclePoint.push_back(point);

    //first point again to complete the box
    point.x = 50;
    point.y = 50;
    point.z = 0;
    obstaclePoint.push_back(point);

    obstacleArray.push_back(obstaclePoint);

    //first point
    point.x = 10;
    point.y = 10;
    point.z = 0;

    obstaclePoint2.push_back(point);

    //second point
    point.x = 10;
    point.y = 40;
    point.z = 0;

    obstaclePoint2.push_back(point);

    //third point
    point.x = 40;
    point.y = 40;
    point.z = 0;

    obstaclePoint2.push_back(point);

    //fourth point
    point.x = 40;
    point.y = 10;
    point.z = 0;

    obstaclePoint2.push_back(point);

    //first point again to complete the box
    point.x = 10;
    point.y = 10;
    point.z = 0;

    obstaclePoint2.push_back(point);
    obstacleArray.push_back(obstaclePoint2);

    return obstacleArray;

}
