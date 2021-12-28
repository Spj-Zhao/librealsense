//
// Created by john on 2021/12/24.
//



/*
 * @Description:使用迭代最近点算法(ICP)
 * http://robot.czxy.com/docs/pcl/chapter03/registration/
 * https://www.cnblogs.com/li-yao7758258/p/6489585.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-22 16:50:22
 * @LastEditTime: 2020-10-22 17:05:21
 * @FilePath: /pcl-learning/14registration配准/1使用迭代最近点算法(ICP)/iterative_closest_point.cpp
 */

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../example.hpp" // Include short list of convenience functions for rendering

#include <iostream>               //标准输入输出头文件
#include <pcl/io/pcd_io.h>        //I/O操作头文件
#include <pcl/point_types.h>      //点类型定义头文件
#include <pcl/registration/icp.h> //ICP配准类相关头文件

#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
#include <vector>

using namespace std;
using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
              ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

float3 colors[] { { 0.8f, 0.1f, 0.3f },
                  { 0.1f, 0.9f, 0.5f },
};

void register_glfw_callbacks(window& app, state& app_state);

void draw_cuboid(window &app, state &app_state, const std::vector<pcl_ptr> &points, const Eigen::Matrix<float, 4, 4> & matrix);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);

int main(int argc, char **argv)
{
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "static icp");
    // Construct an object to manage view state
    state app_state;
    // register callbacks to allow manipulation of the pointcloud
    //register_glfw_callbacks(app, app_state);

    register_glfw_callbacks(app, app_state);

    // 定义输入和输出点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read (argv[1], *cloud_in);
    reader.read (argv[2], *cloud_out);




    // //实现一个简单的点云刚体变换，以构造目标点云，将cloud_out中的x平移0.7f米，然后再次输出数据值。
//    for (size_t i = 0; i < cloud_out->points.size(); ++i){
//        cloud_out->points[i].x = cloud_out->points[i].x + 0.3f;
//    }

//    pcl::io::savePCDFileASCII("out.pcd", *cloud_out);
//    std::cerr << "Saved " << cloud_out->points.size() << " data points to test_pcd.pcd." << std::endl;

//    // 打印这些点
//    std::cout << "Transformed " << cloud_in->points.size() << " data points:"
//              << std::endl;
//    for (size_t i = 0; i < cloud_out->points.size(); ++i)                         //打印构造出来的目标点云
//        std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

    // 创建IterativeClosestPoint的实例
    // setInputSource将cloud_in作为输入点云
    // setInputTarget将平移后的cloud_out作为目标点云
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);

    // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
    // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
    // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
    // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
    std::cout << matrix << std::endl;

    std::vector<pcl_ptr> layers;
    //layers.push_back(pcl_points);
    layers.push_back(cloud_in);
    layers.push_back(cloud_out);
    Eigen::Matrix<float, 4, 4> matrix_eig = Eigen::Matrix<float, 4, 4>(matrix);
//    Eigen::Matrix<float, 4, 4> matrix_eig;
//    matrix_eig << 1, 0, 0, 0.3,
//                  0, 1, 0 , 0,
//                  0, 0, 1, 0,
//                  0 ,0 ,0, 1;


    draw_pointcloud(app, app_state, layers);
    while (app){
        draw_cuboid(app, app_state, layers, matrix_eig);
    }

    return (0);
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_cuboid(window& app, state& app_state, const std::vector<pcl_ptr>& points, const Eigen::Matrix<float, 4, 4> & matrix)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_TEXTURE_2D);

    int color = 0;

    for (auto&& pc : points)
    {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

        glBegin(GL_POINTS);
        glColor3f(c.x, c.y, c.z);

        /* this segment actually prints the pointcloud */
        for (int i = 0; i < pc->points.size(); i++)
        {
            auto&& p = pc->points[i];
            if (p.z)

            {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
    }

    Eigen::Matrix<float, 8, 4> matrix_84;
    matrix_84 << 0.0, 0.0, 0.2, 1,
            0.0, 0.1, 0.2, 1,
            0.1, 0.1, 0.2, 1,
            0.1, 0.0, 0.2, 1,
            0.1, 0.0, 0.25, 1,
            0.0, 0.0, 0.25, 1,
            0.0, 0.1, 0.25, 1,
            0.1, 0.1, 0.25, 1;
    Eigen::Matrix<float, 4, 8> matrix_48 = matrix_84.transpose();
    matrix_48 = matrix * matrix_48;

//    float vertex_list[][3] =
//            {
//                    {0.0, 0.0, 0.2},
//                    {0.0, 0.1, 0.2},
//                    {0.1, 0.1, 0.2},
//                    {0.1, 0.0, 0.2},
//
//                    {0.1, 0.0, 0.25},
//                    {0.0, 0.0, 0.25},
//                    {0.0, 0.1, 0.25},
//                    {0.1, 0.1, 0.25}
//
//            };




    //绘制一个矩形,默认为填充模式
    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glBegin(GL_POLYGON);// 设置各个顶点，glBegin开始,glEnd结束
//    glVertex3f(0.0,0.0,0.2);
//    glVertex3f(0.0,0.1,0.2);
//    glVertex3f(0.1,0.1,0.2);
//    glVertex3f(0.1,0.0,0.2);
//
//    glVertex3f(0.1,0.0,0.25);
//    glVertex3f(0.0,0.0,0.25);
//    glVertex3f(0.0,0.1,0.25);
//    glVertex3f(0.1,0.1,0.25);

    glVertex3f(matrix_48(0,0),matrix_48(1,0),matrix_48(2,0));
    glVertex3f(matrix_48(0,1),matrix_48(1,1),matrix_48(2,1));
    glVertex3f(matrix_48(0,2),matrix_48(1,2),matrix_48(2,2));
    glVertex3f(matrix_48(0,3),matrix_48(1,3),matrix_48(2,3));
    glEnd();


    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glBegin(GL_POLYGON);// 设置各个顶点，glBegin开始,glEnd结束
    glVertex3f(matrix_48(0,3),matrix_48(1,3),matrix_48(2,3));
    glVertex3f(matrix_48(0,4),matrix_48(1,4),matrix_48(2,4));
    glVertex3f(matrix_48(0,5),matrix_48(1,5),matrix_48(2,5));
    glVertex3f(matrix_48(0,0),matrix_48(1,0),matrix_48(2,0));
    glEnd();

    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glBegin(GL_POLYGON);// 设置各个顶点，glBegin开始,glEnd结束
    glVertex3f(matrix_48(0,4),matrix_48(1,4),matrix_48(2,4));
    glVertex3f(matrix_48(0,7),matrix_48(1,7),matrix_48(2,7));
    glVertex3f(matrix_48(0,6),matrix_48(1,6),matrix_48(2,6));
    glVertex3f(matrix_48(0,5),matrix_48(1,5),matrix_48(2,5));
    glEnd();


    glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
    glBegin(GL_POLYGON);// 设置各个顶点，glBegin开始,glEnd结束
    glVertex3f(matrix_48(0,7),matrix_48(1,7),matrix_48(2,7));
    glVertex3f(matrix_48(0,6),matrix_48(1,6),matrix_48(2,6));
    glVertex3f(matrix_48(0,1),matrix_48(1,1),matrix_48(2,1));
    glVertex3f(matrix_48(0,2),matrix_48(1,2),matrix_48(2,2));
    glEnd();



    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}

void register_glfw_callbacks(window& app, state& app_state)
{
    app.on_left_mouse = [&](bool pressed)
    {
        app_state.ml = pressed;
    };

    app.on_mouse_scroll = [&](double xoffset, double yoffset)
    {
        app_state.offset_x += static_cast<float>(xoffset);
        app_state.offset_y += static_cast<float>(yoffset);
    };

    app.on_mouse_move = [&](double x, double y)
    {
        if (app_state.ml)
        {
            app_state.yaw -= (x - app_state.last_x);
            app_state.yaw = std::max(app_state.yaw, -120.0);
            app_state.yaw = std::min(app_state.yaw, +120.0);
            app_state.pitch += (y - app_state.last_y);
            app_state.pitch = std::max(app_state.pitch, -80.0);
            app_state.pitch = std::min(app_state.pitch, +80.0);
        }
        app_state.last_x = x;
        app_state.last_y = y;
    };

    app.on_key_release = [&](int key)
    {
        if (key == 32) // Escape
        {
            app_state.yaw = app_state.pitch = 0; app_state.offset_x = app_state.offset_y = 0.0;
        }
    };
}

// Handles all the OpenGL calls needed to display the point cloud
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points)
{
    // OpenGL commands that prep screen for the pointcloud
    glPopMatrix();
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    float width = app.width(), height = app.height();

    glClearColor(153.f / 255, 153.f / 255, 153.f / 255, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    gluPerspective(60, width / height, 0.01f, 10.0f);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);

    glTranslatef(0, 0, +0.5f + app_state.offset_y*0.05f);
    glRotated(app_state.pitch, 1, 0, 0);
    glRotated(app_state.yaw, 0, 1, 0);
    glTranslatef(0, 0, -0.5f);

    glPointSize(width / 640);
    glEnable(GL_TEXTURE_2D);

    int color = 0;

    for (auto&& pc : points)
    {
        auto c = colors[(color++) % (sizeof(colors) / sizeof(float3))];

        glBegin(GL_POINTS);
        glColor3f(c.x, c.y, c.z);

        /* this segment actually prints the pointcloud */
        for (int i = 0; i < pc->points.size(); i++)
        {
            auto&& p = pc->points[i];
            if (p.z)

            {
                // upload the point and texture coordinates only for points we have depth data for
                glVertex3f(p.x, p.y, p.z);
            }
        }

        glEnd();
    }



    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}






