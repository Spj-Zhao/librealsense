
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../example.hpp" // Include short list of convenience functions for rendering

#include " register.h"

#include <iostream>

#include <pcl/filters/passthrough.h>

#include <boost/make_shared.hpp>               //boost指针相关头文件
#include <pcl/point_types.h>                   //点类型定义头文件
#include <pcl/point_cloud.h>                   //点云类定义头文件
#include <pcl/point_representation.h>          //点表示相关的头文件
#include <pcl/io/pcd_io.h>                     //PCD文件打开存储类头文件
#include <pcl/filters/voxel_grid.h>             //用于体素网格化的滤波类头文件
#include <pcl/filters/filter.h>                //滤波相关头文件
#include <pcl/features/normal_3d.h>            //法线特征头文件
#include <pcl/registration/icp.h>              //ICP类相关头文件
#include <pcl/registration/icp_nl.h>           //非线性ICP 相关头文件
#include <pcl/registration/transforms.h>       //变换矩阵类头文件
#include <pcl/visualization/pcl_visualizer.h>  //可视化类头文件

#include <unistd.h>


using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

//定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud; //申明pcl::PointXYZ数据
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;




// Struct for managing rotation of pointcloud view
struct state {
    state() : yaw(0.0), pitch(0.0), last_x(0.0), last_y(0.0),
              ml(false), offset_x(0.0f), offset_y(0.0f) {}
    double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y;
};

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// Helper functions
void register_glfw_callbacks(window& app, state& app_state);
void draw_pointcloud(window& app, state& app_state, const std::vector<pcl_ptr>& points);
void draw_cuboid(window &app, state &app_state, const std::vector<pcl_ptr> &points, const Eigen::Matrix<float, 4, 4> & matrix);

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

float3 colors[] { { 0.8f, 0.1f, 0.3f },
                  { 0.1f, 0.9f, 0.5f },
};

int main(int argc, char * argv[]) try
{
//    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense PCL Pointcloud Example");
//    // Construct an object to manage view state
    state app_state;
//    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read (argv[1], *cloud_in);


//    std::vector<pcl_ptr> layers;
//    layers.push_back(cloud_in);
//    draw_pointcloud(app, app_state, layers);




    while (app) // Application still alive?
//    {
//        draw_pointcloud(app, app_state, layers);
//    }
    {

        rs2::pointcloud pc;
        rs2::points points;

        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        // Start streaming with default recommended configuration
        pipe.start();

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        points = pc.calculate(depth);
        // Generate the pointcloud and texture mappings

        auto clouds = points_to_pcl(points);


        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(clouds);

        //设置点云距离相机的位置
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.05, 0.33);

       pass.filter(*clouds);

        //设置y轴
        pcl::PassThrough<pcl::PointXYZ> pass2;
        pass2.setInputCloud(clouds);
        pass2.setFilterFieldName("y");
        pass2.setFilterLimits(-1, 0.10);
        pass2.filter(*clouds);

//        pcl::io::savePCDFileASCII("out.pcd", *clouds);
//        std::cerr << "Saved " << clouds->points.size() << " data points to out.pcd." << std::endl;
//       //cloud_out = clouds;
//         std::vector<pcl_ptr> layers;
// //        layers.push_back(clouds);
//        layers.push_back(clouds);
//        draw_pointcloud(app, app_state, layers);


 //        pcl::PCDWriter writer;
 //        writer.write("./test_pcd.pcd", cloud_filtered, false);



        //************经典icp*************
        // 创建IterativeClosestPoint的实例
        // setInputSource将cloud_in作为输入点云
        // setInputTarget将平移后的cloud_out作为目标点云
//        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//        icp.setInputSource(cloud_in);
//        icp.setInputTarget(clouds);

        // 创建一个 pcl::PointCloud<pcl::PointXYZ>实例 Final 对象,存储配准变换后的源点云,
        // 应用 ICP 算法后, IterativeClosestPoint 能够保存结果点云集,如果这两个点云匹配正确的话
        // （即仅对其中一个应用某种刚体变换，就可以得到两个在同一坐标系下相同的点云）,那么 icp. hasConverged()= 1 (true),
        // 然后会输出最终变换矩阵的匹配分数和变换矩阵等信息。
//        pcl::PointCloud<pcl::PointXYZ> Final;
//        icp.align(Final);
//        std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
//        const pcl::Registration<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4 &matrix = icp.getFinalTransformation();
//        cout << "icp:" << endl;
//        std::cout << matrix << std::endl;

        Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
        PointCloud::Ptr temp(new PointCloud);
        // pairTransform返回从目标点云target到source的变换矩阵
        // ===================重点=========================
        pairAlign(clouds, cloud_in, temp, pairTransform, true);
        cout << "pairAlign:" << endl;
        cout << pairTransform << endl;


        std::vector<pcl_ptr> layers;
        //layers.push_back(pcl_points);
        layers.push_back(cloud_in);
        layers.push_back(clouds);
        //Eigen::Matrix<float, 4, 4> matrix_eig = Eigen::Matrix<float, 4, 4>(matrix);
        //draw_cuboid(app, app_state, layers, matrix_eig);
        draw_cuboid(app, app_state, layers, pairTransform);

    }




    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

// Registers the state variable and callbacks to allow mouse control of the pointcloud
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

    // OpenGL cleanup
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glPopAttrib();
    glPushMatrix();
}

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
            0.0, 0.12, 0.2, 1,
            0.12, 0.12, 0.2, 1,
            0.12, 0.0, 0.2, 1,
            0.12, 0.0, 0.25, 1,
            0.0, 0.0, 0.25, 1,
            0.0, 0.12, 0.25, 1,
            0.12, 0.12, 0.25, 1;
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



