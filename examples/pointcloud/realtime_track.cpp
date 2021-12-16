//
// Created by john on 2021/11/9.
//
//// License: Apache 2.0. See LICENSE file in root directory.
//// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
//
//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"          // Include short list of convenience functions for rendering
//
//#include <algorithm>            // std::min, std::max
//
//#include <iostream>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
// //#include <pcl/filters/passthrough.h>
//
//using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
//
// // Helper functions
//void register_glfw_callbacks(window& app, glfw_state& app_state);
//
//
//
//pcl_ptr points_to_pcl(const rs2::points& points)
//{
//    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//    auto sp = points.get_profile().as<rs2::video_stream_profile>();
//    cloud->width = sp.width();
//    cloud->height = sp.height();
//    cloud->is_dense = false;
//    cloud->points.resize(points.size());
//    auto ptr = points.get_vertices();
//    for (auto& p : cloud->points)
//    {
//        p.x = ptr->x;
//        p.y = ptr->y;
//        p.z = ptr->z;
//        //ptr++;
//        ++ptr;
//    }
//
//    return cloud;
//}
//
//
//
//
//int main(int argc, char * argv[]) try
//{
//    // Create a simple OpenGL window for rendering:
//    window app(1280, 720, "RealSense Pointcloud Example");
//    // Construct an object to manage view state
//    glfw_state app_state;
//    // register callbacks to allow manipulation of the pointcloud
//    register_glfw_callbacks(app, app_state);
//
//    // Declare pointcloud object, for calculating pointclouds and texture mappings
//    rs2::pointcloud pc;
//    // We want the points object to be persistent so we can display the last cloud when a frame drops
//    rs2::points points;
//
//    // Declare RealSense pipeline, encapsulating the actual device and sensors
//    rs2::pipeline pipe;
//    // Start streaming with default recommended configuration
//    pipe.start();
//
//
//    //john
//    // Wait for the next set of frames from the camera
//    auto frames = pipe.wait_for_frames();
//
//    auto depth = frames.get_depth_frame();
//
//    // Generate the pointcloud and texture mappings
//    points = pc.calculate(depth);
//
//    auto pcl_points = points_to_pcl(points);
//
//    //pcl::io::savePCDFileASCII ("./test_pcd.pcd", pcl_points->points);
//
//    pcl::PCDWriter writer;;
//    writer.write("./test_pcd.pcd", *pcl_points, false);
//
//    //打印输出存储的点云数据
//    std::cerr << "Saved " << pcl_points->points.size() << " data points to test_pcd.pcd." << std::endl;
//
//
//    // pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//    // pcl::PassThrough<pcl::PointXYZ> pass;
//    // pass.setInputCloud(pcl_points);
//    // pass.setFilterFieldName("z");
//    // pass.setFilterLimits(0.0, 1.0);
//    // pass.filter(*cloud_filtered);
//
//    // std::vector<pcl_ptr> layers;
//    // layers.push_back(pcl_points);
//    // layers.push_back(cloud_filtered);
//
//
//
//
//
//
//    while (app) // Application still alive?
//    {
//        // Wait for the next set of frames from the camera
//        auto frames = pipe.wait_for_frames();
//
//        auto color = frames.get_color_frame();
//
//        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
//        if (!color)
//            color = frames.get_infrared_frame();
//
//        // Tell pointcloud object to map to this color frame
//        pc.map_to(color);
//
//        auto depth = frames.get_depth_frame();
//
//        // Generate the pointcloud and texture mappings
//        points = pc.calculate(depth);
//
//        // Upload the color frame to OpenGL
//        app_state.tex.upload(color);
//
//        // Draw the pointcloud
//        draw_pointcloud(app.width(), app.height(), app_state, points);
//    }
//
//    return EXIT_SUCCESS;
//}
//catch (const rs2::error & e)
//{
//    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//    return EXIT_FAILURE;
//}
//catch (const std::exception & e)
//{
//    std::cerr << e.what() << std::endl;
//    return EXIT_FAILURE;
//}
//


// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "../example.hpp" // Include short list of convenience functions for rendering

//#include <pcl/point_types.h>
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

using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

//定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud; //申明pcl::PointXYZ数据
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
    MyPointRepresentation()
    {
        nr_dimensions_ = 4; //定义点的维度
    }

    // 重载copyToFloatArray方法将点转化为四维数组
    virtual void copyToFloatArray(const PointNormalT &p, float *out) const
    {
        // < x, y, z, curvature >
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature; // 曲率
    }
};


void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
    // Downsample for consistency and speed
    // \note enable this for large datasets
    PointCloud::Ptr src(new PointCloud); //存储滤波后的源点云
    PointCloud::Ptr tgt(new PointCloud); //存储滤波后的目标点云
    pcl::VoxelGrid<PointT> grid;         //滤波处理对象
    if (downsample)
    {
        grid.setLeafSize(0.05, 0.05, 0.05); //设置滤波时采用的体素大小
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*tgt);
    }
    else
    {
        src = cloud_src;
        tgt = cloud_tgt;
    }
    // 计算表面的法向量和曲率
    PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

    pcl::NormalEstimation<PointT, PointNormalT> norm_est; //点云法线估计对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    //
    // Instantiate our custom point representation (defined above) ...
    MyPointRepresentation point_representation;
    // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_representation.setRescaleValues(alpha);

    //
    // 配准
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg; // 配准对象
    reg.setTransformationEpsilon(1e-6);
    //设置收敛判断条件，越小精度越大，收敛也越慢
    // Set the maximum distance between two correspondences (src<->tgt) to 10cm大于此值的点对不考虑
    // Note: adjust this based on the size of your datasets
    reg.setMaxCorrespondenceDistance(0.1);
    // 设置点表示
    reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

    reg.setInputSource(points_with_normals_src); // 设置源点云
    reg.setInputTarget(points_with_normals_tgt); // 设置目标点云
    //
    // Run the same optimization in a loop and visualize the results
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2); ////设置最大的迭代次数，即每迭代两次就认为收敛，停止内部迭代
    for (int i = 0; i < 30; ++i) //手动迭代，每手动迭代一次，在配准结果视口对迭代的最新结果进行刷新显示
    {
        //PCL_INFO("Iteration Nr. %d.\n", i);

        // 存储点云以便可视化
        //points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        //accumulate transformation between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        //if the difference between this transformation and the previous one
        //is smaller than the threshold, refine the process by reducing
        //the maximal correspondence distance
        if (fabs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
            reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

        prev = reg.getLastIncrementalTransformation();

        // visualize current state
        //showCloudsRight(points_with_normals_tgt, points_with_normals_src);
    }

    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse(); //deidao

    //
    // Transform target back in source frame
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);



    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
}


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
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense PCL Pointcloud Example");
    // Construct an object to manage view state
    state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);

    auto pcl_points = points_to_pcl(points);

    pcl_ptr cloud_filtered1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_points);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 0.3);

    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 0.3);

    pass.setFilterFieldName ("y");
    pass.setFilterLimits (0.0, 0.3);

    pass.filter(*cloud_filtered1);

    std::vector<pcl_ptr> layers;
    layers.push_back(pcl_points);
    layers.push_back(cloud_filtered1);

    pcl::io::savePCDFileASCII("./test_pcd.pcd", *cloud_filtered1);
    std::cerr << "Saved " << cloud_filtered1->points.size() << " data points to test_pcd.pcd." << std::endl;

    pipe.stop();

    while (app) // Application still alive?
    {
        // Declare RealSense pipeline, encapsulating the actual device and sensors
        rs2::pipeline pipe;
        // Start streaming with default recommended configuration
        pipe.start();

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto pcl_points = points_to_pcl(points);

        pcl_ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass1;
        pass1.setInputCloud(pcl_points);

        //设置点云距离相机的位置
        pass1.setFilterFieldName("z");
        pass1.setFilterLimits(0.0, 0.3);

        pass1.setFilterFieldName ("x");
        pass1.setFilterLimits (0.0, 0.3);

        pass1.setFilterFieldName ("y");
        pass1.setFilterLimits (0.0, 0.3);
        pass1.filter(*cloud_filtered);

        std::vector<pcl_ptr> layers;
        layers.push_back(pcl_points);
        layers.push_back(cloud_filtered);
        draw_pointcloud(app, app_state, layers);
//        pcl::PCDWriter writer;
//        writer.write("./test_pcd.pcd", cloud_filtered, false);


        PointCloud::Ptr result(new PointCloud), source, target;
        Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;
        PointCloud::Ptr temp(new PointCloud);
        //PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());
        // pairTransform返回从目标点云target到source的变换矩阵
        pairAlign(cloud_filtered1, cloud_filtered, temp, pairTransform, true);  // ===================重点=========================
        cout << pairTransform << std::endl << '\n';

//        //把当前两两配准后的点云temp转化到全局坐标系下返回result
//        pcl::transformPointCloud(*temp, *result, GlobalTransform);
//
//        //用当前的两组点云之间的变换更新全局变换
//        GlobalTransform = GlobalTransform * pairTransform;


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


