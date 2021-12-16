//
// Created by john on 2021/11/9.
//

#ifndef LIBREALSENSE2_REGISTER_H
#define LIBREALSENSE2_REGISTER_H
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
        PCL_INFO("Iteration Nr. %d.\n", i);

        // 存储点云以便可视化
        points_with_normals_src = reg_result;

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
        showCloudsRight(points_with_normals_tgt, points_with_normals_src);
    }

    //
    // Get the transformation from target to source
    targetToSource = Ti.inverse(); //deidao

    //
    // Transform target back in source frame
    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

    p->removePointCloud("source");
    p->removePointCloud("target");

    PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
    p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
    p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

    PCL_INFO("Press q to continue the registration.\n");
    p->spin();

    p->removePointCloud("source");
    p->removePointCloud("target");

    //add the source to the transformed target
    *output += *cloud_src;

    final_transform = targetToSource;
}
#endif //LIBREALSENSE2_REGISTER_H
