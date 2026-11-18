
#include <iostream>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

int main()
{
    using PointT = pcl::PointXYZ;
    using Cloud = pcl::PointCloud<PointT>;

    Cloud::Ptr cloud(new Cloud);

    // CLUSTER 1: Points around (1, 1)
    cloud->push_back(PointT(1.0, 1.0, 0.0));
    cloud->push_back(PointT(1.1, 1.0, 0.0));
    cloud->push_back(PointT(1.0, 1.1, 0.0));
    cloud->push_back(PointT(1.1, 1.1, 0.0));
    cloud->push_back(PointT(1.05, 1.05, 0.0));

    // CLUSTER 2: Points around (3, 3)
    cloud->push_back(PointT(3.0, 3.0, 0.0));
    cloud->push_back(PointT(3.1, 3.0, 0.0));
    cloud->push_back(PointT(3.0, 3.1, 0.0));
    cloud->push_back(PointT(3.1, 3.1, 0.0));
    cloud->push_back(PointT(3.05, 3.05, 0.0));

    // CLUSTER 3: Points around (5, 1)
    cloud->push_back(PointT(5.0, 1.0, 0.0));
    cloud->push_back(PointT(5.1, 1.0, 0.0));
    cloud->push_back(PointT(5.0, 1.1, 0.0));
    cloud->push_back(PointT(5.1, 1.1, 0.0));
    cloud->push_back(PointT(5.05, 1.05, 0.0));

    // Isolated noise points
    cloud->push_back(PointT(10.0, 10.0, 0.0));
    cloud->push_back(PointT(-5.0, 8.0, 0.0));

    std::cout << "Total input points: "
              << cloud->size() << "\n";

    // Create KD-tree
    pcl::search::KdTree<PointT>::Ptr tree(
        new pcl::search::KdTree<PointT>
    );

    tree->setInputCloud(cloud);

    // Configure Euclidean clustering
    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance(0.25);
    ec.setMinClusterSize(3);
    ec.setMaxClusterSize(100);

    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    // Extract clusters
    std::vector<pcl::PointIndices> clusters;

    ec.extract(clusters);

    // Display results
    std::cout << "\nDetected clusters: "
              << clusters.size() << "\n";

    for (size_t i = 0; i < clusters.size(); i++)
    {
        std::cout << "\nCluster " << i + 1 << "\n";

        for (int index : clusters[i].indices)
        {
            const auto& point = cloud->points[index];

            std::cout << "("
                      << point.x << ", "
                      << point.y << ", "
                      << point.z << ")\n";
        }

        std::cout << "Points in cluster: "
                  << clusters[i].indices.size()
                  << "\n";
    }

    return 0;
}
