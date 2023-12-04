#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace pcl;

std::string createCubeXml(const std::string& model_name, const PointXYZ& position, float size = 0.1) {
    std::stringstream ss;
    ss << "<model name=\"" << model_name << "\">";
    ss << "<static>true</static>";
    ss << "<pose>" << position.x << " " << position.y << " " << position.z << " 0 0 0</pose>";
    ss << "<link name=\"link\">";
    ss << "<visual name=\"visual\">";
    ss << "<geometry>";
    ss << "<box><size>" << size << " " << size << " " << size << "</size></box>";
    ss << "</geometry>";
    ss << "</visual>";
    ss << "</link>";
    ss << "</model>";
    return ss.str();
}


int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_pcd_file.pcd> <output_world_file.world>" << std::endl;
        return -1;
    }

    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr cloudFiltered(new PointCloud<PointXYZ>);

    if (io::loadPCDFile<PointXYZ>(argv[1], *cloud) == -1) {
        std::cerr << "Couldn't read file " << argv[1] << std::endl;
        return -1;
    }

    // pcl::VoxelGrid<PointXYZ> filter;
    // filter.setInputCloud(cloud);
    // filter.setLeafSize(1.0f, 1.0f, 1.0f); // Adjust the voxel size
    // filter.filter(*cloudFiltered);

    std::ofstream outFile(argv[2]);
    if (!outFile) {
        std::cerr << "Failed to open the output file." << std::endl;
        return -1;
    }

    // Write the header of the world file
    outFile << "<sdf version=\"1.6\">" << std::endl;
    outFile << "<world name=\"default\">" << std::endl;

    // Write each cube model
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        if(i%10000!=0){continue;}
        std::string modelXml = createCubeXml("voxel_" + std::to_string(i), cloud->points[i], 0.1f);
        outFile << modelXml << std::endl;
    }

    // Write the footer of the world file
    outFile << "</world>" << std::endl;
    outFile << "</sdf>" << std::endl;

    outFile.close();
    std::cout << "Gazebo world file generated: " << argv[2] << std::endl;
    return 0;
}
