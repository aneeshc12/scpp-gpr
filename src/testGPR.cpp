#include "Scancontext.h"
#include "npy.h"
#include <iostream>
#include <string>


SCManager scManager;
std::string gpr10Path = "/media/aneesh/Cherry passport vibes/RRC/Datasets/GPR/GPR10";
std::string gpr15Path = "/media/aneesh/Cherry passport vibes/RRC/Datasets/GPR/GPR15";

int num10 = 879;
int num15 = 1674;

std::pair<std::vector<pcl::PointCloud<pcl::PointXYZ>>, std::vector<float*>> gpr10All, gpr15All;

int loadGPR10(){
    for(int i = 0; i < 1; i++){

        // load one pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        if (pcl::io::loadPCDFile<pcl::PointXYZ> (gpr10Path + std::string("/000335.pcd"), *cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
          return (-1);
        }
        std::cout << "Loaded "
                  << cloud->width * cloud->height
                  << " data points from test_pcd.pcd with the following fields: "
                  << std::endl;
        for (const auto& point: *cloud)
          std::cout << "    " << point.x
                    << " "    << point.y
                    << " "    << point.z << std::endl;
        
        
        // load one pose
        

        return (0);
    }
}

int main(void){
    std::cout << scManager.testFn(5);

    // std::cout << gpr10Path + gpr15Path << "\n";
    loadGPR10();
}