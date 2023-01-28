#include "Scancontext.h"
#include "npy.h"
#include <iostream>
#include <string>
#include <cmath>


SCManager scManagerGPR10;
std::string gpr10Path = "/media/aneesh/Cherry passport vibes/RRC/Datasets/GPR/GPR10/";
std::string gpr15Path = "/media/aneesh/Cherry passport vibes/RRC/Datasets/GPR/GPR15/";

int num10 = 878;        // 780 and 98 split
int num15 = 1673;

std::vector<std::pair<pcl::PointCloud<pcl::PointXYZ>, std::vector<double>>> gpr10All, gpr15All;
std::vector<std::vector<double>> gpr10Odometry, gpr15Odometry;

std::vector<std::pair<std::pair<int, int>, float>> gpr10loopsAndDistances;

int loadGPR10(){
    for(int i = 1; i <= (int) (780); i++){

        // // get path
        char buffer[256]; 
        sprintf(buffer, "%06d", i);
        std::string name = std::string(buffer);


        // load one pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        std::string fullPath = gpr10Path + name + std::string(".pcd");

        std::cout << fullPath << '\n';
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (gpr10Path + name + std::string(".pcd"), *cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
          return (-1);
        }

        scManagerGPR10.makeAndSaveScancontextAndKeys(*cloud);

        
        

        // load one pose
        std::vector<unsigned long> shape {};
        bool fortran_order;
        std::vector<double> pose, myPose;

        npy::LoadArrayFromNumpy(gpr10Path + name + std::string("_pose6d.npy"), shape, fortran_order, pose);
        myPose = std::vector<double>(pose.begin(), pose.begin() + 3);
        gpr10Odometry.push_back(myPose);

    }

    return(0);
}

double calcDistance(std::vector<double> a, std::vector<double> b){
    double dist = 0;
    dist += (a[0] - b[0])*(a[0] - b[0]);
    dist += (a[1] - b[1])*(a[1] - b[1]);
    dist += (a[2] - b[2])*(a[2] - b[2]);

    dist = sqrt(dist);
    return dist;
}

int evaluateGPR10() {
    for(int i = 781; i <= (int) (878); i++){

        // // get path
        char buffer[256]; 
        sprintf(buffer, "%06d", i);
        std::string name = std::string(buffer);


        // load one pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        std::string fullPath = gpr10Path + name + std::string(".pcd");

        std::cout << fullPath << '\n';
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (gpr10Path + name + std::string(".pcd"), *cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
          return (-1);
        }

        scManagerGPR10.makeAndSaveScancontextAndKeys(*cloud);

        // load one pose
        std::vector<unsigned long> shape {};
        bool fortran_order;
        std::vector<double> pose, myPose;

        npy::LoadArrayFromNumpy(gpr10Path + name + std::string("_pose6d.npy"), shape, fortran_order, pose);
        myPose = std::vector<double>(pose.begin(), pose.begin() + 3);
        gpr10Odometry.push_back(myPose);

        std::pair<int, float> loop = scManagerGPR10.detectLoopClosureID();
        if(loop.first != -1){
            std::pair<int, int> ids(loop.first, i);
            std::cout << "Loop matched! ids: " << loop.first << " and " << i << std::endl;

            std::cout << "Yaw diff: " << loop.second  << std::endl;


            std::cout << "Distance: " << calcDistance(myPose, gpr10Odometry[loop.first]) << i << std::endl;
        }
    }

    for(int i = 1; i <= (int) (1); i++){

        // // get path
        char buffer[256]; 
        sprintf(buffer, "%06d", i);
        std::string name = std::string(buffer);


        // load one pcd
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        std::string fullPath = gpr10Path + name + std::string(".pcd");

        std::cout << fullPath << '\n';
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (gpr10Path + name + std::string(".pcd"), *cloud) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
          return (-1);
        }

        scManagerGPR10.makeAndSaveScancontextAndKeys(*cloud);

        // load one pose
        std::vector<unsigned long> shape {};
        bool fortran_order;
        std::vector<double> pose, myPose;

        npy::LoadArrayFromNumpy(gpr10Path + name + std::string("_pose6d.npy"), shape, fortran_order, pose);
        myPose = std::vector<double>(pose.begin(), pose.begin() + 3);
        gpr10Odometry.push_back(myPose);

        std::pair<int, float> loop = scManagerGPR10.detectLoopClosureID();
        if(loop.first != -1){
            std::pair<int, int> ids(loop.first, i);
            std::cout << "Loop matched! ids: " << loop.first << " and " << i << std::endl;

            std::cout << "Yaw diff: " << loop.second  << std::endl;


            std::cout << "Distance: " << calcDistance(myPose, gpr10Odometry[loop.first]) << " | " << i << std::endl;
        }
    }

    return(0);
}


int main(void){
    // std::cout << gpr10Path + gpr15Path << "\n";
    loadGPR10();
    evaluateGPR10();

    // for(const auto &point : gpr10All[0].first){
    //     std::cout << "    " << point.x
    //                 << " "    << point.y
    //                 << " "    << point.z << std::endl;
    //     break;
    // }

    // for(const auto &point : gpr10All[0].second){
    //     std::cout << point << " ";
    //     std::cout << std::endl;
    //     break;
    // }

    std::cout << gpr10Odometry[877][0] << ", " <<  gpr10Odometry[877][1] << ", " gpr10Odometry[877][2] << "\n";
    std::cout << gpr10Odometry[535][0] << ", " <<  gpr10Odometry[535][1] << ", " gpr10Odometry[535][2] << "\n";
    std::cout << calcDistance(gpr10Odometry[535], gpr10Odometry[535]) << "\n";

}