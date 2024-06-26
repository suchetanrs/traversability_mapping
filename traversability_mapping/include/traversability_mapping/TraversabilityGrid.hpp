//
// Created by d.vivet on 19/04/23.
//

#ifndef TRAVERSABILITY_TRAVERSABILITYGRID_HPP_
#define TRAVERSABILITY_TRAVERSABILITYGRID_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <iostream>
#include <vector>
namespace traversability_mapping {
class NodeMetaData{

public:
    NodeMetaData(){
        z_min = 100.;
        z_max = -100.;
        sx = 0; sy = 0; sz = 0;  sx2 = 0; sy2 = 0; sz2 =0; sxy =0; sxz =0; syz = 0;
        N = 0;
    }

    void reset(){
        z_min = 100.;
        z_max = -100.;
        sx = 0; sy = 0; sz = 0;  sx2 = 0; sy2 = 0; sz2 =0; sxy =0; sxz =0; syz = 0;
        N = 0;
    }

    void insert(Eigen::Vector3d &p3d){
        N++;
        // deal with min max
        // TODO: Check logic of min and max.
        z_min = std::min(p3d.z(), z_min);
        z_max = std::max(p3d.z(), z_max);

        // update momentums
        sx += p3d.x();
        sy += p3d.y();
        sz += p3d.z();
        sx2 += p3d.x()*p3d.x();
        sy2 += p3d.y()*p3d.y();
        sz2 += p3d.z()*p3d.z();
        sxy += p3d.x()*p3d.y();
        sxz += p3d.x()*p3d.z();
        syz += p3d.y()*p3d.z();
    }

    unsigned int N=0;
    double z_min=0.;
    double z_max=0.;
    double sx = 0.;
    double sy = 0.;
    double sz = 0.;
    double sx2 = 0.;
    double sy2 = 0.;
    double sz2 =0.;
    double sxy =0.;
    double sxz =0.;
    double syz = 0.;
};



class traversabilityGrid {
public:
    traversabilityGrid(double resolution, Eigen::Vector2d ahalfside, Eigen::Vector2d offSet)
            : _resolution(resolution), _halfside(ahalfside), _gridOffset(offSet) {

        // resize grid
        NodeMetaData default_value;
        double Nd_x = _halfside.x()/_resolution;
        double Nd_y = _halfside.y()/_resolution;
        size_x_ = 2.*(std::ceil(Nd_x))+1;
        size_y_ = 2.*(std::ceil(Nd_y))+1;
        _grid.resize(size_x_, std::vector<NodeMetaData>(size_y_, default_value));
        reset();
    }

    // @brief responsible for inserting a 3D point into the appropriate cell within the grid.
    // The appropriate cell here is that all points with the same x and y coordinates as the node in the grid map are grouped regardless of the z direction.
    void insert_data(Eigen::Vector3d &p3d);

    // @brief resets metadata for all cells in the grid.
    void reset(){ 
        for(int i=0; i < size_x_; ++i)
            for(int j=0; j < size_y_; ++j)
                _grid.at(i).at(j).reset();
    }

    // @brief takes vector2d coordinates for which goodness needs to be calculated. These coordinates can be in meters.
    Eigen::Vector4d get_goodness_m(Eigen::Vector2d meters, const double distance, const double ground_clearance, const double max_pitch);
    
    // @brief takes vector2d coordinates for which goodness needs to be calculated. These coordinates must be in indices (row and column of the traversability grid map).
    Eigen::Vector4d get_goodness(Eigen::Vector2d ind, const double distance, const double ground_clearance, const double max_pitch);
    
    // @brief
    // void get_traversability(std::vector<Eigen::Matrix<double,6,1>> &cells, const double distance, const double ground_clearance, const double max_pitch);

    uint getNbCells(){ return size_x_*size_y_;}

    Eigen::Vector2d meter2ind(Eigen::Vector2d meter){
        Eigen::Vector2d idx =  (meter-_gridOffset+_halfside)/_resolution;
        return Eigen::Vector2d(std::min(static_cast<int>(std::round(idx.x())), size_x_ - 1) , std::min(static_cast<int>(std::round(idx.y())), size_y_ - 1));
    }

    Eigen::Vector2d ind2meter(Eigen::Vector2d ind){
        return (ind*_resolution -_halfside +_gridOffset);
    }

    std::vector<std::vector<NodeMetaData>>& getGrid() {
        return _grid;
    }

private :
    double _resolution;                           /// cell size
    Eigen::Vector2d _center;                      /// Center
    Eigen::Vector2d _halfside;                    /// Half-size of the cube
    Eigen::Vector2d _gridOffset;

    NodeMetaData _meta_data;                      /// Meta-data container

    int size_x_;
    int size_y_;

    std::vector<std::vector<NodeMetaData>> _grid;
};
}

#endif
