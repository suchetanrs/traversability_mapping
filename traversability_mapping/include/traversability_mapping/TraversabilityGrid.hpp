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

    void insert(const float xp, const float yp, const float zp){
        N++;
        // deal with min max
        // TODO: Check logic of min and max.
        z_min = std::min(zp, z_min);
        z_max = std::max(zp, z_max);

        // update momentums
        sx += xp;
        sy += yp;
        sz += zp;
        sx2 += xp*xp;
        sy2 += yp*yp;
        sz2 += zp*zp;
        sxy += xp*yp;
        sxz += xp*zp;
        syz += yp*zp;
    }

    unsigned int N=0;
    float z_min=0.;
    float z_max=0.;
    float sx = 0.;
    float sy = 0.;
    float sz = 0.;
    float sx2 = 0.;
    float sy2 = 0.;
    float sz2 =0.;
    float sxy =0.;
    float sxz =0.;
    float syz = 0.;
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

        offsetX = _gridOffset.x();
        offsetY = _gridOffset.y();
        halfsideX = _halfside.x();
        halfsideY = _halfside.y();
        // _computeRoughness = parameterInstance.getValue<bool>("compute_roughness");
        _computeRoughness = false;
    }

    // @brief responsible for inserting a 3D point into the appropriate cell within the grid.
    // The appropriate cell here is that all points with the same x and y coordinates as the node in the grid map are grouped regardless of the z direction.
    void insert_data(float xp, float yp, float zp);

    // @brief resets metadata for all cells in the grid.
    void reset(){ 
        for(int i=0; i < size_x_; ++i)
            for(int j=0; j < size_y_; ++j)
                _grid.at(i).at(j).reset();
    }

    // @brief takes vector2d coordinates for which goodness needs to be calculated. These coordinates can be in meters.
    Eigen::Vector4d get_goodness_m(float xp, float yp, const double distance, const double ground_clearance, const double max_pitch);
    
    // @brief takes vector2d coordinates for which goodness needs to be calculated. These coordinates must be in indices (row and column of the traversability grid map).
    Eigen::Vector4d get_goodness(float index_x, float index_y, const double distance, const double ground_clearance, const double max_pitch);
    
    // @brief
    // void get_traversability(std::vector<Eigen::Matrix<double,6,1>> &cells, const double distance, const double ground_clearance, const double max_pitch);

    uint getNbCells(){ return size_x_*size_y_;}

    Eigen::Vector2d meter2ind(Eigen::Vector2d meter){
        Eigen::Vector2d idx =  (meter-_gridOffset+_halfside)/_resolution;
        return Eigen::Vector2d(std::min(static_cast<int>(std::round(idx.x())), size_x_ - 1) , std::min(static_cast<int>(std::round(idx.y())), size_y_ - 1));
    }

    void meter2indOpt(float xp, float yp, float& xpo, float& ypo){
        auto idx = (xp-offsetX+halfsideX)/_resolution;
        auto idy = (yp-offsetY+halfsideY)/_resolution;
        xpo = std::min(static_cast<int>(std::round(idx)), size_x_ - 1);
        ypo = std::min(static_cast<int>(std::round(idy)), size_y_ - 1);
    }

    Eigen::Vector2d ind2meter(Eigen::Vector2d ind){
        return (ind*_resolution -_halfside +_gridOffset);
    }

    void ind2meterOpt(float indx, float indy, float& mx, float& my){
        mx = indx*_resolution -halfsideX +offsetX;
        my = indy*_resolution -halfsideY +offsetY;
    }

    std::vector<std::vector<NodeMetaData>>& getGrid() {
        return _grid;
    }

private :
    double _resolution;                           /// cell size
    bool _computeRoughness;
    Eigen::Vector2d _center;                      /// Center
    Eigen::Vector2d _halfside;                    /// Half-size of the cube
    Eigen::Vector2d _gridOffset;
    float offsetX;
    float offsetY;
    float halfsideX;
    float halfsideY;

    NodeMetaData _meta_data;                      /// Meta-data container

    int size_x_;
    int size_y_;

    std::vector<std::vector<NodeMetaData>> _grid;
};
}

#endif
