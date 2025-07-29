/* Traversability Mapping - A global and local traversability mapping algorithm.
 * Copyright (C) 2024 Suchetan Saravanan and Damien Vivet
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.  If not, see
 * <https://www.gnu.org/licenses/>.
 */

#ifndef TRAVERSABILITY_TRAVERSABILITYGRID_HPP_
#define TRAVERSABILITY_TRAVERSABILITYGRID_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>

#include <traversability_mapping/Parameters.hpp>
namespace traversability_mapping {
class NodeMetaData{

public:
    NodeMetaData(){
        sx = 0; sy = 0; sz = 0;  sx2 = 0; sy2 = 0; sz2 =0; sxy =0; sxz =0; syz = 0;
        N = 0;
    }

    void reset(){
        sx = 0; sy = 0; sz = 0;  sx2 = 0; sy2 = 0; sz2 =0; sxy =0; sxz =0; syz = 0;
        N = 0;
    }

    void insert(const float xp, const float yp, const float zp){
        N++;

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

    void fuseWith(const NodeMetaData& data){
        N     += data.N;
        
        sx    += data.sx;
        sy    += data.sy;
        sz    += data.sz;
        sx2   += data.sx2;
        sy2   += data.sy2;
        sz2   += data.sz2;
        sxy   += data.sxy;
        sxz   += data.sxz;
        syz   += data.syz;
    }

    unsigned int N=0;
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

struct Normals
{
    Eigen::Vector3d center;
    Eigen::Quaterniond quaternion;
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

        use_pca_to_compute_normals = parameterInstance.getValue<bool>("use_pca_to_compute_normals");
        use_least_squares_fit_to_compute_normals = parameterInstance.getValue<bool>("use_least_squares_fit_to_compute_normals");
        if (use_pca_to_compute_normals && use_least_squares_fit_to_compute_normals)
        {
            throw std::runtime_error("Both PCA and least squares fit are enabled. Please disable one of them.");
        }
        if (!use_pca_to_compute_normals && !use_least_squares_fit_to_compute_normals)
        {
            throw std::runtime_error("Both PCA and least squares fit are disabled. Please enable one of them.");
        }
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
    
    // overall hazard, border hazard (sparsity), elevation, slope hazard, step hazard, roughness hazard
    std::array<double, 9> get_goodness_v2(float index_x, float index_y, const double distance, const double ground_clearance, const double max_pitch);

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
    
    void saveNormals(const std::string& filename) const;

    std::vector<Normals> _normals;
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
    bool use_pca_to_compute_normals = false;
    bool use_least_squares_fit_to_compute_normals = true;
};
}

#endif
