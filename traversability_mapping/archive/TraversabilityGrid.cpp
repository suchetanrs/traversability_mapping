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
#include "traversability_mapping/TraversabilityGrid.hpp"
#include <Eigen/Eigenvalues>
namespace traversability_mapping
{
    void traversabilityGrid::insert_data(float xp, float yp, float zp)
    {

        // Are we at maximal subdivision lvl?
        float xpo = 0;
        float ypo = 0;
        meter2indOpt(xp, yp, xpo, ypo);

        if (xpo >= size_x_ || ypo >= size_y_ ||
            xpo < 0 || ypo < 0)
            return;
        _grid.at(xpo).at(ypo).insert(xp, yp, zp);
    }

    std::array<double, 6> traversabilityGrid::get_goodness_v2(float index_x, float index_y, const double distance, const double ground_clearance, const double max_pitch)
    {
        std::array<double, 6> haz;
        haz.fill(std::numeric_limits<double>::quiet_NaN());

        int delta_ind = std::ceil((distance / 2.0) / _resolution);
        uint border_hazard = 0;
        auto cell = _grid.at(index_x).at(index_y);

        if (
            index_x <= delta_ind || 
            index_x >= (size_x_ - delta_ind) || 
            index_y <= delta_ind || 
            index_y >= (size_y_ - delta_ind)
            ) 
            {
            border_hazard++;
            haz[1] = border_hazard;
            return haz;
        }

        // -----------------------------------------------------------------------------------------------------------------------------
        // Get the complete meta data for the zone and get all barycenters
        std::vector<Eigen::Vector3d> barycenters;
        NodeMetaData data_viscinity;
        uint nb_min_vicinity = 15; // min points in vicinity
        for (int i = std::max(0, int(index_x - delta_ind)); i < std::min(int(index_x + delta_ind), size_x_); i++) {
            for (int j = std::max(0, int(index_y - delta_ind)); j < std::min(int(index_y + delta_ind), size_y_); j++) {
                NodeMetaData data = _grid.at(i).at(j);
                if (data.N == 0) {
                    border_hazard++;
                    continue;
                }                       
                data_viscinity.fuseWith(data);
                barycenters.push_back(Eigen::Vector3d(data.sx, data.sy, data.sz)/(double)data.N);
            }
        }
        if (data_viscinity.N < nb_min_vicinity || cell.N < 1) {
            border_hazard+= data_viscinity.N;
            haz[1] = border_hazard;
            return haz;
        }

        // -----------------------------------------------------------------------------------------------------------------------------
        // BORDER HAZARD :
        if (border_hazard > 0) {
            haz[1] = border_hazard;
            return haz;
        }

        // -----------------------------------------------------------------------------------------------------------------------------
        // Normal of the given index.
        Eigen::Vector3d N;
        Eigen::Vector3d vicinity_centroid = Eigen::Vector3d(data_viscinity.sx, data_viscinity.sy, data_viscinity.sz) / data_viscinity.N;
        
        if (use_least_squares_fit_to_compute_normals) {
            Eigen::Vector3d cell_barycenter = Eigen::Vector3d(cell.sx, cell.sy, cell.sz)/(double)cell.N;
            // Find the normal : F X = Y  ==> N = (-X(1:2),1)
            Eigen::MatrixXd F = Eigen::MatrixXd(3, 3);
            F(0, 0) = data_viscinity.sx2;
            F(0, 1) = data_viscinity.sxy;
            F(0, 2) = data_viscinity.sx;
            F(1, 0) = data_viscinity.sxy;
            F(1, 1) = data_viscinity.sy2;
            F(1, 2) = data_viscinity.sy;
            F(2, 0) = data_viscinity.sx;
            F(2, 1) = data_viscinity.sy;
            F(2, 2) = data_viscinity.N;
            Eigen::Vector3d Y(3);
            Y(0) = data_viscinity.sxz;
            Y(1) = data_viscinity.syz;
            Y(2) = data_viscinity.sz;
            
            Eigen::Vector3d X = F.inverse() * Y;
            Eigen::Vector3d N_unormalized(-X(0), -X(1), 1.0);
            N = N_unormalized.normalized();
            
            // 1. Build the quaternion: rotate (0,0,1) → N
            Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
            double dot = z_axis.dot(N);
            
            Eigen::Quaterniond q_eig = 
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), N);
            _normals.push_back({cell_barycenter, q_eig});
        }
        else if (use_pca_to_compute_normals) {
            Eigen::Vector3d cell_barycenter = Eigen::Vector3d(cell.sx, cell.sy, cell.sz)/(double)cell.N;
            Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
            for (const auto& barycenter : barycenters)
            {
                Eigen::Vector3d d = barycenter - vicinity_centroid;
                C.noalias() += d * d.transpose();
            }
            C /= barycenters.size();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
            Eigen::Vector3d normal = es.eigenvectors().col(0);  // eigenvector of min eigenvalue

            // 4) Ensure the normal points upward (positive Z)
            if (normal.z() < 0) {
                normal = -normal;
            }
            N = normal.normalized();

            // 1. Build the quaternion: rotate (0,0,1) → N
            Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
            double dot = z_axis.dot(N);
            
            Eigen::Quaterniond q_eig = 
            Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), N);
            _normals.push_back({cell_barycenter, q_eig});
        }

        // // Find border hazard if cells are too line like using PCA
        // Eigen::Matrix2d C = Eigen::Matrix2d::Zero();
        // for (const auto& bc : barycenters)                      // you already built this vector
        // {
        //     Eigen::Vector2d d = bc.head<2>() - vicinity_centroid.head<2>();
        //     C.noalias() += d * d.transpose();
        // }
        // C /= barycenters.size();

        // Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(C);
        // double lambda_max = es2.eigenvalues()(1);
        // double lambda_min = es2.eigenvalues()(0);

        // constexpr double min_shape_ratio = 0.50;   // 0 ≤ ratio ≤ 1
        // double shape_ratio = std::sqrt(lambda_min / lambda_max);

        // if (shape_ratio < min_shape_ratio)
        // {
        //     haz[1] = 1;
        //     return haz;                 // neighbourhood too 1-D
        // }

        // -----------------------------------------------------------------------------------------------------------------------------
        // ELEVATION:
        haz[2] = cell.sz / cell.N;

        // -----------------------------------------------------------------------------------------------------------------------------
        // SLOPE HAZARD :
        double angle_with_z_axis = std::abs(std::acos(N.dot(Eigen::Vector3d(0., 0., 1.))));
        haz[3] = angle_with_z_axis / max_pitch;
        haz[3] = std::min(haz[3], 1.0);

        // -----------------------------------------------------------------------------------------------------------------------------
        // STEP HAZARD :
        // find distance of max - min from the plane of the normal.

        double min_d =  std::numeric_limits<double>::infinity();
        double max_d = -std::numeric_limits<double>::infinity();
        double sum_distances_sq = 0;
        for (auto& p : barycenters) {
            double d = (p - vicinity_centroid).dot(N);
            min_d = std::min(min_d, d);
            max_d = std::max(max_d, d);
            sum_distances_sq += d * d;
        }
        double h_step = max_d - min_d;
        haz[4] = h_step / ground_clearance;
        haz[4] = std::min(haz[4], 1.0);

        // -----------------------------------------------------------------------------------------------------------------------------
        // ROUGHNESS HAZARD :
        
        double roughness = std::sqrt(sum_distances_sq / barycenters.size());
        haz[5] = roughness / ground_clearance;
        haz[5] = std::min(haz[5], 1.0);

        haz[0] = std::max(haz[3], std::max(haz[4], haz[5])); // max of slope, step, roughness

        return haz;
    }

    void traversabilityGrid::saveNormals(const std::string& filename) const
    {
        std::ofstream ofs(filename);
        if (!ofs)
            throw std::runtime_error("Cannot open " + filename);

        // optional: header line
        ofs << "# x y z qx qy qz qw\n";
        ofs << std::fixed << std::setprecision(6);   // 6 decimals is usually enough

        for (const auto& n : _normals)
        {
            ofs << n.center.x() << ' '
                << n.center.y() << ' '
                << n.center.z() << ' '
                << n.quaternion.x() << ' '
                << n.quaternion.y() << ' '
                << n.quaternion.z() << ' '
                << n.quaternion.w() << '\n';
        }
    }
}
