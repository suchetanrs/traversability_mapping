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

    Eigen::Vector4d traversabilityGrid::get_goodness_m(float xp, float yp, const double distance, const double ground_clearance, const double max_pitch)
    {
        float xpo = 0;
        float ypo = 0;
        meter2indOpt(xp, yp, xpo, ypo);
        return get_goodness(xpo, ypo, distance, ground_clearance, max_pitch);
    }

    Eigen::Vector4d traversabilityGrid::get_goodness(float index_x, float index_y, const double distance, const double ground_clearance, const double max_pitch)
    {
        double delta = distance / _resolution;
        // TODO: std::max instead of std::min?
        uint delta_ind = std::min(1.0, std::ceil(delta));

        // Point goodness cannot be calculated (border of the area) or too close to robot
        if (index_x < delta_ind || index_x > (size_x_ - delta_ind) ||
            index_y < delta_ind || index_y > (size_y_ - delta_ind))
        {
            return Eigen::Vector4d(-1.0, 0.0, 0.0, 0.0);
        }

        // Init params
        uint border_hazard = 0;
        float zM = -100., zm = 100.;

        uint nb_min = 0.25 * (2 * delta_ind + 1) * (2 * delta_ind + 1);

        // Get all points to be considered for the current cell and get min/max altitude
        std::vector<Eigen::Vector3d> P3Ds; // stores all centers of neighbouring cells.
        Eigen::Vector3d cellsSum = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cellsSumSquared = Eigen::Matrix3d::Zero(); // stores all the covariances of the neighbouring cells.
        double cellsNumPoints = 0;
        for (int i = std::max(0, int(index_x - delta_ind)); i < std::min(int(index_x + delta_ind), size_x_); i++)
        {
            for (int j = std::max(0, int(index_y - delta_ind)); j < std::min(int(index_y + delta_ind), size_y_); j++)
            {
                const auto& cell = _grid.at(i).at(j);
                if (cell.N < nb_min)
                {
                    border_hazard++;
                    return Eigen::Vector4d(-1.0, 0.0, 0.0, 0.0);
                }
                else
                {
                    if(_computeRoughness)
                    {
                        Eigen::Vector3d center = Eigen::Vector3d(cell.sx, cell.sy, cell.sz) / cell.N;
                        P3Ds.push_back(center);
                    }

                    Eigen::Matrix3d sumSquared;
                    sumSquared << cell.sx2, cell.sxy, cell.sxz,
                        cell.sxy, cell.sy2, cell.syz,
                        cell.sxz, cell.syz, cell.sz2;

                    Eigen::Vector3d sum(cell.sx, cell.sy, cell.sz);

                    cellsSum += sum;
                    cellsSumSquared.noalias() += sumSquared;
                    cellsNumPoints += cell.N;
                    if(abs(i - index_x) < _resolution * 2 && abs(j - index_y) < _resolution * 2)
                    {
                        zM = std::max(zM, cell.z_max);
                        zm = std::min(zm, cell.z_min);
                    }
                }
            }
        }

        if (border_hazard > 0)
        {
            Eigen::Vector4d X;
            X << -1., 0., 0., 0.;
            return X;
        }

        // Check if delta z is bigger than the robot ground clearance (how to deals with grass?)
        double step_hazard = ((zM - zm) / ground_clearance);
        if (step_hazard > 1)
            step_hazard = 1;

        // Process surface normal

        /*
            // Method n°1 : classical LMS with cells barycenters
            // AX=B
            // Eigen::Vector3f X = A.colPivHouseholderQr().solve(B);
            Eigen::MatrixXd P3Ds_M(3, P3Ds.size());
            for(uint i=0; i < P3Ds.size(); ++i)
                P3Ds_M.block<3,1>(0,i) = P3Ds.at(i);
            Eigen::VectorXd ONE = Eigen::VectorXd::Ones(P3Ds.size()) ;
            Eigen::Vector3d planeLMS = P3Ds_M.transpose().colPivHouseholderQr().solve(ONE);
            planeLMS.normalize();

            double roughness = (Eigen::VectorXd::Ones(P3Ds.size())-P3Ds_M.transpose()*planeLMS).norm()/P3Ds.size();
        */

        // Method n°2 : using covariance ponderation (trace of P3D covariance as weight)
        Eigen::Vector3d mean = cellsSum / cellsNumPoints;
        Eigen::Matrix3d A = cellsSumSquared / cellsNumPoints - mean * mean.transpose();

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(A); // eigenvalue
        Eigen::Vector3d planeLMS = es.eigenvectors().col(0);  // eigenvector having the direction of surface normal

        if (planeLMS.z() < 0)
            planeLMS = -planeLMS;

        // std::cout << planeLMS.transpose() << std::endl;

        double roughness_hazard;
        if(_computeRoughness)
        {
            double roughness = 0.0;
            double d = mean.transpose() * planeLMS;
            for (uint i = 0; i < P3Ds.size(); ++i)
            {
                roughness += double((P3Ds.at(i).transpose() * planeLMS) - d) * double((P3Ds.at(i).transpose() * planeLMS) - d);
            }
            roughness = std::sqrt(roughness) / P3Ds.size();
            roughness_hazard = 3. * roughness / ground_clearance;
            if (roughness_hazard > 1)
                roughness_hazard = 1;
        }

        // Check if ground slope is bigger than the robot admissive max slope
        double pitch = std::abs(std::acos(planeLMS.dot(Eigen::Vector3d(0., 0., 1.))));
        // std::cout << "pitch : " << pitch << ", " << max_pitch << ", " << planeLMS.norm() << std::endl;
        // std::cout << "Z max and min: " << zM << ", " << zm;
        double pitch_hazard = pitch / max_pitch;
        if (pitch_hazard > 1)
            pitch_hazard = 1;

        // std::cout << "pitch_hazard : " << pitch_hazard << " roughness_hazard : " << roughness_hazard << " step_hazard : " << step_hazard << std::endl;

        Eigen::Vector4d haz;
        haz(0) = std::max(std::max(step_hazard, (0.0 * roughness_hazard)), pitch_hazard);
        haz(1) = step_hazard;
        // haz(2) = roughness_hazard;
        haz(3) = pitch_hazard;
        // haz(4) = border_hazard;
        // elevation
        // haz(5) = _grid.at(index_x).at(index_y).sz / _grid.at(index_x).at(index_y).N;
        // making haz(2) as elevation for now. Was roughness before.
        haz(2) = _grid.at(index_x).at(index_y).sz / _grid.at(index_x).at(index_y).N;

        return haz;
    }

    // void traversabilityGrid::get_traversability(std::vector<Eigen::Matrix<double, 6, 1>> &cells, const double distance, const double ground_clearance, const double max_pitch)
    // {
    //     for (int i = 0; i < size_x_; ++i)
    //     {
    //         for (int j = 0; j < size_y_; ++j)
    //         {

    //             Eigen::Vector4d haz = get_goodness(Eigen::Vector2d(i, j), distance, ground_clearance, max_pitch);
    //             Eigen::Vector2d center = ind2meter(Eigen::Vector2d(i, j));

    //             Eigen::Matrix<double, 6, 1> node;
    //             node << center.x(), center.y(), _halfside.x(), _halfside.y(), haz(0), 0.;
    //             cells.push_back(node);
    //         }
    //     }
    // }
}
