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
#ifndef TRAVERSABILITY_HELPERS_HPP_
#define TRAVERSABILITY_HELPERS_HPP_

#include <chrono>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// NOTE: the ROS occupancy-grid conversion (gridMapToOccupancyGrid) lives in the ROS
// adapter, not here, so the core library carries zero ROS headers.

namespace traversability_mapping
{
    inline double probabilityToLogOdds(double p) {
        // Clamp away from {0, 1} so log-odds stays finite
        // extreme observation pins the cell at +/- inf forever.
        constexpr double eps = 1e-2;
        p = std::min(std::max(p, eps), 1.0 - eps);
        return std::log(p / (1.0 - p));
    }

    inline double logOddsToProbability(double l) {
        return 1.0 / (1.0 + std::exp(-l));
    }

    // Function to update the occupancy of a cell
    inline double updateCellLogOdds(float& log_odds, double new_observation) {
        // If the cell is uninitialized (NaN), initialize with neutral log-odds
        if (std::isnan(log_odds)) {
            log_odds = 0.0;  // Neutral belief (50%)
        }

        double new_log_odds = probabilityToLogOdds(new_observation);

        log_odds += new_log_odds;

        return logOddsToProbability(log_odds);
    }

    class Profiler
    {
    public:
        Profiler(const std::string & functionName) : functionName(functionName)
        {
            start = std::chrono::high_resolution_clock::now();
        }

        ~Profiler()
        {
            auto end      = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            float seconds = duration.count() / 1e6;
            std::cout << "\033[1;94m" << functionName << " Execution Time: " << seconds << " Seconds\033[0m" << std::endl;
        }

    private:
        std::string functionName;
        std::chrono::time_point<std::chrono::high_resolution_clock> start;
    };

    #define PROFILE_FUNCTION Profiler profiler_instance(__func__);
}
#endif // TRAVERSABILITY_HELPERS_HPP_