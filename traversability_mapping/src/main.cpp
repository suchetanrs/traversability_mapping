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
#include "traversability_mapping/System.hpp"

class MainClass {
public:
    MainClass(traversability_mapping::System* systemPtr) : systemPtr_(systemPtr) {
        // Add new keyframe from the class constructor
        const double ts1 = 100.0;
        long unsigned int mnId = 0;
        systemPtr_->addNewLocalMap(1);
        systemPtr_->addNewKeyFrameTsDouble(ts1, mnId, 1);
        // systemPtr_->addNewKeyFrame(323.45, 2);
        // systemPtr_->addNewKeyFrame(523.45, 3);
    }

    ~MainClass() {
        // Clean up
        // delete systemPtr_;
    }

private:
    traversability_mapping::System* systemPtr_;
};

int main() {
#ifdef WITH_TRAVERSABILITY_MAP
    std::cout << "With trav" << std::endl;
#else
    std::cout << "Without trav" << std::endl;
#endif

    // Create an instance of System using new
    traversability_mapping::System* systemPtr = new traversability_mapping::System();

    // Create an instance of MainClass and pass the system pointer
    MainClass mainObj(systemPtr);

    // You can continue using systemPtr here if needed

    return 0;
}
