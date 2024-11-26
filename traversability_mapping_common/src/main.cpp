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

#include <traversability_mapping_common/type_conversion.hpp>

int main()
{
    std::shared_ptr<traversability_mapping::TraversabilityTypeConversions> typeConversion_;
    typeConversion_ = std::make_shared<traversability_mapping::TraversabilityTypeConversions>();
}