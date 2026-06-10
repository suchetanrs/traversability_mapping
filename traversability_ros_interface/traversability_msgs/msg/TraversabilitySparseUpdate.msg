# Sparse / patch update of a traversability grid_map for navigation.
#
# Cells are identified by their ABSOLUTE lattice id (resize-stable), not by a
# grid_map buffer index. The receiver reconstructs each cell centre as
#   x = origin_x + ci * resolution,  y = origin_y + cj * resolution
# where (ci, cj) come from un-packing cell_keys (high 32 bits = ci, low = cj).
#
# values is row-major per cell: [cell0_layer0 .. cell0_layerN-1, cell1_layer0 ..]
# so values.size() == cell_keys.size() * layers.size().
# A NaN value means "unknown / cleared" for that cell+layer.

std_msgs/Header header        # header.frame_id == map frame
float64 resolution            # lattice resolution (m)
float64 origin_x              # lattice origin x (m)
float64 origin_y              # lattice origin y (m)
string[] layers              # layer names, defines the per-cell value ordering
bool is_full_snapshot        # true: receiver should reset before applying
uint64[] cell_keys           # absolute lattice keys (Lattice::key(ci, cj))
float32[] values             # cell_keys.size() * layers.size(), row-major per cell
