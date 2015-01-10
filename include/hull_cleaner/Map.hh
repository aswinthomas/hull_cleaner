
#ifndef _MAP_HH
#define _MAP_HH

#include <hull_cleaner/BoustrophedonPlanner.hh>

//* Robot
/**
 * \brief This class stores a vector map
 *
 */
class Map {
private:
  struct cell {
    geometry_msgs::Point pos;
    bool filled;
  };
  std::vector< std::vector<cell> > grid;

public:
  friend class BoustrophedonPlanner;
  /*!
   * \brief The constructor
   *
   * This method initializes all variables
   *
   */
  Map(int rows,int cols,double cellSize) {
    cell newcell;
    std::vector<cell> grid_column;

    for(int i=0; i<rows; i++) {
      grid_column.clear();
      for(int j=0; j<cols; j++) {
        newcell.filled=false;
        newcell.pos.x = cellSize*(i+1);
        newcell.pos.y = cellSize*(j+1);
        grid_column.push_back(newcell);
      }
      grid.push_back(grid_column);
    }
  }
};

#endif
