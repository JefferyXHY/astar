package astar;

public interface RectangularMap {

	int getRows();

	int getColumns();

	int getValueAt(int row, int col);

	int getValueAt(Location loc);

  void setLocationAt(Location loc, int value);

	Iterable<Location> getNeighbours(Location loc);
}
