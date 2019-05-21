package astar;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;

public class PathFinder {

    public final static int BARRIER = 1; // inaccessible location

    private Location start;	   // start location
    private Location goal;        // goal location
    private RectangularMap map;   // the map

    private List<Location> closeList = new ArrayList<Location>();
    private Queue<Location> openList = new PriorityQueue<Location>();

    public PathFinder(RectangularMap map, Location start, Location goal) {
        this.map = map;
        this.start = start;
        this.goal = goal;
    }

    public RectangularMap getMap() {
        return map;
    }

    public Location getStart() {
        return start;
    }

    public Location getGoal() {
        return goal;
    }

    public Path findPath() {

        if(isBarrier(start.getRow(), start.getColumn()) || isBarrier(goal.getRow(), goal.getColumn())) return null;

        // initialize variables
        closeList.clear();
        openList.clear();
        openList.add(start); // search path from start location

        while (openList.size() != 0)
        {
            // yes, we find the path
            if(isLocationInCloseList(goal)) break;

            // poll out the top of openList to closeList
            // goes through its neighours and add valid ones to openList
            Location current = openList.poll();
            closeList.add(current);
            addNeighborLocationToOpenList(current);
        }

        // construct the path
        return retrievePath();
    }

    private boolean isLocationInCloseList(Location location){
        return location != null && isLocationInCloseList(location.getRow(), location.getColumn());
    }

    private boolean isLocationInCloseList(int row, int column){
        if(closeList.size() == 0) return false;
        for(Location location : closeList){
            if(location.getRow() == row && location.getColumn() == column) return true;
        }
        return false;
    }

    private void addNeighborLocationToOpenList(Location current){
        int row = current.getRow();
        int column = current.getColumn();

        // agent can only move in four directions
        addNeighborLocationToOpenList(current, row - 1, column);
        addNeighborLocationToOpenList(current, row + 1, column);
        addNeighborLocationToOpenList(current, row, column - 1);
        addNeighborLocationToOpenList(current, row, column + 1);
    }

    private void addNeighborLocationToOpenList(Location current, int row, int column){
        if(canAddToOpenList(row, column)){
            int G = current.G + 1;
            Location neighbour = findLocationInOpenList(row, column);
            if(neighbour == null){ // neighour is not in openList
                int H = calculateH(row, column);

                // we found the goal location
                if(isGoalLocation(row, column)){
                    neighbour = goal;
                    neighbour.parent = current;
                    neighbour.G = G;
                    neighbour.H = H;
                }
                else{
                    // its a total new neighour
                    neighbour = new Location(row, column, G, H);
                    neighbour.parent = current;
                }
                openList.add(neighbour);
            }
            else if(neighbour.G > G){
                // neighour in openList and new route is better than previous router
                // so we update neighour's parent as current location
                neighbour.G = G;
                neighbour.parent = current;
                openList.add(neighbour);
            }
        }
    }

    /**
     *  check if a location can be added to openList
     */
    private boolean canAddToOpenList(int row, int column){
        if(row < 0 || column < 0 || row >= map.getRows() || column >= map.getColumns()) return false;
        if(map.getValueAt(row, column) == BARRIER) return false;
        if(isLocationInCloseList(row, column)) return false;
        return true;
    }

    /**
     *  check if a location is in openList, if so return it
     */
    private Location findLocationInOpenList(int row, int column){
        if (openList.size() == 0) return null;
        for (Location location : openList)
        {
            if(location.getRow() == row && location.getColumn() == column) return location;
        }
        return null;
    }

    /**
     *  calculate H value from current row/column to the goal location
     *  it is a heuristic function, because we ignore barriers,
     *  and just calculate math distance of two locations
     */
    private int calculateH(int row, int column){
        return Math.abs(goal.getRow() - row) + Math.abs(goal.getColumn() - column);
    }

    /**
     *  check if the location is goal location
     */
    private boolean isGoalLocation(int row, int column){
        return goal.getRow() == row && goal.getColumn() == column;
    }

    /**
     *  construct path by reverse parent location from goal location
     */
    private Path retrievePath(){
        if(goal == null || goal.parent == null) return null;
        ArrayList<Location> reversed_path = new ArrayList<Location>();
        while(goal != null){
            reversed_path.add(goal);
            goal = goal.parent;
        }

        Path path = new Path(reversed_path.get(reversed_path.size() - 1));
        for(int i = reversed_path.size() - 2; i >= 0; i--){
            path.moveTo(reversed_path.get(i));
        }
        return path;
    }

    private boolean isBarrier(int row, int column){
        return map.getValueAt(row, column) == BARRIER;
    }
}
