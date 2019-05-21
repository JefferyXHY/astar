package astar;

import java.util.HashMap;
import java.util.Map;

public class AbstractDistance {

    public static final int INFINITY = Integer.MAX_VALUE;

    RectangularMap map;
    Location agentGoal, agentInitialLoc;
    public Map<Location, Integer> gScores = new HashMap<Location, Integer>();

    public Path path;

    public AbstractDistance(RectangularMap map, Location agentGoal, Location agentInitialLoc) {
        this.map = map;
        this.agentGoal = agentGoal;
        this.agentInitialLoc = agentInitialLoc;
    }

    public int distance(Location loc) {

        if(map.getValueAt(loc) == 1 || map.getValueAt(agentGoal) == 1) return INFINITY;
        Integer gScore = this.gScores.get(loc);
        if(gScore != null) return gScore;

        Location start = agentGoal;
        Location goal = new Location(loc.getRow(), loc.getColumn());
        if(start.equals(goal)){
            this.path = new Path(start);
            return 0;
        }

        PathFinder pathFinder = new PathFinder(map, start, goal);
        Path newPath = pathFinder.findPath();
        if(newPath != null){
            for(Location location : newPath.getLocations()){
                this.gScores.put(location, location.G);
            }
            this.path = newPath;
            return newPath.getCost();
        } else {
            return INFINITY;
        }
    }
}
