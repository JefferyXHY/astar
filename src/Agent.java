package astar;

import java.util.HashMap;
import java.util.List;
import java.util.ArrayList;
import java.util.Map;

public class Agent {
    public final static int BARRIER = 1;

    private Location start, goal;
    private int priority;
    private Location current;
    public Map<Location, Location> cameFrom = new HashMap<Location, Location>();
    public AbstractDistance abstractDistance;


    public Agent(int priority, Location start, Location goal) {
        this.priority = priority;
        this.start = start;
        this.goal = goal;
        this.current = start;
    }

    public int getPriority() {
        return priority;
    }

    public Location getStart() {
        return start;
    }

    public Location getGoal() {
        return goal;
    }

	public String getName() {
		return "Agent " + priority;
	}

    public Location getCurrent(){
        return current;
    }

    public void setCurrent(Location current){
        this.current = current;
    }

    public Path getPath(){
    	return abstractDistance.path;
    }

    public boolean reachGoal(){
        return current.equals(goal);
    }

    public Location nextStepOf(Location location){
        for(Location loc : getPath().getLocations()){
            if(loc.equals(location)) return loc.parent;
        }
        if(abstractDistance.distance(location) != Integer.MAX_VALUE){
            for(Location loc : getPath().getLocations()){
                if(loc.equals(location)) return loc.parent;
            }
        }
        return location.parent;
    }

    public Map<Location, Integer> gScores(){
        return abstractDistance.gScores;
    }

    public int distance(Location loc){
        int dist =  abstractDistance.distance(loc);
        return dist;
    }

    public void setAbstractDistance(RectangularMap map){
        abstractDistance = new AbstractDistance(map, goal, start);
    }

    public void moveOneStep(RectangularMap map, List<HashMap<Location, Agent>> spaceTimeMap, int step, List<Agent> agents){
        Location nextStepLocation = nextStepOf(current);



        if(nextStepLocation != null){
        }

        if(spaceTimeMap.size() == step){
           HashMap<Location, Agent> reservation_table = new HashMap<Location, Agent>();
           spaceTimeMap.add(reservation_table);
        }

        HashMap<Location, Agent> reservation_table = spaceTimeMap.get(step);


        if(this.reachGoal()){

            // situation 3: I reach goal, but I have to move to give way
            Agent agentComeToMyCurrentLocation = reservation_table.get(current);
            if(agentComeToMyCurrentLocation != null){

                Iterable<Location> neighbours = map.getNeighbours(current);

                int minGScore = Integer.MAX_VALUE;
                for(Location neighbour : neighbours){
                    if(!validLocation(map, neighbour)) continue;
                    if(map.getValueAt(neighbour) == BARRIER) continue;
                    if(neighbour.equals(agentComeToMyCurrentLocation.cameFrom.get(current))) continue;
                    if(reservation_table.get(neighbour) != null) continue;

                    Integer gScore = distance(neighbour);

                    if(gScore == Integer.MAX_VALUE) continue;
                    if(minGScore >= gScore){
                        minGScore = gScore;
                        nextStepLocation = neighbour;
                        cameFrom.put(nextStepLocation, current);
                        current = nextStepLocation;
                        reservation_table.put(nextStepLocation, this);
                        spaceTimeMap.set(step, reservation_table);
                        return;
                    }
                }
            } else {
                // I will stay at current location
                return;
            }

        }

        // situation 1: collision with other agent
        // current -> destination, other agent destination -> current
        Agent agentComeToMyCurrentLocation = reservation_table.get(current);

        if(agentComeToMyCurrentLocation != null &&  agentComeToMyCurrentLocation.cameFrom.get(current) != null && agentComeToMyCurrentLocation.cameFrom.get(current).equals(nextStepLocation)){

            Iterable<Location> neighbours = map.getNeighbours(current);

            int minGScore = Integer.MAX_VALUE;
            List<Location> refinedNeighours = new ArrayList<Location>();
            List<Location> nextCollisionNeighours = new ArrayList<Location>();
            for(Location neighbour : neighbours){
                if(!validLocation(map, neighbour)) continue;
                if(map.getValueAt(neighbour) == BARRIER) continue;
                if(neighbour.equals(nextStepLocation)) continue;
                if(reservation_table.get(neighbour) != null) continue;
                if(neighbour.equals(agentComeToMyCurrentLocation.cameFrom.get(current))) continue;

                refinedNeighours.add(neighbour);
                if(neighbour.equals(agentComeToMyCurrentLocation.nextStepOf(current))) nextCollisionNeighours.add(neighbour);
            }

            for(Location neighbour : refinedNeighours){
                if(neighbour.equals(agentComeToMyCurrentLocation.nextStepOf(current))){
                    if(!(refinedNeighours.size() == nextCollisionNeighours.size() && nextCollisionNeighours.get(nextCollisionNeighours.size() -1).equals(neighbour))){
                        continue;
                    }
                }

                Integer gScore = distance(neighbour);

                if(gScore == Integer.MAX_VALUE) continue;
                if(minGScore >= gScore){
                    minGScore = gScore;
                    nextStepLocation = neighbour;
                    cameFrom.put(nextStepLocation, current);
                    current = nextStepLocation;
                    reservation_table.put(nextStepLocation, this);
                    spaceTimeMap.set(step, reservation_table);
                    return;
                }
            }
        }

        // situation 2: next location occupied with other agent
        // current -> destination, other agent -> destination
        Agent agentOfNextStepLocation = reservation_table.get(nextStepLocation);
        if(map.getValueAt(nextStepLocation) == BARRIER || (agentOfNextStepLocation != null && !agentOfNextStepLocation.equals(this) && agentOfNextStepLocation.priority < this.priority )){

            List<Location> neighbours = (List<Location>) map.getNeighbours(current);

            if(agentOfNextStepLocation != null){
                neighbours.add(current);
            } else {
                this.setAbstractDistance(map);
                this.distance(current);
                nextStepLocation = nextStepOf(current);
                neighbours.add(nextStepLocation);
            }



            int minGScore = Integer.MAX_VALUE;
            Location selectedNeighbour = null;
            for(Location neighbour : neighbours){
                if(!validLocation(map, neighbour)) continue;
                if(map.getValueAt(neighbour) == BARRIER) continue;
                // if(neighbour.equals(nextStepLocation)) continue;
                if(reservation_table.get(neighbour) != null) continue;
                Integer gScore = distance(neighbour);
                if(gScore == null) continue;
                if(minGScore >= gScore){
                    minGScore = gScore;
                    selectedNeighbour = neighbour;
                }
            }


            if(selectedNeighbour.equals(current)){
                current = selectedNeighbour;

                current.parent = nextStepLocation;
            } else{
                cameFrom.put(selectedNeighbour, current);
            }

            reservation_table.put(selectedNeighbour, this);
            spaceTimeMap.set(step, reservation_table);
            return;

        } else {
            reservation_table.put(nextStepLocation, this);
            spaceTimeMap.set(step, reservation_table);
            cameFrom.put(nextStepLocation, current);
            current = nextStepLocation;
            return;
        }
    }

    public boolean validLocation(RectangularMap map, Location loc){
        if(loc == null) return false;
        if(loc.getRow() < 0 || loc.getRow() > (map.getRows() -1) || loc.getColumn() < 0 || loc.getColumn() > (map.getColumns() -1)) return false;
        return true;
    }


    @Override
    public String toString() {
        return getName() + "[start="+start+" goal="+goal+"]";
    }

}

