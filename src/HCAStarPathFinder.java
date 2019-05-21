package astar;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class HCAStarPathFinder {
    public final static int BARRIER = 1; // inaccessible location

    private RectangularMap map;   // the map
    private List<Agent> agents;     // the list of agents
    private int maxTimeSteps;     // max time steps to consider in each path
    private RectangularMap refinedMap;

    public HCAStarPathFinder(RectangularMap map, List<Agent> agents, int maxTimeSteps) {
        this.map = map;
        this.agents = agents;
        this.maxTimeSteps = maxTimeSteps;
    }

    public List<Agent> getAgents() {
        return agents;
    }

    public RectangularMap getMap() {
        return map;
    }

    public int maxTimeSteps() {
        return maxTimeSteps;
    }

    public List<Path> findPaths() {
        if(agents.size() == 0) return null;

        List<HashMap<Location, Agent>> spaceTimeMap = new ArrayList<HashMap<Location, Agent>>();
        HashMap<Location, Agent> reservation_table = new HashMap<Location, Agent>();

        for(Agent agent : agents){
            agent.setAbstractDistance(map);
            if(agent.distance(agent.getStart()) == Integer.MAX_VALUE) return null;
            reservation_table.put(agent.getStart(), agent);
        }
        spaceTimeMap.add(reservation_table);


        int step = 1;


        refinedMap = new ArrayMap(map.getRows(), map.getColumns());
        for(int r = 0; r < map.getRows(); r++) {
            for(int c = 0; c < map.getColumns(); c++) {
                refinedMap.setLocationAt(new Location(r,c), map.getValueAt(r,c));
            }
        }

        while(!allAgentsReachGoal() && step < maxTimeSteps() + 1){

            for(Agent agent : agents){
                agent.moveOneStep(refinedMap, spaceTimeMap, step, agents);
                updateRefinedMap(agent);
            }

            step++;
        }

        if(allAgentsReachGoal()){
            return retrievePaths(spaceTimeMap);
        } else {
            return null;
        }

    }

    private boolean allAgentsReachGoal(){
        for(Agent agent : agents){
            if(!agent.reachGoal()) return false;
        }
        return true;
    }

    private List<Path> retrievePaths(List<HashMap<Location, Agent>> spaceTimeMap){
        List<Path> agentPaths = new ArrayList<Path>();
        HashMap<Agent, ArrayList<Location>> reVisitStart = new HashMap<Agent, ArrayList<Location>>();

        for(int i = 0; i< agents.size(); i++){
            agentPaths.add(null);
        }

        for (HashMap<Location, Agent> lA : spaceTimeMap) {
            for(Location l : lA.keySet()){
                int agentPathsIndex = lA.get(l).getPriority();
                Agent agent = agents.get(agentPathsIndex);
                Path path = agentPaths.get(agentPathsIndex);

                if(path == null){
                    path = new Path(l);
                } else {
                    if(path.indexOfLocation(l) != -1){
                        if(reVisitStart.get(agent) == null){
                        	ArrayList<Location> tmpList = new ArrayList<Location>();
                            tmpList.add(l);
                            reVisitStart.put(agent, tmpList);
                        } else{
                        	ArrayList<Location> tmpList = reVisitStart.get(agent);
                            tmpList.add(l);
                            reVisitStart.put(agent, tmpList);
                        }
                    }

                    path.moveTo(l);
                }
                agentPaths.set(agentPathsIndex, path);
            }
        }

        return agentPaths;
    }

    private Path shortPath(List<HashMap<Location, Agent>> spaceTimeMap, Path path, Agent agent, Location location){
        int fromIndex = -1;
        int endIndex = -1;
        for(int i = 0; i < path.getLength(); i++){

            if(fromIndex != -1 && endIndex != -1) break;
            if(path.getLocations().get(i).equals(location)){
                if(fromIndex == -1){
                     fromIndex = i;
                } else {
                    endIndex = i;
                }
            }
        }

        if(fromIndex == -1 || endIndex == -1) return path;

        Path subPath1 = path.subPath(0, fromIndex - 1);
        Path subPath2 = path.subPath(endIndex, path.getLength() - 1);
        for(Location loc : subPath2.getLocations()){
            subPath1.moveTo(loc);
        }

        for(int i = 0; i < subPath1.getLength(); i++){
            Agent agentOccupyLocation = spaceTimeMap.get(i).get(subPath1.getLocations().get(i));
            if(agentOccupyLocation != null && !agentOccupyLocation.equals(agent)) return path;
        }

        return subPath1;
    }

    private void updateRefinedMap(Agent agent){
        if(agent.reachGoal()){
            boolean canUpdateMap = true;
            for(int i = 0; i < agent.getPriority(); i++){
                if(agents.get(i).reachGoal() == false) canUpdateMap = false;
            }
            if(canUpdateMap){
                refinedMap.setLocationAt(agent.getGoal(), 1);
                return;
            }
        }
    }
}
