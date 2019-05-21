package astar;

import java.util.ArrayList;

public class Path {

    private ArrayList<Location> locations = new ArrayList<>();

    public Path(Location initialPosition) {
        locations.add(initialPosition);
    }

    public void moveTo(Location loc) {
        locations.add(loc);
    }

    public int getLength() {
        return locations.size();
    }

    public int getCost() {
        return getLength()-1;
    }

    public ArrayList<Location> getLocations(){
        return locations;
    }

    public Path subPath(int fromIndex, int endIndex){
        Path newPath = new Path(locations.get(fromIndex));
        for(int i = fromIndex + 1; i <= endIndex; i++){
            newPath.moveTo(locations.get(i));
        }
        return newPath;
    }

    public int indexOfLocation(Location loc){
        for(int i = 0; i< locations.size(); i++){
            if(locations.get(i).equals(loc)) return i;
        }
        return -1;
    }

    @Override
    public String toString() {
        return toLocationsString() + " [path cost= " + getCost() + "]";
    }

    public String toLocationsString() {
        StringBuilder repr = new StringBuilder();
        for(Location loc : locations) {
            repr.append(loc);
        }
        return repr.toString();
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result
                + ((locations == null) ? 0 : locations.hashCode());
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        Path other = (Path) obj;
        if (locations == null) {
            return other.locations == null;
        } else return locations.equals(other.locations);
    }

    /* for testing; do not use */
    Location getLocation(int index) {
        return locations.get(index);
    }
}
