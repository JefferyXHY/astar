package astar;

public class Location implements Comparable<Location>{
    private int row, column;

    public Location parent;
    public int G;
    public int H;

    public Location(int row, int column) {
        this.row = row;
        this.column = column;
        this.G = 0;
        this.H = 0;
        this.parent = null;
    }

    public Location(int row, int column, int g, int h){
        this.row = row;
        this.column = column;
        this.G = g;
        this.H = h;
    }

    public int getRow() {
        return row;
    }

    public int getColumn() {
        return column;
    }

    public Location dump(){
        return new Location(this.row, this.column);
    }

    @Override
    public int compareTo(Location loc){
        if (loc == null) return -1;
        if (G + H > loc.G + loc.H) return 1;
        if (G + H < loc.G + loc.H) return -1;
        return 0;
    }

    @Override
    public int hashCode() {
        final int prime = 31;
        int result = 1;
        result = prime * result + column;
        result = prime * result + row;
        return result;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == null || getClass() != obj.getClass()) return false;
        Location other = (Location) obj;
        return this == obj || other.column == column && other.row == row;
    }

    @Override
    public String toString() {
        return "(" + row + "," + column + ")";
    }

}

