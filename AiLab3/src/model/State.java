package model;

public class State {
	
	private int row;
	private int col;
	private int heading;
	
	public State(int row, int col, int heading) {
		this.row = row;
		this.col = col;
		this.heading = heading;
	}
	
	public int getRow() {
		return row;
	}
	
	public int getCol() {
		return col;
	}
	
	public int getHeading() {
		return heading;
	}

}
