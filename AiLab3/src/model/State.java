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
	
	public boolean isNeighbor(State state) {
		int oRow = state.getRow();
		int oCol = state.getCol();
		if(oRow == row && oCol == col) {
			return false;
		}
		else if(Math.abs(oRow-row) <= 1 && Math.abs(oCol-col) <= 1) {
			return true;
		}
		return false;
	}
	
	public boolean isSecondNeighbor(State state) {
		int oRow = state.getRow();
		int oCol = state.getCol();
		if(oRow == row && oCol == col) {
			return false;
		}
		int rowDiff = Math.abs(oRow-row);
		int colDiff = Math.abs(oCol-col);
		if((rowDiff == 2 && colDiff <= 2) || (colDiff == 2 && rowDiff <= 2) ) {
			return true;
		}
		return false;
	}
	
	public boolean samePosition(Object o) {
		if(o instanceof State) {
			State inState = (State) o;
			return inState.getRow() == getRow() && inState.getCol() == getCol();
		}
		return false;
	}
	
	public boolean isCorner() {
		return (row == 0 || row ==3) && (col == 0 || col == 3); 
	}
	
	public boolean isWall() {
		return (row == 0 || row ==3) || (col == 0 || col == 3); 
	}

}
