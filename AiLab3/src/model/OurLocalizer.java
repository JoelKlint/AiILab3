package model;

import java.util.ArrayList;
import java.util.Random;

import control.EstimatorInterface;

public class OurLocalizer implements EstimatorInterface {

	public static final int NORTH = 0;
	public static final int EAST = 1;
	public static final int SOUTH = 2;
	public static final int WEST = 3;

	private int trueRow, trueCol, trueHeading;

	double[][] transitionMatrix = new double[64][64];
	State[] states = new State[64];
	private int rows, cols, heads;

	public OurLocalizer(int rows, int cols, int heads) {
		this.rows = rows;
		this.cols = cols;
		this.heads = heads;
		trueRow = 0;
		trueCol = 1;
		trueHeading = NORTH;

		// Create all possible states
		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				states[i(row, col, NORTH)] = new State(row, col, NORTH);
				states[i(row, col, EAST)] = new State(row, col, EAST);
				states[i(row, col, SOUTH)] = new State(row, col, SOUTH);
				states[i(row, col, WEST)] = new State(row, col, WEST);
			}
		}
		
		//Create transitionMatrix
		int row, col;
		
		//Middle cases
		for (row = 1; row <= 2; row++) {
			for (col = 1; col <= 2; col++) {
				// North

				// Looking north
				transitionMatrix[i(row, col, NORTH)][i(row - 1, col, NORTH)] = 0.7;
				transitionMatrix[i(row, col, NORTH)][i(row, col + 1, EAST)] = 0.1;
				transitionMatrix[i(row, col, NORTH)][i(row + 1, col, SOUTH)] = 0.1;
				transitionMatrix[i(row, col, NORTH)][i(row, col - 1, WEST)] = 0.1;
				
				// Looking east
				transitionMatrix[i(row, col, EAST)][i(row, col+1, EAST)] = 0.7;
				transitionMatrix[i(row, col, EAST)][i(row - 1, col, NORTH)] = 0.1;
				transitionMatrix[i(row, col, EAST)][i(row + 1, col, SOUTH)] = 0.1;
				transitionMatrix[i(row, col, EAST)][i(row, col - 1, WEST)] = 0.1;
				
				// Looking south
				transitionMatrix[i(row, col, SOUTH)][i(row + 1, col, SOUTH)] = 0.7;
				transitionMatrix[i(row, col, SOUTH)][i(row - 1, col, NORTH)] = 0.1;
				transitionMatrix[i(row, col, SOUTH)][i(row, col + 1, EAST)] = 0.1;
				transitionMatrix[i(row, col, SOUTH)][i(row, col - 1, WEST)] = 0.1;
				
				// Looking west
				transitionMatrix[i(row, col, WEST)][i(row, col - 1, WEST)] = 0.7;
				transitionMatrix[i(row, col, WEST)][i(row - 1, col, NORTH)] = 0.1;
				transitionMatrix[i(row, col, WEST)][i(row + 1, col, SOUTH)] = 0.1;
				transitionMatrix[i(row, col, WEST)][i(row, col + 1, EAST)] = 0.1;


			}
		}
		
		
		//Corner cases
		//North West
		row = 0;
		col = 0;
			//Looking into wall
		transitionMatrix[i(row, col, NORTH)][i(row, col+1, EAST)] = 0.5;
		transitionMatrix[i(row, col, NORTH)][i(row+1, col, SOUTH)] = 0.5;
		transitionMatrix[i(row, col, WEST)][i(row, col+1, EAST)] = 0.5;
		transitionMatrix[i(row, col, WEST)][i(row+1, col, SOUTH)] = 0.5;
			//not looking into wall
		transitionMatrix[i(row, col, EAST)][i(row, col+1, EAST)] = 0.7;
		transitionMatrix[i(row, col, EAST)][i(row+1, col, SOUTH)] = 0.3;
		transitionMatrix[i(row, col, SOUTH)][i(row, col+1, EAST)] = 0.3;
		transitionMatrix[i(row, col, SOUTH)][i(row+1, col, SOUTH)] = 0.7;
		//North East
		row = 0;
		col = 3;
			//Looking into wall
		transitionMatrix[i(row, col, NORTH)][i(row, col-1, WEST)] = 0.5;
		transitionMatrix[i(row, col, NORTH)][i(row+1, col, SOUTH)] = 0.5;
		transitionMatrix[i(row, col, EAST)][i(row, col-1, WEST)] = 0.5;
		transitionMatrix[i(row, col, EAST)][i(row+1, col, SOUTH)] = 0.5;
			//not looking into wall
		transitionMatrix[i(row, col, WEST)][i(row, col-1, WEST)] = 0.7;
		transitionMatrix[i(row, col, WEST)][i(row+1, col, SOUTH)] = 0.3;
		transitionMatrix[i(row, col, SOUTH)][i(row, col-1, WEST)] = 0.3;
		transitionMatrix[i(row, col, SOUTH)][i(row+1, col, SOUTH)] = 0.7;
		//South West
		row = 3;
		col = 0;
			//Looking into wall
		transitionMatrix[i(row, col, SOUTH)][i(row, col+1, EAST)] = 0.5;
		transitionMatrix[i(row, col, SOUTH)][i(row-1, col, NORTH)] = 0.5;
		transitionMatrix[i(row, col, WEST)][i(row, col+1, EAST)] = 0.5;
		transitionMatrix[i(row, col, WEST)][i(row-1, col, NORTH)] = 0.5;
			//not looking into wall
		transitionMatrix[i(row, col, EAST)][i(row, col+1, EAST)] = 0.7;
		transitionMatrix[i(row, col, EAST)][i(row-1, col, NORTH)] = 0.3;
		transitionMatrix[i(row, col, NORTH)][i(row, col+1, EAST)] = 0.3;
		transitionMatrix[i(row, col, NORTH)][i(row-1, col, NORTH)] = 0.7;
		//South East
		row = 3;
		col = 3;
			//Looking into wall
		transitionMatrix[i(row, col, SOUTH)][i(row, col-1, WEST)] = 0.5;
		transitionMatrix[i(row, col, SOUTH)][i(row-1, col, NORTH)] = 0.5;
		transitionMatrix[i(row, col, EAST)][i(row, col-1, WEST)] = 0.5;
		transitionMatrix[i(row, col, EAST)][i(row-1, col, NORTH)] = 0.5;
			//not looking into wall
		transitionMatrix[i(row, col, WEST)][i(row, col-1, WEST)] = 0.7;
		transitionMatrix[i(row, col, WEST)][i(row-1, col, NORTH)] = 0.3;
		transitionMatrix[i(row, col, NORTH)][i(row, col-1, WEST)] = 0.3;
		transitionMatrix[i(row, col, NORTH)][i(row-1, col, NORTH)] = 0.7;
		
		
		/*
		 * Next to wall cases
		 */
		for (col = 1; col <= 2; col++) {
			// North
			row = 0;
			// Looking north
			transitionMatrix[i(row, col, NORTH)][i(row, col + 1, EAST)] = 0.33;
			transitionMatrix[i(row, col, NORTH)][i(row + 1, col, SOUTH)] = 0.33;
			transitionMatrix[i(row, col, NORTH)][i(row, col - 1, WEST)] = 0.33;
			// Looking east
			transitionMatrix[i(row, col, EAST)][i(row, col + 1, EAST)] = 0.7;
			transitionMatrix[i(row, col, EAST)][i(row + 1, col, SOUTH)] = 0.15;
			transitionMatrix[i(row, col, EAST)][i(row, col - 1, WEST)] = 0.15;
			// looking south
			transitionMatrix[i(row, col, SOUTH)][i(row, col + 1, EAST)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row + 1, col, SOUTH)] = 0.7;
			transitionMatrix[i(row, col, SOUTH)][i(row, col - 1, WEST)] = 0.15;
			// looking west
			transitionMatrix[i(row, col, WEST)][i(row, col + 1, EAST)] = 0.15;
			transitionMatrix[i(row, col, WEST)][i(row + 1, col, SOUTH)] = 0.15;
			transitionMatrix[i(row, col, WEST)][i(row, col - 1, WEST)] = 0.7;
			// South
			row = 3;
				//looking north
			transitionMatrix[i(row, col, NORTH)][i(row, col+1, EAST)] = 0.15;
			transitionMatrix[i(row, col, NORTH)][i(row-1, col, NORTH)] = 0.7;
			transitionMatrix[i(row, col, NORTH)][i(row, col-1, WEST)] = 0.15;
				//looking east
			transitionMatrix[i(row, col, EAST)][i(row, col+1, EAST)] = 0.7;
			transitionMatrix[i(row, col, EAST)][i(row-1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, EAST)][i(row, col-1, WEST)] = 0.15;
				//looking south
			transitionMatrix[i(row, col, SOUTH)][i(row, col+1, EAST)] = 0.33;
			transitionMatrix[i(row, col, SOUTH)][i(row-1, col, NORTH)] = 0.33;
			transitionMatrix[i(row, col, SOUTH)][i(row, col-1, WEST)] = 0.33;
				//looking west
			transitionMatrix[i(row, col, WEST)][i(row, col+1, EAST)] = 0.15;
			transitionMatrix[i(row, col, WEST)][i(row-1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, WEST)][i(row, col-1, WEST)] = 0.7;
		}
		for(row = 1; row <= 2; row++) {
			//West
			col = 0;
				//Looking north
			transitionMatrix[i(row, col, NORTH)][i(row-1, col, NORTH)] = 0.7;
			transitionMatrix[i(row, col, NORTH)][i(row, col+1, EAST)] = 0.15;
			transitionMatrix[i(row, col, NORTH)][i(row+1, col, SOUTH)] = 0.15;
				//looking east
			transitionMatrix[i(row, col, EAST)][i(row-1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, EAST)][i(row, col+1, EAST)] = 0.7;
			transitionMatrix[i(row, col, EAST)][i(row+1, col, SOUTH)] = 0.15;
				//looking south
			transitionMatrix[i(row, col, SOUTH)][i(row-1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row, col+1, EAST)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row+1, col, SOUTH)] = 0.7;
				//looking west
			transitionMatrix[i(row, col, WEST)][i(row-1, col, NORTH)] = 0.33;
			transitionMatrix[i(row, col, WEST)][i(row, col+1, EAST)] = 0.33;
			transitionMatrix[i(row, col, WEST)][i(row+1, col, SOUTH)] = 0.33;
			//East
			col = 3;
				//looking north
			transitionMatrix[i(row, col, NORTH)][i(row-1, col, NORTH)] = 0.7;
			transitionMatrix[i(row, col, NORTH)][i(row, col-1, WEST)] = 0.15;
			transitionMatrix[i(row, col, NORTH)][i(row+1, col, SOUTH)] = 0.15;
				//looking east
			transitionMatrix[i(row, col, EAST)][i(row-1, col, NORTH)] = 0.33;
			transitionMatrix[i(row, col, EAST)][i(row, col-1, WEST)] = 0.33;
			transitionMatrix[i(row, col, EAST)][i(row+1, col, SOUTH)] = 0.33;
				//looking south
			transitionMatrix[i(row, col, SOUTH)][i(row-1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row, col-1, WEST)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row+1, col, SOUTH)] = 0.7;
				//looking west
			transitionMatrix[i(row, col, WEST)][i(row-1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, WEST)][i(row, col-1, WEST)] = 0.7;
			transitionMatrix[i(row, col, WEST)][i(row+1, col, SOUTH)] = 0.15;
		}

	}

	private int i(int row, int col, int heading) {
		return 16 * row + 4 * col + heading % 4;
	}

	@Override
	public int getNumRows() {
		return rows;
	}

	@Override
	public int getNumCols() {
		return cols;
	}

	@Override
	public int getNumHead() {
		return heads;
	}

	@Override
	public void update() {
		int currentStateIndex = i(trueRow, trueCol, trueHeading);
		Random rand = new Random();
		double random = rand.nextDouble();
		double totProb = 0;
		double[] probs = transitionMatrix[currentStateIndex];
		for(int col = 0; col < probs.length; col++) {
			
			if(probs[col] != 0) {
				//Check if random is in prob interval
				if(random >= totProb && random < totProb + probs[col] ) {
					// We have found our next state
					State state = states[col];
					trueRow = state.getRow();
					trueCol = state.getCol();
					trueHeading = state.getHeading();
					break;
				}
				else {
					totProb += probs[col];
				}
			}
			
		}
		// TODO Auto-generated method stub

	}

	@Override
	public int[] getCurrentTruePosition() {
		int[] res = new int[2];
		res[0] = trueRow;
		res[1] = trueCol;
		return res;
	}

	@Override
	public int[] getCurrentReading() {
		int[] result = new int[2];
		int currentStateIndex = i(trueRow, trueCol, trueHeading);
		State currentState = states[currentStateIndex];
		
		//Get all neighbors 
		ArrayList<State> neighbors = new ArrayList<State>();
		for(State state : states) {
			if(currentState.isNeighbor(state)) {
				neighbors.add(state);
			}
		}
		ArrayList<State> secondNeighbors = new ArrayList<State>();
		for(State state : states) {
			if(currentState.isSecondNeighbor(state)) {
				secondNeighbors.add(state);
			}
		}

		// RETURN SOMETHING
		Random rand = new Random();
		double random = rand.nextDouble();
		double totProb = 0;
		// return true position
		if(random <= totProb + 0.1) {
			result[0] = trueRow;
			result[1] = trueCol;
			return result;
		}
		totProb += 0.1;
		// return neighbor
		if(random <= totProb + 0.05*neighbors.size()/4) {
			int randomIndex = rand.nextInt(neighbors.size());
			State state = neighbors.get(randomIndex);
			result[0] = state.getRow();
			result[1] = state.getCol();
			return result;
		}
		totProb += 0.05*neighbors.size()/4;
		//return second neighbor
		if(random <= totProb + 0.025*secondNeighbors.size()/4) {
			int randomIndex = rand.nextInt(secondNeighbors.size());
			State state = secondNeighbors.get(randomIndex);
			result[0] = state.getRow();
			result[1] = state.getCol();
			return result;
		}
		totProb += 0.025*secondNeighbors.size()/4;
		
		//return nothing
		return null;
		
	}

	@Override
	public double getCurrentProb(int x, int y) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getOrXY(int rX, int rY, int x, int y) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getTProb(int x, int y, int h, int nX, int nY, int nH) {
		// TODO Auto-generated method stub
		return transitionMatrix[i(x, y, h)][i(nX, nY, nH)];
	}

}
