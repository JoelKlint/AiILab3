package model;

import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;

import control.EstimatorInterface;

public class OurLocalizer implements EstimatorInterface {

	public static final int NORTH = 0;
	public static final int EAST = 1;
	public static final int SOUTH = 2;
	public static final int WEST = 3;
	
	private double alpha = 1;
	
	private RealMatrix f = MatrixUtils.createRealMatrix(new double[64][1]);

	private int currentStateIndex;
	private int[] sensorPosition = new int[2];

	double[][] transitionMatrix = new double[64][64];
	double[][] sensorMatrix = new double[64][64];
	double[][] sensorNullMatrix = new double[64][64];
	State[] states = new State[64];
	private int rows, cols, heads;

	public OurLocalizer(int rows, int cols, int heads) {
		this.rows = rows;
		this.cols = cols;
		this.heads = heads;

		Random rand = new Random();
		currentStateIndex = rand.nextInt(states.length);
		
		//Fill start values in f matrix
		for(int row = 0; row < f.getRowDimension(); row++) {
			for(int col = 0; col < f.getColumnDimension(); col++) {
				f.setEntry(row, col, 0.015625);
			}
		}
		
		//Create sensor matrix for when sensor returns nothing
		for(int heading = 0; heading <= WEST; heading++) {
			//Corners
			sensorNullMatrix[i(0, 0, heading)][i(0, 0, heading)] = 0.625;
			sensorNullMatrix[i(0, 3, heading)][i(0, 3, heading)] = 0.625;
			sensorNullMatrix[i(3, 0, heading)][i(3, 0, heading)] = 0.625;
			sensorNullMatrix[i(3, 3, heading)][i(3, 3, heading)] = 0.625;
			
			//walls
			for(int col = 1; col <= 2; col++) {
				sensorNullMatrix[i(0, col, heading)][i(0, col, heading)] = 0.5;
				sensorNullMatrix[i(3, col, heading)][i(3, col, heading)] = 0.5;
			}
			for(int row = 1; row <= 2; row++) {
				sensorNullMatrix[i(row, 0, heading)][i(row, 0, heading)] = 0.5;
				sensorNullMatrix[i(row, 3, heading)][i(row, 0, heading)] = 0.5;
			}
			
			//middle
			for(int row = 1; row <= 2;  row++) {
				for(int col = 1; col <= 2; col++) {
					sensorNullMatrix[i(row, col, heading)][i(row, col, heading)] = 0.325;
				}
				}
		}
		
		// Create all possible states
		for (int row = 0; row < rows; row++) {
			for (int col = 0; col < cols; col++) {
				states[i(row, col, NORTH)] = new State(row, col, NORTH);
				states[i(row, col, EAST)] = new State(row, col, EAST);
				states[i(row, col, SOUTH)] = new State(row, col, SOUTH);
				states[i(row, col, WEST)] = new State(row, col, WEST);
			}
		}

		// Create transitionMatrix
		int row, col;

		// Middle cases
		for (row = 1; row <= 2; row++) {
			for (col = 1; col <= 2; col++) {
				// North

				// Looking north
				transitionMatrix[i(row, col, NORTH)][i(row - 1, col, NORTH)] = 0.7;
				transitionMatrix[i(row, col, NORTH)][i(row, col + 1, EAST)] = 0.1;
				transitionMatrix[i(row, col, NORTH)][i(row + 1, col, SOUTH)] = 0.1;
				transitionMatrix[i(row, col, NORTH)][i(row, col - 1, WEST)] = 0.1;

				// Looking east
				transitionMatrix[i(row, col, EAST)][i(row, col + 1, EAST)] = 0.7;
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

		// Corner cases
		// North West
		row = 0;
		col = 0;
		// Looking into wall
		transitionMatrix[i(row, col, NORTH)][i(row, col + 1, EAST)] = 0.5;
		transitionMatrix[i(row, col, NORTH)][i(row + 1, col, SOUTH)] = 0.5;
		transitionMatrix[i(row, col, WEST)][i(row, col + 1, EAST)] = 0.5;
		transitionMatrix[i(row, col, WEST)][i(row + 1, col, SOUTH)] = 0.5;
		// not looking into wall
		transitionMatrix[i(row, col, EAST)][i(row, col + 1, EAST)] = 0.7;
		transitionMatrix[i(row, col, EAST)][i(row + 1, col, SOUTH)] = 0.3;
		transitionMatrix[i(row, col, SOUTH)][i(row, col + 1, EAST)] = 0.3;
		transitionMatrix[i(row, col, SOUTH)][i(row + 1, col, SOUTH)] = 0.7;
		// North East
		row = 0;
		col = 3;
		// Looking into wall
		transitionMatrix[i(row, col, NORTH)][i(row, col - 1, WEST)] = 0.5;
		transitionMatrix[i(row, col, NORTH)][i(row + 1, col, SOUTH)] = 0.5;
		transitionMatrix[i(row, col, EAST)][i(row, col - 1, WEST)] = 0.5;
		transitionMatrix[i(row, col, EAST)][i(row + 1, col, SOUTH)] = 0.5;
		// not looking into wall
		transitionMatrix[i(row, col, WEST)][i(row, col - 1, WEST)] = 0.7;
		transitionMatrix[i(row, col, WEST)][i(row + 1, col, SOUTH)] = 0.3;
		transitionMatrix[i(row, col, SOUTH)][i(row, col - 1, WEST)] = 0.3;
		transitionMatrix[i(row, col, SOUTH)][i(row + 1, col, SOUTH)] = 0.7;
		// South West
		row = 3;
		col = 0;
		// Looking into wall
		transitionMatrix[i(row, col, SOUTH)][i(row, col + 1, EAST)] = 0.5;
		transitionMatrix[i(row, col, SOUTH)][i(row - 1, col, NORTH)] = 0.5;
		transitionMatrix[i(row, col, WEST)][i(row, col + 1, EAST)] = 0.5;
		transitionMatrix[i(row, col, WEST)][i(row - 1, col, NORTH)] = 0.5;
		// not looking into wall
		transitionMatrix[i(row, col, EAST)][i(row, col + 1, EAST)] = 0.7;
		transitionMatrix[i(row, col, EAST)][i(row - 1, col, NORTH)] = 0.3;
		transitionMatrix[i(row, col, NORTH)][i(row, col + 1, EAST)] = 0.3;
		transitionMatrix[i(row, col, NORTH)][i(row - 1, col, NORTH)] = 0.7;
		// South East
		row = 3;
		col = 3;
		// Looking into wall
		transitionMatrix[i(row, col, SOUTH)][i(row, col - 1, WEST)] = 0.5;
		transitionMatrix[i(row, col, SOUTH)][i(row - 1, col, NORTH)] = 0.5;
		transitionMatrix[i(row, col, EAST)][i(row, col - 1, WEST)] = 0.5;
		transitionMatrix[i(row, col, EAST)][i(row - 1, col, NORTH)] = 0.5;
		// not looking into wall
		transitionMatrix[i(row, col, WEST)][i(row, col - 1, WEST)] = 0.7;
		transitionMatrix[i(row, col, WEST)][i(row - 1, col, NORTH)] = 0.3;
		transitionMatrix[i(row, col, NORTH)][i(row, col - 1, WEST)] = 0.3;
		transitionMatrix[i(row, col, NORTH)][i(row - 1, col, NORTH)] = 0.7;

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
			// looking north
			transitionMatrix[i(row, col, NORTH)][i(row, col + 1, EAST)] = 0.15;
			transitionMatrix[i(row, col, NORTH)][i(row - 1, col, NORTH)] = 0.7;
			transitionMatrix[i(row, col, NORTH)][i(row, col - 1, WEST)] = 0.15;
			// looking east
			transitionMatrix[i(row, col, EAST)][i(row, col + 1, EAST)] = 0.7;
			transitionMatrix[i(row, col, EAST)][i(row - 1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, EAST)][i(row, col - 1, WEST)] = 0.15;
			// looking south
			transitionMatrix[i(row, col, SOUTH)][i(row, col + 1, EAST)] = 0.33;
			transitionMatrix[i(row, col, SOUTH)][i(row - 1, col, NORTH)] = 0.33;
			transitionMatrix[i(row, col, SOUTH)][i(row, col - 1, WEST)] = 0.33;
			// looking west
			transitionMatrix[i(row, col, WEST)][i(row, col + 1, EAST)] = 0.15;
			transitionMatrix[i(row, col, WEST)][i(row - 1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, WEST)][i(row, col - 1, WEST)] = 0.7;
		}
		for (row = 1; row <= 2; row++) {
			// West
			col = 0;
			// Looking north
			transitionMatrix[i(row, col, NORTH)][i(row - 1, col, NORTH)] = 0.7;
			transitionMatrix[i(row, col, NORTH)][i(row, col + 1, EAST)] = 0.15;
			transitionMatrix[i(row, col, NORTH)][i(row + 1, col, SOUTH)] = 0.15;
			// looking east
			transitionMatrix[i(row, col, EAST)][i(row - 1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, EAST)][i(row, col + 1, EAST)] = 0.7;
			transitionMatrix[i(row, col, EAST)][i(row + 1, col, SOUTH)] = 0.15;
			// looking south
			transitionMatrix[i(row, col, SOUTH)][i(row - 1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row, col + 1, EAST)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row + 1, col, SOUTH)] = 0.7;
			// looking west
			transitionMatrix[i(row, col, WEST)][i(row - 1, col, NORTH)] = 0.33;
			transitionMatrix[i(row, col, WEST)][i(row, col + 1, EAST)] = 0.33;
			transitionMatrix[i(row, col, WEST)][i(row + 1, col, SOUTH)] = 0.33;
			// East
			col = 3;
			// looking north
			transitionMatrix[i(row, col, NORTH)][i(row - 1, col, NORTH)] = 0.7;
			transitionMatrix[i(row, col, NORTH)][i(row, col - 1, WEST)] = 0.15;
			transitionMatrix[i(row, col, NORTH)][i(row + 1, col, SOUTH)] = 0.15;
			// looking east
			transitionMatrix[i(row, col, EAST)][i(row - 1, col, NORTH)] = 0.33;
			transitionMatrix[i(row, col, EAST)][i(row, col - 1, WEST)] = 0.33;
			transitionMatrix[i(row, col, EAST)][i(row + 1, col, SOUTH)] = 0.33;
			// looking south
			transitionMatrix[i(row, col, SOUTH)][i(row - 1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row, col - 1, WEST)] = 0.15;
			transitionMatrix[i(row, col, SOUTH)][i(row + 1, col, SOUTH)] = 0.7;
			// looking west
			transitionMatrix[i(row, col, WEST)][i(row - 1, col, NORTH)] = 0.15;
			transitionMatrix[i(row, col, WEST)][i(row, col - 1, WEST)] = 0.7;
			transitionMatrix[i(row, col, WEST)][i(row + 1, col, SOUTH)] = 0.15;
		}
		
		//Create sensor matrix
		for(int fromHeading = NORTH; fromHeading <= WEST; fromHeading++) {
			for(int toHeading = NORTH; toHeading <= WEST; toHeading++) {
				for(State currentState : states) {
					int cRow = currentState.getRow();
					int cCol = currentState.getCol();
					for(State sensorState : states) {
						int sRow = sensorState.getRow();
						int sCol = sensorState.getCol();
						if(sensorState.samePosition(currentState)) {
							sensorMatrix[i(cRow, cCol, fromHeading)][i(sRow, sCol, toHeading)] = 0.1;
							//sensorMatrix[i(cRow, cCol, fromHeading)][i(sRow, sCol, toHeading)] = 0.025;
						}
						else if(sensorState.isNeighbor(currentState)) {
							sensorMatrix[i(cRow, cCol, fromHeading)][i(sRow, sCol, toHeading)] = 0.05;
							//sensorMatrix[i(cRow, cCol, fromHeading)][i(sRow, sCol, toHeading)] = 0.0125;
						}
						else if(sensorState.isSecondNeighbor(currentState)) {
							sensorMatrix[i(cRow, cCol, fromHeading)][i(sRow, sCol, toHeading)] = 0.025;
							//sensorMatrix[i(cRow, cCol, fromHeading)][i(sRow, sCol, toHeading)] = 0.00625;
						}
					}
				}
			}
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
		
		double[][] currentSensorO = new double[64][64];
		//System.out.println("New X Y");
		for(int i = 0; i < sensorMatrix.length; i++) {
			if(sensorPosition != null) {
				int sRow = sensorPosition[0];
				int sCol = sensorPosition[1];
				currentSensorO[i][i] = sensorMatrix[i(sRow, sCol, NORTH)][i];
			}
			else {
				currentSensorO = sensorNullMatrix;				
			}
		}
		RealMatrix sensor = MatrixUtils.createRealMatrix(currentSensorO);
		RealMatrix transition = MatrixUtils.createRealMatrix(transitionMatrix);
		transition = transition.transpose();
		RealMatrix res = sensor.multiply(transition);
		
		f = res.multiply(f);
		double fSum = 0;
		for(double value : f.getColumn(0)) {
			fSum += value;
		}
		alpha = 1/fSum;
		f = f.scalarMultiply(alpha);

		Random rand = new Random();
		double random = rand.nextDouble();
		double totProb = 0;
		double[] probs = transitionMatrix[currentStateIndex];
		// Move one step
		for (int col = 0; col < probs.length; col++) {
			if (probs[col] != 0) {
				// Check if random is in prob interval
				if (random >= totProb && random < totProb + probs[col]) {
					// We have found our next state
					currentStateIndex = col;
					break;
				} else {
					totProb += probs[col];
				}
			}
			
		}
		
	}

	@Override
	public int[] getCurrentTruePosition() {
		int[] res = new int[2];
		res[0] = states[currentStateIndex].getRow();
		res[1] = states[currentStateIndex].getCol();
		System.out.println("I am hiding in " + res[0] + "," + res[1] + ". Come find me");
		return res;
	}

	@Override
	public int[] getCurrentReading() {
		sensorPosition = new int[2];
		State currentState = states[currentStateIndex];

		// Get all neighbors
		ArrayList<State> neighbors = new ArrayList<State>();
		for (State state : states) {
			if (currentState.isNeighbor(state)) {
				neighbors.add(state);
			}
		}
		ArrayList<State> secondNeighbors = new ArrayList<State>();
		for (State state : states) {
			if (currentState.isSecondNeighbor(state)) {
				secondNeighbors.add(state);
			}
		}

		// RETURN SOMETHING
		Random rand = new Random();
		double random = rand.nextDouble();
		double totProb = 0;
		// return true position
		if (random <= totProb + 0.1) {
			sensorPosition[0] = states[currentStateIndex].getRow();
			sensorPosition[1] = states[currentStateIndex].getCol();
			System.out.println("Sensor gave position " + sensorPosition[0] + "," + sensorPosition[1]);
			return sensorPosition;
		}
		totProb += 0.1;
		// return neighbor
		if (random <= totProb + 0.05 * neighbors.size() / 4) {
			int randomIndex = rand.nextInt(neighbors.size());
			State state = neighbors.get(randomIndex);
			sensorPosition[0] = state.getRow();
			sensorPosition[1] = state.getCol();
			System.out.println("Sensor gave position " + sensorPosition[0] + "," + sensorPosition[1]);
			return sensorPosition;
		}
		totProb += 0.05 * neighbors.size() / 4;
		// return second neighbor
		if (random <= totProb + 0.025 * secondNeighbors.size() / 4) {
			int randomIndex = rand.nextInt(secondNeighbors.size());
			State state = secondNeighbors.get(randomIndex);
			sensorPosition[0] = state.getRow();
			sensorPosition[1] = state.getCol();
			System.out.println("Sensor gave position " + sensorPosition[0] + "," + sensorPosition[1]);
			return sensorPosition;
		}
		totProb += 0.025 * secondNeighbors.size() / 4;

		// return nothing
		sensorPosition = null;
		System.out.println("No sensor reading");
		return sensorPosition;

	}

	@Override
	public double getCurrentProb(int x, int y) {
		double res = 0;
		res += f.getEntry(i(x, y, NORTH), 0);
		res += f.getEntry(i(x, y, EAST), 0);
		res += f.getEntry(i(x, y, SOUTH), 0);
		res += f.getEntry(i(x, y, WEST), 0);
		return res;
	}

	@Override
	public double getOrXY(int rX, int rY, int x, int y) {
		if(rY == -1 || rX == -1) {
			return sensorNullMatrix[i(x, y, NORTH)][i(x, y, NORTH)];
		}
		else {
			return sensorMatrix[i(x, y, NORTH)][i(rX, rY, NORTH)];
		}
	}

	@Override
	public double getTProb(int x, int y, int h, int nX, int nY, int nH) {
		return transitionMatrix[i(x, y, h)][i(nX, nY, nH)];
	}

}
