import java.util.*;
import java.awt.event.MouseEvent;
import java.awt.Graphics;
import java.awt.Color;

import static java.lang.Math.sqrt;
import static java.lang.Math.ulp;

class Agent
{
	private ArrayList<GameState> path = new ArrayList<>();
	private PathPlanner planner = new PathPlanner();
	private double destX = 0.0;
	private double destY = 0.0;

	void update(Model m)
	{
		Controller c = m.getController();
		while (true)
		{
			MouseEvent e = c.nextMouseEvent();
			if (e == null) { break; }
			destX = e.getX();
			destY = e.getY();
		}


		//If left mouse button is clicked do UCS else if the right mouse is clicked do A star search
		if (c.leftMouse)
		{
			GameState startState = new GameState(0.0, null, m.getX(), m.getY());
			GameState goalState = new GameState(0.0, null, destX, destY);
			planner = new PathPlanner(m);

			GameState goal = planner.UniformCostSearch(startState, goalState);
			path = planner.CreatePath(goal);
			if (path.size() > 1)
			{
				m.setDestination(path.get(1).positionOfPlayerX, path.get(1).positionOfPlayerY);
			}
		}
		else if (c.rightMouse)
		{
			GameState startState = new GameState(0.0, null, m.getX(), m.getY());
			GameState goalState = new GameState(0.0, null, destX, destY);
			planner = new PathPlanner(m);

			GameState goal = planner.AStarSearch(startState, goalState, m);
			path = planner.CreatePath(goal);
			if (path.size() > 1)
			{
				m.setDestination(path.get(1).positionOfPlayerX, path.get(1).positionOfPlayerY);
			}
		}
	}

	void drawPlan(Graphics g, Model m)
	{

		g.setColor(Color.red);
		for (int i = 0; i < path.size() - 1; i++)
		{
			g.drawLine((int) path.get(i).positionOfPlayerX, (int) path.get(i).positionOfPlayerY, (int) path.get(i + 1).positionOfPlayerX, (int) path.get(i + 1).positionOfPlayerY);
		}

		g.setColor(Color.orange);
		if (planner.frontier != null)
		{
			while (! planner.frontier.isEmpty())
			{
				GameState current = planner.frontier.remove();
				g.fillOval((int) (current.positionOfPlayerX / 10) * 10, (int) (current.positionOfPlayerY / 10) * 10, 10, 10);
			}
			planner.frontier.clear();
		}
	}

	public static void main(String[] args) throws Exception
	{
		Controller.playGame();
	}
}


class GameState
{
	double cost;
	GameState parent;
	double positionOfPlayerX;
	double positionOfPlayerY;

	GameState(GameState parent)
	{
		this.cost = parent.cost;
		this.parent = parent;
		this.positionOfPlayerX = parent.positionOfPlayerX;
		this.positionOfPlayerY = parent.positionOfPlayerY;
	}

	GameState(double costOfAction, GameState parent, double currentPositionOfPlayerX, double currentPositionOfPlayerY)
	{
		this.cost = costOfAction;
		this.parent = parent;
		this.positionOfPlayerX = currentPositionOfPlayerX;
		this.positionOfPlayerY = currentPositionOfPlayerY;
	}
}


class PathPlanner
{
	private Model model;
	PriorityQueue<GameState> frontier;

	PathPlanner() { }

	PathPlanner(Model m)
	{
		this.model = m;
	}

	GameState UniformCostSearch(GameState startState, GameState goalState)
	{
		CostComparator costComp = new CostComparator();
		frontier = new PriorityQueue<>(costComp);
		StateComparator stateComp = new StateComparator();
		TreeSet<GameState> beenThere = new TreeSet<GameState>(stateComp);
		String[] actions = new String[8];

		actions[0] = "Up";
		actions[1] = "Down";
		actions[2] = "Right";
		actions[3] = "Left";
		actions[4] = "Top left diagonal";
		actions[5] = "Top right diagonal";
		actions[6] = "Bottom left diagonal";
		actions[7] = "Bottom right diagonal";

		//initialize
		startState.cost = 0.0;
		startState.parent = null;

		//Add the start state to the quque and add it to the been there set
		frontier.add(startState);
		beenThere.add(startState);

		while (! frontier.isEmpty())
		{
			//Get the lowest-cost state
			GameState currentState = frontier.poll();
			//Check to see if its the goal state
			if (CheckIfAtGoal(currentState, goalState))
			{
				return currentState;
			}

			//make the neighbors
			for (String action : actions)
			{
				GameState child = transitions(currentState, action); //compute the next state
				//Check to see if the state is on the screen
				if (ValidatePosition(child))
				{
					double actionCost = ActionCost(child, action); // compute the cost for that action
					//Check to see if the child is in the set if so check to see if the cost is lower than the current one
					if (beenThere.contains(child))
					{
						GameState oldChild = beenThere.floor(child);
						if (currentState.cost + actionCost < Objects.requireNonNull(oldChild).cost)
						{
							oldChild.cost = currentState.cost + actionCost;
							oldChild.parent = currentState;
						}
					}
					else
					{
						child.cost = currentState.cost + actionCost;
						child.parent = currentState;
						frontier.add(child);
						beenThere.add(child);

					}
				}
			}
		}
		throw new RuntimeException("There is no path to the goal");
	}

	ArrayList<GameState> CreatePath(GameState goal)
	{
		ArrayList<GameState> path = new ArrayList<>();

		GameState temp = goal;
		path.add(temp);
		while (temp.parent != null)
		{
			temp = temp.parent;
			path.add(0, temp);
		}
		return path;
	}

	GameState AStarSearch(GameState startState, GameState goalState, Model model)
	{
		AStarCostComparator aStarCostComp = new AStarCostComparator(goalState, model);
		frontier = new PriorityQueue<>(aStarCostComp);
		CostComparator costComp = new CostComparator();
		StateComparator stateComp = new StateComparator();
		TreeSet<GameState> beenThere = new TreeSet<GameState>(stateComp);
		String[] actions = new String[8];

		actions[0] = "Up";
		actions[1] = "Down";
		actions[2] = "Right";
		actions[3] = "Left";
		actions[4] = "Top left diagonal";
		actions[5] = "Top right diagonal";
		actions[6] = "Bottom left diagonal";
		actions[7] = "Bottom right diagonal";

		//initialize
		startState.cost = 0.0;
		startState.parent = null;

		//Add the start state to the quque and add it to the been there set
		frontier.add(startState);
		beenThere.add(startState);

		while (! frontier.isEmpty())
		{
			//Get the lowest-cost state
			GameState currentState = frontier.poll();
			//Check to see if its the goal state
			if (CheckIfAtGoal(currentState, goalState))
			{
				return currentState;
			}

			//make the neighbors
			for (String action : actions)
			{
				GameState child = transitions(currentState, action); //compute the next state
				//Check to see if the state is on the screen
				if (ValidatePosition(child))
				{
					double actionCost = ActionCost(child, action); // compute the cost for that action
					//Check to see if the child is in the set if so check to see if the cost is lower than the current one
					if (beenThere.contains(child))
					{
						GameState oldChild = beenThere.floor(child);
						if (currentState.cost + actionCost < Objects.requireNonNull(oldChild).cost)
						{
							oldChild.cost = currentState.cost + actionCost;
							oldChild.parent = currentState;
						}
					}
					else
					{
						child.cost = currentState.cost + actionCost;
						child.parent = currentState;
						frontier.add(child);
						beenThere.add(child);
					}
				}
			}
		}
		throw new RuntimeException("There is no path to the goal");
	}

	private boolean CheckIfAtGoal(GameState currentState, GameState goalState)
	{
		return Math.sqrt((currentState.positionOfPlayerX - goalState.positionOfPlayerX) * (currentState.positionOfPlayerX - goalState.positionOfPlayerX) + (currentState.positionOfPlayerY - goalState.positionOfPlayerY) * (currentState.positionOfPlayerY - goalState.positionOfPlayerY)) < 10;
	}

	private boolean ValidatePosition(GameState currentState)
	{
		boolean xIsValid = false;
		boolean test = false;
		boolean yIsValid = false;
		if (currentState.positionOfPlayerX > 0 && currentState.positionOfPlayerX < Model.XMAX)
		{
			xIsValid = true;
		}
		if (currentState.positionOfPlayerY > 0 && currentState.positionOfPlayerY < Model.YMAX)
		{
			yIsValid = true;
		}
		return xIsValid && yIsValid;
	}

	private GameState transitions(GameState currentState, String action)
	{
		GameState tempState = new GameState(currentState);
		switch (action)
		{
			case "Up":
				tempState.positionOfPlayerY -= 10;
				break;
			case "Down":
				tempState.positionOfPlayerY += 10;
				break;
			case "Right":
				tempState.positionOfPlayerX += 10;
				break;
			case "Left":
				tempState.positionOfPlayerX -= 10;
				break;
			case "Top left diagonal":
				tempState.positionOfPlayerX -= 10;
				tempState.positionOfPlayerY -= 10;
				break;
			case "Top right diagonal":
				tempState.positionOfPlayerX += 10;
				tempState.positionOfPlayerY -= 10;
				break;
			case "Bottom left diagonal":
				tempState.positionOfPlayerX -= 10;
				tempState.positionOfPlayerY += 10;
				break;
			case "Bottom right diagonal":
				tempState.positionOfPlayerX += 10;
				tempState.positionOfPlayerY += 10;
				break;
		}
		return tempState;
	}

	private double ActionCost(GameState currentState, String action)
	{
		double actionCost = 0.0;
		double travelSpeed = model.getTravelSpeed(currentState.positionOfPlayerX, currentState.positionOfPlayerY);
		switch (action)
		{
			case "Up":
				actionCost = 10.0f / travelSpeed;
				break;
			case "Down":
				actionCost = 10.0f / travelSpeed;
				break;
			case "Right":
				actionCost = 10.0f / travelSpeed;
				break;
			case "Left":
				actionCost = 10.0f / travelSpeed;
				break;
			case "Top left diagonal":
				actionCost = sqrt(2) * 10.0f / travelSpeed;
				break;
			case "Top right diagonal":
				actionCost = sqrt(2) * 10.0f / travelSpeed;
				break;
			case "Bottom left diagonal":
				actionCost = sqrt(2) * 10.0f / travelSpeed;
				break;
			case "Bottom right diagonal":
				actionCost = sqrt(2) * 10.0f / travelSpeed;
				break;
		}
		return actionCost;
	}
}

class StateComparator implements Comparator<GameState>
{

	public int compare(GameState a, GameState b)
	{
		if ((int) (a.positionOfPlayerX / 10) < (int) (b.positionOfPlayerX / 10))
		{
			return - 1;
		}
		if ((int) (a.positionOfPlayerX / 10) > (int) (b.positionOfPlayerX / 10))
		{
			return 1;
		}
		return Integer.compare((int) (a.positionOfPlayerY / 10), (int) (b.positionOfPlayerY / 10));
	}
}

class CostComparator implements Comparator<GameState>
{
	public int compare(GameState a, GameState b)
	{
		return Double.compare(a.cost, b.cost);
	}
}

class AStarCostComparator implements Comparator<GameState>
{
	private Model model;
	private GameState goal;

	AStarCostComparator(GameState goal, Model model)
	{
		this.goal = goal;
		this.model = model;
	}

	private double distanceToGoal(GameState currentState, GameState goalState)
	{
		return Math.sqrt((currentState.positionOfPlayerX - goalState.positionOfPlayerX) * (currentState.positionOfPlayerX - goalState.positionOfPlayerX) + (currentState.positionOfPlayerY - goalState.positionOfPlayerY) * (currentState.positionOfPlayerY - goalState.positionOfPlayerY));
	}

	public int compare(GameState a, GameState b)
	{
		double lowestCost = 1.0f / model.getTravelSpeed(310, 410);
		return Double.compare(a.cost + (distanceToGoal(a, goal) * lowestCost), b.cost + (distanceToGoal(b, goal) * lowestCost));
	}
}

