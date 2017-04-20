import java.util.*;

public class PathFindingSquaredGrid_AStar {
    //A* is like Greedy Best-First-Search in that it can use a heuristic to guide itself

	private static final int DIAGONAL_COST = 14; //final Diagonal Cost
    private static final int V_H_COST = 10; //final Vertical + horizontal cost

    //Blocked cells are just null Cell values in grid
    private static SquaredCell [][] grid;
    //opened cells on the Grid
    private static PriorityQueue<SquaredCell> open;
    //Array List to store the path score (final cost)
    private static ArrayList<Integer> finalPathValues = new ArrayList<>();

    //Array List to store the g cost
    private static ArrayList<Integer> finalPathGValues = new ArrayList<>();


    //Boolean type to check the closed cell
    private static boolean closed[][];
    //Start ROW and Start COLUMN
    private static int startI, startJ;
    //Start ROW and Start COLUMN
    private static int endI, endJ;
    //total cost variable to find the total cost travelled from START node to END node
    private static int totalCost = 0;

    private static int totalGCost = 0;

    private static int totalCostDistance = 0;
    //find the elaped time taken for the AStar METHOD to execute
    private static double timer = 0;
    //get heuristic input from user
    private static String heuristicInput;

    //Set the START cell values using the method
     private void setStartNode(int i, int j){
        startI = i;
        startJ = j;
    }

    //Set the START cell values using the method
    private void setEndNode(int i, int j){ //Set the END cell values
        endI = i;
        endJ = j;
    }

    //Set NULL to Randomly generated Nodes
    private void setBlocked(int i, int j){
        grid[i][j] = null;
    }

    //Method to check and update the t_final_cost based on the heuristic cost value
    private void checkAndUpdateCost(SquaredCell current, SquaredCell t, int cost, int movementCost){
        // Condition to check if cell null or empty then exit from cell
        if(t == null || closed[t.i][t.j])return;
        //Assign heuristic cost and cost to t_final_cost
        int t_final_cost = t.heuristicCost+cost;

        boolean inOpen = open.contains(t);
        if(!inOpen || t_final_cost<t.finalCost){
            t.finalCost = t_final_cost;
            t.gCost = movementCost;
            t.parent = current;
            if(!inOpen)open.add(t);
        }
    }

    private void AStarCalculation(){
        //add the start location to open list.
        open.add(grid[startI][startJ]);
        //class type variable
       SquaredCell current;

        while(true){
            // Assigning the open.poll() value to current
            current = open.poll();
            // if current node is closed then exit loop
            if(current==null)break;
            //initializing true if the current node is null because null means the current node is blocked
            closed[current.i][current.j]=true;

            //if current value is at the end point then return
            if(current.equals(grid[endI][endJ])){
                return;
            }
            /*
            *The cost is checked based on all 8 sides
            *On a square grid that allows 8 directions of movement, use of Diagonal distance (L∞).
            */
            SquaredCell t;


            if(heuristicInput.equals("m")){
                if(current.i-1>=0){
                    t = grid[current.i-1][current.j];
                    checkAndUpdateCost(current, t, current.finalCost+V_H_COST,V_H_COST);

                }

                if(current.j-1>=0){
                    t = grid[current.i][current.j-1];
                    checkAndUpdateCost(current, t, current.finalCost+V_H_COST, V_H_COST);
                }

                if(current.j+1<grid[0].length){
                    t = grid[current.i][current.j+1];
                    checkAndUpdateCost(current, t, current.finalCost+V_H_COST, V_H_COST);
                }

                if(current.i+1<grid.length){
                    t = grid[current.i+1][current.j];
                    checkAndUpdateCost(current, t, current.finalCost+V_H_COST, V_H_COST);

                }
            }else if(heuristicInput.equals("e")){
                if(current.i-1>=0){
                    if(current.j-1>=0){
                        t = grid[current.i-1][current.j-1];
                        checkAndUpdateCost(current, t, current.finalCost+DIAGONAL_COST, V_H_COST);
                    }

                    if(current.j+1<grid[0].length){
                        t = grid[current.i-1][current.j+1];
                        checkAndUpdateCost(current, t, current.finalCost+DIAGONAL_COST, V_H_COST);
                    }
                }

                if(current.i+1<grid.length){
                    if(current.j-1>=0){
                        t = grid[current.i+1][current.j-1];
                        checkAndUpdateCost(current, t, current.finalCost+DIAGONAL_COST, V_H_COST);
                    }

                    if(current.j+1<grid[0].length){
                        t = grid[current.i+1][current.j+1];
                        checkAndUpdateCost(current, t, current.finalCost+DIAGONAL_COST, V_H_COST);
                    }
                }

            }else if(heuristicInput.equals("c")){
                if(current.i-1>=0){
                    t = grid[current.i-1][current.j];
                    checkAndUpdateCost(current, t, current.finalCost+V_H_COST, V_H_COST);

                    if(current.j-1>=0){
                        t = grid[current.i-1][current.j-1];
                        checkAndUpdateCost(current, t, current.finalCost+DIAGONAL_COST, V_H_COST);
                    }

                    if(current.j+1<grid[0].length){
                        t = grid[current.i-1][current.j+1];
                        checkAndUpdateCost(current, t, current.finalCost+DIAGONAL_COST, V_H_COST);
                    }
                }

                if(current.j-1>=0){
                    t = grid[current.i][current.j-1];
                    checkAndUpdateCost(current, t, current.finalCost+V_H_COST, V_H_COST);
                }

                if(current.j+1<grid[0].length){
                    t = grid[current.i][current.j+1];
                    checkAndUpdateCost(current, t, current.finalCost+V_H_COST, V_H_COST);
                }

                if(current.i+1<grid.length){
                    t = grid[current.i+1][current.j];
                    checkAndUpdateCost(current, t, current.finalCost+V_H_COST, V_H_COST);

                    if(current.j-1>=0){
                        t = grid[current.i+1][current.j-1];
                        checkAndUpdateCost(current, t, current.finalCost+DIAGONAL_COST, V_H_COST);
                    }

                    if(current.j+1<grid[0].length){
                        t = grid[current.i+1][current.j+1];
                        checkAndUpdateCost(current, t, current.finalCost+DIAGONAL_COST, V_H_COST);
                    }
                }
            }



        }
    }

    /* draw the N-by-N boolean matrix to standard draw
     *Display the N by N grid in the console as 1s and 0s
     *1s are the open nodes and 0s are the closed nodes
     */
    private void show(boolean[][] a, boolean which) {
        int N = a.length;
        StdDraw.setXscale(-1, N);
        StdDraw.setYscale(-1, N);
        StdDraw.setPenColor(StdDraw.BLACK);
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                if (a[i][j] == which)
                	StdDraw.square(j, N-i-1, .5);
                else StdDraw.filledSquare(j, N-i-1, .5);
    }

    // draw the N-by-N boolean matrix to standard draw, including the points A (x1, y1) and B (x2,y2) to be marked by a RED circle
    private void show(boolean[][] a, boolean which, int x1, int y1, int x2, int y2) {
        int N = a.length;
        StdDraw.setXscale(-1, N);
        StdDraw.setYscale(-1, N);
        StdDraw.setPenColor(StdDraw.BLACK);
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                if (a[i][j] == which) {
                    StdDraw.setPenColor(StdDraw.WHITE);
                    StdDraw.filledSquare(j, N-i-1, .5);
                    StdDraw.setPenColor(StdDraw.BLACK);
                    StdDraw.square(j, N-i-1, .5);

                    if ((i == x1 && j == y1) || (i == x2 && j == y2)) {
                        StdDraw.setPenColor(StdDraw.RED);
                        StdDraw.filledCircle(j, N - i - 1, .5);
                    } else StdDraw.square(j, N - i - 1, .5);
                }else {
            StdDraw.setPenColor(StdDraw.BLACK);
            StdDraw.filledSquare(j, N-i-1, .5);}
    }

    //The current path highlighted with green blocks
    private void showPath(SquaredCell[][] a, int x1, int y1) {
        // Get the length of the grid
        int N = a.length;
        StdDraw.setXscale(-1, N);
        StdDraw.setYscale(-1, N);
        // Black for blocked Nodes
        StdDraw.setPenColor(StdDraw.BLACK);
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                if (a[i][j] != null)
                	if ((i == x1 && j == y1)) {
                        // Display the current path by filling the square using GREEN
                		StdDraw.setPenColor(StdDraw.GREEN);
                		StdDraw.filledSquare(j, N-i-1, .2);
                		if (i ==endI && j==endJ){
                		    // Display the end pointer as red and highlight the block using green
                            StdDraw.setPenColor(StdDraw.GREEN);
                            StdDraw.filledSquare(endJ, N-endI-1, .5);
                		    StdDraw.setPenColor(StdDraw.RED);
                            StdDraw.filledCircle(endJ, N-endI-1, .5);
                        }
                	}else{
                        StdDraw.setPenColor(StdDraw.BLACK);
                        StdDraw.square(j, N-i-1, .5);
                    }else {
                    StdDraw.setPenColor(StdDraw.BLACK);
                    StdDraw.filledSquare(j, N-i-1, .5);
                }
        // Display the start pointer as red and highlight the block using green
        StdDraw.setPenColor(StdDraw.GREEN);
        StdDraw.filledSquare(startJ, N-startI-1, .5);
        StdDraw.setPenColor(StdDraw.RED);
        StdDraw.filledCircle(startJ, N-startI-1, .5);

    }

    /* return a random N-by-N boolean matrix, where each entry is
     *true with probability p
     *return the boolean type type randomly generated grid
     */
    private boolean[][] random(int N, double p) {
        boolean[][] a = new boolean[N][N];
        for (int i = 0; i < N; i++)
            for (int j = 0; j < N; j++)
                a[i][j] = StdRandom.bernoulli(p);
        return a;
    }

    /*
     * Calculate the heuristic cost based on manhattan distance
     * The standard heuristic for a square grid is the Manhattan distance
     */
    private int heuristicManhattan(int i, int j){
        //Manhattan horizontal and vertical
        return Math.abs(i-endI)+Math.abs(j-endJ);//manhattan
    }

    /*
     *Euclidean distance is shorter than Manhattan or diagonal distance,
     *you will still get shortest paths, but A* will take longer to run.
     *The distance can be defined as a straight line between 2 points
     */
    private int heuristicEuclidean(int i, int j){
        // Euclidean Diagonal
        double x = Math.pow(Math.abs(i-endI), 2.0);
        double y = Math.pow(Math.abs(j-endJ), 2.0);
        return (int)Math.sqrt(x + y);// Euclidean
    }

    /*
     * Chebyshev distance, all 8 adjacent cells from the given point can be reached by one unit
     */
    private int heuristicChebyshev(int i, int j){
       return Math.max(Math.abs(i-endI), Math.abs(j-endJ));
    }

    // This method contains the main execution
    private void components(){
        Scanner input = new Scanner(System.in);
        // boolean[][] open = StdArrayIO.readBoolean2D();

        System.out.print("Enter Grid Size: ");
        // The grid size entered by the user
        int N = input.nextInt();

        // The following will generate a user entered value sized squared grid with relatively few obstacles in it
        // The lower the second parameter, the more obstacles (black cells) are generated
        boolean [][] randomlyGenMatrix = random(N, 0.8);

        StdArrayIO.print(randomlyGenMatrix);
        // Display the randomly generated matrix based on the user entered size
        show(randomlyGenMatrix, true);

        /* Reading the coordinates for points A and B on the input squared grid.*/

        //Get user input for START ROW
        System.out.println("Enter START NODE ROW (i) for A > ");
        int Ai = input.nextInt();
        //Get user input for START COLUMN
        System.out.println("Enter START NODE COLUMN (j) for A > ");
        int Aj = input.nextInt();
        //Get user input for END ROW
        System.out.println("Enter END NODE ROW (i) for B > ");
        int Bi = input.nextInt();
        //Get user input for END COLUMN
        System.out.println("Enter END NODE COLUMN (j) for B > ");
        int Bj = input.nextInt();
        System.out.println();

        //Loop 3 Times to get the heuristic values
        for (int u = 0; u < 3; u++) {
            System.out.print("Enter Heuristic to continue: ");
            // Get user input to use the following heuristic
             heuristicInput = input.next().toLowerCase();
            // Start the clock ticking in order to capture the time being spent on the A star algorithm
            Stopwatch timerFlow = new Stopwatch();
            //this point
            grid = new SquaredCell[N][N];
            closed = new boolean[N][N];
            open = new PriorityQueue<>((Object o1, Object o2) -> {
                SquaredCell c1 = (SquaredCell) o1;
                SquaredCell c2 = (SquaredCell) o2;

                return c1.finalCost < c2.finalCost ? -1 :
                        c1.finalCost > c2.finalCost ? 1 : 0;
            });
            //Set start position
            //Setting to 0,0 by default. Will be useful for the UI part
            setStartNode(Ai, Aj);

            //Set End Location
            setEndNode(Bi, Bj);

            for (int i = 0; i < N; ++i) {
                for (int j = 0; j < N; ++j) {
                    grid[i][j] = new SquaredCell(i, j);


                    // Execute heuristic based on the user input heuristic type
                    switch (heuristicInput) {
                        case "m":
                            //Manhattan Method
                            grid[i][j].heuristicCost = heuristicManhattan(i, j);
                            //System.out.print(grid[i][j].heuristicCost+" ");
                            break;

                        case "e":
                            // Euclidean Method
                            grid[i][j].heuristicCost = heuristicEuclidean(i, j);
                            break;

                        case "c":
                            //chebyshev method
                            grid[i][j].heuristicCost = heuristicChebyshev(i, j);
                            break;
                        default:
                            System.out.println("Invalid input, Please Try Again");
                    }
                }
                //System.out.println();
            }
            grid[Ai][Aj].finalCost = 0;

            //ignore blocks
            for (int i = 0; i < N; i++) {
                for (int j = 0; j < N; j++) {
                    if (!randomlyGenMatrix[i][j]) {
                        setBlocked(i, j);
                    }
                }
            }

            //Display initial map
            System.out.println("Grid: ");
            // Loop through the rows
            for (int i = 0; i < N; ++i) {
                // Loop through the columns
                for (int j = 0; j < N; ++j) {
                    //Source Node
                    if (i == Ai && j == Aj) System.out.print("SO  ");
                    // Destination Node
                    else if (i == Bi && j == Bj) System.out.print("DE  ");
                    else if (grid[i][j] != null) System.out.printf("%-3d ", 0);
                    //blocked nodes
                    else System.out.print("BL  ");
                }
                System.out.println();
            }
            System.out.println();
            // Astar method
            AStarCalculation();

            System.out.println("\nScores for cells: ");
            for (int i = 0; i < N; ++i) {
                for (int j = 0; j < N; ++j) {
                    if (grid[i][j] != null) System.out.printf("%-3d ", grid[i][j].finalCost);
                    else System.out.print("BL  ");
                }
                System.out.println();
            }
            System.out.println();

            if (closed[endI][endJ]) {
                //Trace back the path
                System.out.println("Path: ");
                SquaredCell current = grid[endI][endJ];
                //CHECK
                System.out.print(current);
                // If current parent is not equals to null then the loop will get executed
                while (current.parent != null) {
                    //store the path scores
                    // Store the current node final cost value to an array list
                    finalPathValues.add(grid[current.i][current.j].finalCost);
                    finalPathGValues.add(grid[current.i][current.j].gCost);
                    //System.out.println(grid[current.i][current.j].finalCost);

                    //display the path on console
                    showPath(grid, current.i, current.j);
                    System.out.print(" -> " + current.parent);
                    current = current.parent;
                }
                System.out.println();
                // else display a message saying no possible path
            } else System.out.println("No possible path");
            //timer end and store the timer value to the timer variable
            timer = timerFlow.elapsedTime();

            // use of input next line to clear the path and ask the user to enter the next heuristic input
            input.nextLine();

            //print total cost taken and stored within the array
            for (int myValue : finalPathValues) {
                totalCost += myValue;
            }

            for (int myGValue : finalPathGValues){
                totalGCost += myGValue;
            }
            // total cost taken by the path
            System.out.println("Total cost : " + totalCost);
            System.out.println("Total G Cost : "+ totalGCost);
            // clear array list for the next path values
            finalPathValues.clear();
            finalPathGValues.clear();
            // clear total cost for the next total cost value
            totalCost = 0;
            totalGCost = 0;
            // Stop the clock ticking in order to capture the time being spent on inputting the coordinates
            // print the estimated time taken for the A star algorithm to execute
            StdOut.println("Elapsed time = " + timer);
            // clear timer for the next timimg
            timer = 0;

            // System.out.println("Coordinates for A: [" + Ai + "," + Aj + "]");
            // System.out.println("Coordinates for B: [" + Bi + "," + Bj + "]");
            input.nextLine();
            // Display the random generated grid based on the following pointers entered by the user
            show(randomlyGenMatrix, true, Ai, Aj, Bi, Bj);
        }
    }

	public static void main(String[] args) {
        //creating an object of the class to access local methods
        PathFindingSquaredGrid_AStar pathFindingSquaredGrid = new PathFindingSquaredGrid_AStar();
        //calling the components method
        pathFindingSquaredGrid.components();
    }
}
