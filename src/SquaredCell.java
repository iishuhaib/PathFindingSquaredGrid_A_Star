
public class SquaredCell { 
        int heuristicCost = 0; //Heuristic cost
        int finalCost = 0; //G+H
        int gCost;
        int i, j;
        SquaredCell parent; 
        
        SquaredCell(int i, int j){
            this.i = i;
            this.j = j; 
        }
        
        @Override
        public String toString(){
            return "["+this.i+", "+this.j+"]";
        }
	
}
