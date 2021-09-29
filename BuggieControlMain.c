/*Hybrid Controller v1.0 designed for AllCode formula - Buggie Robot.
 @author Adrian Tukendorf [adt24]
 */

#include "allcode_api.h"

const int WALL_DISTANCE = 57; //Distance wall-robot when value from sensor is bigger, then wall is recorder.
const int NESTED_AREA = 220; //Light level of the nested cell.
const int FLOOR_LINE = 70; //Value indicating presence of floor line.
const int SPEED = 20; //Speed of the robot.
const int SAFE_DISTANCE = 750; //Distance wall-robot to avoid obstacles.
//I managed to calculate these values back in the lab but I am not sure If they still valid because I changed them couple of time for debugging purposes in a little "maze" in my house.

int direction = 0; //0 = 0 , 1 = 90 ,2 = 180 ,3 = 270 (degrees.)
int Pos_X = 2; //Coordinates of the robot, 2 as default because robot starts exploration from this coordinates.
int Pos_Y = 0;
// X --->
int visitedCellsCount = 0; //Visited cell count, can indicate maze exploration completion.
int nestedCellsCount = 0; //Nested cell count.
int state = 0;

/*Structure where most information about the maze and its representation is stored
 if cell was explored, if nested, and information about walls surrounding robot in four boolean variables
 walls[0] - front wall
 walls[1] - right side wall
 walls[2] - back wall
 walls[3] - left side wall
 * false - no wall
 * true - wall present
 */
typedef struct {
    bool explored;
    bool nested;
    bool walls[4];
} Cell;

Cell maze[5][5];

void initializer() {
    /*Initialize all Buggie's internal variables helping represent the maze model
     and keep track of progress. Coordinates and start position.*/
    int i = 0;
    int x = 0;
    int y = 0;


    for (i = 0; i < 8; i++) {
        FA_DelayMillis(100);
        FA_LEDOn(i);
    }

    // Initialize the maze representation by setting its walls attributes to false.
    for (x = 0; x < 5; x++) {
        for (y = 0; y < 5; y++) {
            maze[x][y].walls[0] = false;
            maze[x][y].walls[1] = false;
            maze[x][y].walls[2] = false;
            maze[x][y].walls[3] = false;
            maze[x][y].nested = false;
            maze[x][y].explored = false;
        }}



    /*There are 4 possible start positions but it doesn't matter because location
    according to the maze stays the same. Representation on the LCD will be just rotated. */

    Pos_X = 2;
    Pos_Y = 0;
    direction = 1; //Robot starts heading to the right.
    visitedCellsCount = 0;

    state = 2; //Value passed to state-manager, updating status of the robot. Move to state 2.
}

void cellScanner() {
    /* Cell scanner records information about maze's cells, taking to consideration current direction of the Buggie.
     * Checks if robot completed exploration and look for nest areas, signalizing if one is found. */
    if (visitedCellsCount > 24) {
        FA_DelayMillis(100);
        state = 5; //Maze was explored fully and state is changed to print the maze.
        return;
    }

    if (direction == 0) { //If direction of the robot is 0 degrees then:
        maze[Pos_X][Pos_Y].walls[0] = FA_ReadIR(IR_FRONT) > WALL_DISTANCE; //Record if wall in front of the robot present.
        maze[Pos_X][Pos_Y].walls[1] = FA_ReadIR(IR_RIGHT) > WALL_DISTANCE; //Record if wall on the right is present.
        maze[Pos_X][Pos_Y].walls[2] = FA_ReadIR(IR_REAR) > WALL_DISTANCE; //Record if wall on the back is present.
        maze[Pos_X][Pos_Y].walls[3] = FA_ReadIR(IR_LEFT) > WALL_DISTANCE; //Record if the wall on the left is present.
    } else if (direction == 1) {
        maze[Pos_X][Pos_Y].walls[0] = FA_ReadIR(IR_LEFT) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[1] = FA_ReadIR(IR_FRONT) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[2] = FA_ReadIR(IR_RIGHT) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[3] = FA_ReadIR(IR_REAR) > WALL_DISTANCE;
    } else if (direction == 2) {
        maze[Pos_X][Pos_Y].walls[0] = FA_ReadIR(IR_REAR) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[1] = FA_ReadIR(IR_LEFT) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[2] = FA_ReadIR(IR_FRONT) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[3] = FA_ReadIR(IR_RIGHT) > WALL_DISTANCE;
    } else if (direction == 3) {
        maze[Pos_X][Pos_Y].walls[0] = FA_ReadIR(IR_RIGHT) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[1] = FA_ReadIR(IR_REAR) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[2] = FA_ReadIR(IR_LEFT) > WALL_DISTANCE;
        maze[Pos_X][Pos_Y].walls[3] = FA_ReadIR(IR_FRONT) > WALL_DISTANCE;
    }

    //If cell nested then update maze representation, play sound to indicate nest cell.
    if (FA_ReadLight() < NESTED_AREA) {
        maze[Pos_X][Pos_Y].nested = true;
        nestedCellsCount++;
        FA_PlayNote(440, 500);
        FA_DelayMillis(500);
        FA_PlayNote(580, 500);
    }

    FA_DelayMillis(500);
    state = 3; //Change robot state to 3.
}

void avoid() {
    /*This function helps robot to stay away from walls, not crashing into them.*/

    //If obstacle on the front, go back.
    if (FA_ReadIR(IR_FRONT) > SAFE_DISTANCE) {
        FA_SetMotors(-30, -30);
        FA_DelayMillis(350);
        FA_SetMotors(0, 0);
    }
    //If obstacle on right then turn slightly to the left.
    if (FA_ReadIR(IR_RIGHT) > SAFE_DISTANCE && FA_ReadIR(IR_FRONT_RIGHT) > SAFE_DISTANCE) {
        FA_SetMotors(-20, 25);
        FA_DelayMillis(200);
        FA_SetMotors(0, 0);
    }
    //If obstacle on the left then turn slightly to the right.
    if (FA_ReadIR(IR_LEFT) > SAFE_DISTANCE && FA_ReadIR(IR_FRONT_LEFT) > SAFE_DISTANCE) {
        FA_SetMotors(25, -20);
        FA_DelayMillis(200);
        FA_SetMotors(0, 0);
    }


}

void leftWallRule() {
    /*This function implements left-wall technique to explore every cell of the maze
     * by following left side wall.
     */
    //If cell is nested, turn around.
    if (FA_ReadLight() < NESTED_AREA) {
        FA_Right(180);
        FA_DelayMillis(500);
        direction = ((direction + 2) % 4 + 4) % 4; //Using modulo operator to calculate current heading(wrapping around to always achieve number not greater than 4).
        state = 4; //Going to state 4.
        return;
    }

    if (FA_ReadIR(IR_LEFT) > WALL_DISTANCE) { //Check if turn left is possible.
        FA_Left(90);
        direction = ((direction - 1) % 4 + 4) % 4;
    } else if (FA_ReadIR(IR_FRONT) > WALL_DISTANCE) { //Check if forward is possible.
        FA_DelayMillis(10); //Robot will go forward because status manager drives forward looking for line.
    } else if (FA_ReadIR(IR_RIGHT) > WALL_DISTANCE) { //Check if turn right is possible.
        FA_Right(90);
        direction = ((direction + 1) % 4 + 4) % 4;
    } else { //Dead-end, turn around, update direction.
        FA_Right(180);
        direction = ((direction + 2) % 4 + 4) % 4;
    }
    FA_DelayMillis(500);
    state = 4; //Change robot state to 4.
}

void nextCell() {
    /*After entering to the next cell, check direction, update current position accordingly to current direction of movement
     by incrementing or decrementing position variables(X and Y coordinates).
     example: If robot moves straight ahead, increment Y. If it moves back, decrement Y. */
    if (direction == 0) {
        Pos_Y++;
    } else if (direction == 1) {
        Pos_X++;
    } else if (direction == 2) {
        Pos_Y--;
    } else if (direction == 3) {
        Pos_X--;
    } else {
        FA_LCDPrint("Direction error!", 16, 5, 15, 1, 0); //Error message to check if modulo operations are working correctly.
    }


    // If current cell wasn't explored before, flag as explored and increment visited cells count.
    if (!maze[Pos_X][Pos_Y].explored) {
        maze[Pos_X][Pos_Y].explored = true;
        visitedCellsCount++;
    }
    FA_DelayMillis(500);
    state = 2; //Go to state 2.
}

void exampleMazeModelLoader() {
    //**IT IS NOT ANY ATEMPT TO CHEAT BY HARD CODING MAZE MODEL**
    //This function ** SHOULD BE COMMENTED OR DELETED ** to allow the controller normal functionality.
    //Source: Worksheet 6 "Example maze layout"
    //   X  Y
    //Level 1 (from the bottom))
    maze[0][0].walls[0] = false;
    maze[0][0].walls[1] = true;
    maze[0][0].walls[2] = true;
    maze[0][0].walls[3] = true;
    maze[0][0].nested = true;

    maze[1][0].walls[0] = false;
    maze[1][0].walls[1] = false;
    maze[1][0].walls[2] = true;
    maze[1][0].walls[3] = true;
    maze[1][0].nested = false;

    maze[2][0].walls[0] = true;
    maze[2][0].walls[1] = false;
    maze[2][0].walls[2] = true;
    maze[2][0].walls[3] = false;
    maze[2][0].nested = false;

    maze[3][0].walls[0] = true;
    maze[3][0].walls[1] = true;
    maze[3][0].walls[2] = true;
    maze[3][0].walls[3] = false;
    maze[3][0].nested = true;

    maze[4][0].walls[0] = false;
    maze[4][0].walls[1] = true;
    maze[4][0].walls[2] = true;
    maze[4][0].walls[3] = true;
    maze[4][0].nested = false;
    //Level 2
    maze[0][1].walls[0] = false;
    maze[0][1].walls[1] = true;
    maze[0][1].walls[2] = false;
    maze[0][1].walls[3] = true;
    maze[0][1].nested = false;

    maze[1][1].walls[0] = false;
    maze[1][1].walls[1] = false;
    maze[1][1].walls[2] = false;
    maze[1][1].walls[3] = true;
    maze[1][1].nested = false;

    maze[2][1].walls[0] = true;
    maze[2][1].walls[1] = false;
    maze[2][1].walls[2] = true;
    maze[2][1].walls[3] = false;
    maze[2][1].nested = false;

    maze[3][1].walls[0] = true;
    maze[3][1].walls[1] = false;
    maze[3][1].walls[2] = true;
    maze[3][1].walls[3] = false;
    maze[3][1].nested = false;

    maze[4][1].walls[0] = true;
    maze[4][1].walls[1] = true;
    maze[4][1].walls[2] = false;
    maze[4][1].walls[3] = false;
    maze[4][1].nested = false;
    //Level 3
    maze[0][2].walls[0] = false;
    maze[0][2].walls[1] = false;
    maze[0][2].walls[2] = false;
    maze[0][2].walls[3] = true;
    maze[0][2].nested = false;

    maze[1][2].walls[0] = true;
    maze[1][2].walls[1] = true;
    maze[1][2].walls[2] = false;
    maze[1][2].walls[3] = false;
    maze[1][2].nested = false;

    maze[2][2].walls[0] = false;
    maze[2][2].walls[1] = false;
    maze[2][2].walls[2] = true;
    maze[2][2].walls[3] = true;
    maze[2][2].nested = false;

    maze[3][2].walls[0] = true;
    maze[3][2].walls[1] = false;
    maze[3][2].walls[2] = true;
    maze[3][2].walls[3] = false;
    maze[3][2].nested = false;

    maze[4][2].walls[0] = false;
    maze[4][2].walls[1] = true;
    maze[4][2].walls[2] = true;
    maze[4][2].walls[3] = false;
    maze[4][2].nested = false;
    //Level 4
    maze[0][3].walls[0] = true;
    maze[0][3].walls[1] = false;
    maze[0][3].walls[2] = false;
    maze[0][3].walls[3] = true;
    maze[0][3].nested = false;

    maze[1][3].walls[0] = false;
    maze[1][3].walls[1] = true;
    maze[1][3].walls[2] = true;
    maze[1][3].walls[3] = false;
    maze[1][3].nested = false;

    maze[2][3].walls[0] = true;
    maze[2][3].walls[1] = true;
    maze[2][3].walls[2] = false;
    maze[2][3].walls[3] = true;
    maze[2][3].nested = true;

    maze[3][3].walls[0] = false;
    maze[3][3].walls[1] = false;
    maze[3][3].walls[2] = true;
    maze[3][3].walls[3] = true;
    maze[3][3].nested = false;

    maze[4][3].walls[0] = true;
    maze[4][3].walls[1] = true;
    maze[4][3].walls[2] = false;
    maze[4][3].walls[3] = false;
    maze[4][3].nested = false;
    //Level 5
    maze[0][4].walls[0] = true;
    maze[0][4].walls[1] = false;
    maze[0][4].walls[2] = true;
    maze[0][4].walls[3] = true;
    maze[0][4].nested = true;

    maze[1][4].walls[0] = true;
    maze[1][4].walls[1] = false;
    maze[1][4].walls[2] = false;
    maze[1][4].walls[3] = false;
    maze[1][4].nested = false;

    maze[2][4].walls[0] = true;
    maze[2][4].walls[1] = false;
    maze[2][4].walls[2] = true;
    maze[2][4].walls[3] = false;
    maze[2][4].nested = false;

    maze[3][4].walls[0] = true;
    maze[3][4].walls[1] = false;
    maze[3][4].walls[2] = false;
    maze[3][4].walls[3] = false;
    maze[3][4].nested = false;

    maze[4][4].walls[0] = true;
    maze[4][4].walls[1] = true;
    maze[4][4].walls[2] = true;
    maze[4][4].walls[3] = false;
    maze[4][4].nested = true;

}

void painter() {
    /* Painter function allow the Buggie to display maze representation
     gathered during exploration of the maze.
     I was designing this function after the lock down
     and I wasn't able to test it in the laboratory, because of that I decided
     to hard-code example maze from the worksheet to work on my displaying function
     it's contained in exampleMazeModelLoader() but  */

    // exampleMazeModelLoader(); //This function should be commented (only for debug purposes)

    FA_LCDBacklight(100);
    FA_LCDClear();

    int xCord, yCord; //Coordinates of the maze inside robot structure.
    int xLCD = 0; //Coordinates of the LCD display.
    int yLCD = 25;

    for (xCord = 0; xCord < 5; xCord++) {
        if (xCord > 0) { //After first loop add 5pix to X cord. of the display.
            xLCD = xLCD + 5;
        }

        yLCD = 25; //During every first loop add 25pix to Y cord. of the display.
        for (yCord = 0; yCord < 5; yCord++) {
            yLCD = yLCD - 5;
            if (maze[xCord][yCord].walls[0]) {
                FA_LCDLine(xLCD, yLCD, xLCD + 5, yLCD); //Draws horizontal wall of the cell in front of the robot.
            }
            if (maze[xCord][yCord].walls[1]) {
                FA_LCDLine(xLCD + 5, yLCD, xLCD + 5, yLCD + 5); //Draws vertical wall of the cell on the right side of the robot.
            }
            if (maze[xCord][yCord].walls[2]) {
                FA_LCDLine(xLCD, yLCD + 5, xLCD + 5, yLCD + 5); //Draws horizontal wall of the cell behind the robot.
            }
            if (maze[xCord][yCord].walls[3]) {
                FA_LCDLine(xLCD, yLCD, xLCD, yLCD + 5); //Draws vertical wall of the cell on the left side of the robot.
            }
            if (maze[xCord][yCord].nested) { //If cell is nested draw two crossed lines on the cell.
                FA_LCDLine(xLCD, yLCD, xLCD + 5, yLCD + 5);
                FA_LCDLine(xLCD + 5, yLCD, xLCD, yLCD + 5);
            }

        }
    }

    FA_LCDPrint("Maze model", 10, 35, 10, 2, 0);
    FA_LCDPrint("x = nest", 8, 35, 17, 2, 0);
    FA_LCDNumber(nestedCellsCount, 94, 4, 2, 0); //number of nested areas
    FA_DelayMillis(10000); //10 sec. delay to allow user to read the screen of the Buggie.
    state = 6; //Go to state 6.
}

void finishDance() {
    int i;
    for (i = 0; i < 8; i++) {
        FA_DelayMillis(200);
        FA_LEDOff(i);
    }
    i=0;
    for (i = 0; i < 8; i++) {
        FA_DelayMillis(200);
        FA_LEDOn(i);
    }
    FA_SetMotors(-30, 30);
    FA_LCDBacklight(0);
    FA_DelayMillis(1000);
    FA_LCDBacklight(50);
    FA_SetMotors(30, -30);
    FA_LCDBacklight(100);
    FA_DelayMillis(2000);
    FA_SetMotors(-3, 3);
    FA_DelayMillis(5000);

for (i = 0; i < 8; i++) {
        FA_DelayMillis(200);
        FA_LEDOff(i);
    }
    for (i = 0; i > 8; i--) {
        FA_DelayMillis(200);
        FA_LEDOn(i);
    }
    FA_SetMotors(0, 0);
}

void stateManager() {
    /* State manager, combines low-level, reactive behaviors with higher-level, deliberative behaviors what makes possible hybrid control
     * of the robot. It updates task after execution of certain functions, changes in environment (sensors data) and progress of maze exploration.
     * One of the main tasks is also detecting if Buggie entered a new cell by detecting if line on the floor was crossed.
     */
    while (true) {

        if (state == 1) {
            initializer();
        } else if (state == 2) {
            cellScanner();
            avoid();
        } else if (state == 3) {
            leftWallRule();
        } else if (state == 4) {
            //Drive until next cell entered.

            FA_SetMotors(SPEED, SPEED);
            if (FA_ReadLine(CHANNEL_RIGHT) < FLOOR_LINE && FA_ReadLine(CHANNEL_LEFT) < FLOOR_LINE) { //Detect line on the floor.
                FA_DelayMillis(200); //Delay to allow Buggie to drive to the new cell. (Needs adjustment in the lab); (PlayNote is delaying too)
                FA_PlayNote(423, 220);
                FA_SetMotors(0, 0);
                nextCell();
            }

            avoid();
        } else if (state == 5) {
            painter();
        } else if (state == 6) {
            finishDance();
            avoid();
        } else {
            FA_LCDPrint("State error!", 12, 5, 15, 1, 0);
        }
    }
}

int main() {
    FA_RobotInit(); //ROBOT API initialization
    FA_DelayMillis(500);
    state = 1; //HAVE TO BE "1" or for debugging purposes other
    stateManager();
    return 0; //The end which should never be reached.
}
