#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <string>

/*
    This code performs the following steps *without* any console prompts:

    1) Takes an "inner" grid of size (innerRows x innerCols), where 0 = free cell, 1 = obstacle.
       We automatically add a wall border around it, forming an "augmented" grid of size (innerRows+2) x (innerCols+2).

    2) Takes a robot start coordinate (row,col) in the *inner* coordinate system (0-based).
       We offset it by +1 internally to account for the wall border.

    3) Takes multiple parcels, each defined by a pick-up cell and drop-off cell (also in *inner* coords).

    4) Uses BFS to find a path from robot->pickUp, then pickUp->dropOff for each parcel in order.

    5) Converts each BFS path into a movement command string with the following 2-bit codes:
       - 11 = move forward
       - 10 = turn left
       - 01 = turn right
       - 00 = stop
       We also append "PICK|" after finishing a route to a parcel pick-up, and "DROP|" after finishing a route to a drop-off.

    6) Accumulates these commands into one *finalCommands* string, which is returned at the end.

    If BFS fails at any step, an error message is printed, and we return whatever commands we accumulated so far.
*/

// Direction offsets for BFS (Up, Right, Down, Left)
static int dRow[4] = {-1, 0, 1, 0};
static int dCol[4] = {0, 1, 0, -1};

// Orientation codes for the robot:
//   0 = North,  1 = East,  2 = South,  3 = West
// Turning left => (orientation + 3) % 4
// Turning right => (orientation + 1) % 4

// A simple struct to store a grid cell.
struct Cell
{
    int row;
    int col;
};

//------------------------------------------------------------------------------
// Utility: check if (r,c) is inside the grid boundaries.
bool isValid(int r, int c, int N, int M)
{
    return (r >= 0 && r < N && c >= 0 && c < M);
}

//------------------------------------------------------------------------------
// BFS to find a path from (sr,sc) to (er,ec) in grid.
//   - grid[r][c] = 1 => blocked, 0 => free
//   - returns a vector of Cell from start to end if found, or empty if no path.
std::vector<Cell> bfsSingleRobot(const std::vector<std::vector<int>> &grid,
                                 int sr, int sc,
                                 int er, int ec)
{
    int N = grid.size();
    int M = grid[0].size();

    // Basic bounds/blocked checks
    if (!isValid(sr, sc, N, M) || !isValid(er, ec, N, M))
        return {};
    if (grid[sr][sc] == 1 || grid[er][ec] == 1)
        return {};

    // Set up BFS structures
    std::vector<std::vector<bool>> visited(N, std::vector<bool>(M, false));
    std::vector<std::vector<Cell>> parent(N, std::vector<Cell>(M, {-1, -1}));

    std::queue<Cell> q;
    visited[sr][sc] = true;
    q.push({sr, sc});

    bool foundPath = false;

    while (!q.empty())
    {
        Cell curr = q.front();
        q.pop();

        if (curr.row == er && curr.col == ec)
        {
            foundPath = true;
            break;
        }

        for (int i = 0; i < 4; i++)
        {
            int nr = curr.row + dRow[i];
            int nc = curr.col + dCol[i];
            if (isValid(nr, nc, N, M) && !visited[nr][nc] && grid[nr][nc] == 0)
            {
                visited[nr][nc] = true;
                parent[nr][nc] = curr;
                q.push({nr, nc});
            }
        }
    }

    if (!foundPath)
    {
        return {};
    }

    // Reconstruct path
    std::vector<Cell> path;
    int rr = er, cc = ec;
    while (!(rr == sr && cc == sc))
    {
        path.push_back({rr, cc});
        Cell p = parent[rr][cc];
        rr = p.row;
        cc = p.col;
    }
    // include the start cell
    path.push_back({sr, sc});
    std::reverse(path.begin(), path.end());
    return path;
}

//------------------------------------------------------------------------------
// Convert BFS path to a compressed command string using your bit codes:
//   11 = move forward
//   10 = turn left
//   01 = turn right
//   00 = stop
// For a 180° turn, we do two right turns (01_0|01_0|).
// The robotOrientation is updated as we generate commands so we know
// which way the robot is facing for the next segment.
std::string createMovementCommands(const std::vector<Cell> &path, int &orientation)
{
    // If the path is trivial or empty, just "stop"
    if (path.size() < 2)
    {
        return "00_0|";
    }

    // Helper to figure out direction from one cell to the next
    auto getDir = [&](int r1, int c1, int r2, int c2) -> int
    {
        // 0=N,1=E,2=S,3=W
        int dr = r2 - r1;
        int dc = r2 - r1;
        // OOPS: careful, we should do this properly:
        dr = r2 - r1;
        dc = c2 - c1;

        if (dr == -1 && dc == 0)
            return 0; // North
        if (dr == 1 && dc == 0)
            return 2; // South
        if (dr == 0 && dc == 1)
            return 1; // East
        if (dr == 0 && dc == -1)
            return 3; // West

        return -1; // BFS shouldn't produce diagonal or invalid steps
    };

    std::string result;
    size_t i = 0;

    while (i < path.size() - 1)
    {
        int nextDir = getDir(path[i].row, path[i].col, path[i + 1].row, path[i + 1].col);
        if (nextDir == -1)
        {
            i++;
            continue;
        }

        // orientation difference => how we turn
        int diff = (nextDir - orientation + 4) % 4;

        // 180° turn => two right turns
        if (diff == 2)
        {
            result += "01_0|";
            orientation = (orientation + 1) % 4;
            result += "01_0|";
            orientation = (orientation + 1) % 4;
        }
        // turn right => diff == 1
        else if (diff == 1)
        {
            result += "01_0|";
            orientation = (orientation + 1) % 4;
        }
        // turn left => diff == 3
        else if (diff == 3)
        {
            result += "10_0|";
            orientation = (orientation + 3) % 4;
        }
        // diff == 0 => no turn needed

        // Now move forward as many steps as possible in this orientation
        int steps = 0;
        while (i < path.size() - 1)
        {
            int checkDir = getDir(path[i].row, path[i].col, path[i + 1].row, path[i + 1].col);
            if (checkDir == orientation)
            {
                steps++;
                i++;
            }
            else
            {
                break;
            }
        }
        // "11_X" => move forward X steps
        if (steps > 0)
        {
            result += "11_" + std::to_string(steps) + "|";
        }
    }

    // Always end with stop
    result += "00_0|";
    return result;
}

//------------------------------------------------------------------------------
// This function performs the entire workflow of the robot scenario *without* prompting.
// You call it with:
//   1) innerRows, innerCols: dimensions of the "inner" grid (no walls).
//   2) innerGrid: a 2D vector of size (innerRows x innerCols) with 0=free, 1=obstacle.
//   3) startRow, startCol: robot's *inner* coordinates (0-based).
//   4) parcelCount: number of parcels
//   5) pickRows, pickCols, dropRows, dropCols: arrays storing each parcel's pick-up & drop-off.
// The function returns a single string with all the movement commands plus "PICK|" and "DROP|" markers.
// If BFS fails, it stops and returns the commands accumulated so far, plus an error message on cout.
std::string runRobotScenario(int innerRows, int innerCols,
                             const std::vector<std::vector<int>> &innerGrid,
                             int startRow, int startCol,
                             int parcelCount,
                             const std::vector<int> &pickRows,
                             const std::vector<int> &pickCols,
                             const std::vector<int> &dropRows,
                             const std::vector<int> &dropCols)
{
    // 1) Create augmented grid with an added wall border
    int N = innerRows + 2;
    int M = innerCols + 2;

    // Initialize with all 1's => obstacles
    std::vector<std::vector<int>> grid(N, std::vector<int>(M, 1));

    // Fill the "inner" region with user-provided data
    // We'll offset each cell by +1 in both row & col
    for (int r = 0; r < innerRows; r++)
    {
        for (int c = 0; c < innerCols; c++)
        {
            grid[r + 1][c + 1] = innerGrid[r][c];
        }
    }

    // 2) Robot start => offset by +1
    int R_sr = startRow + 1;
    int R_sc = startCol + 1;

    // Check if start is valid
    if (!isValid(R_sr, R_sc, N, M) || grid[R_sr][R_sc] == 1)
    {
        std::cerr << "[ERROR] Robot start is invalid or blocked.\n";
        return "NO_PATH|"; // or empty string if you prefer
    }

    // The robot starts facing North = 0
    int robotOrientation = 0;

    // 3) For each parcel => BFS from (R_sr,R_sc) to pick, then BFS pick->drop
    std::string finalCommands;

    for (int i = 0; i < parcelCount; i++)
    {
        // Convert pick coords to augmented offsets
        int pr = pickRows[i] + 1;
        int pc = pickCols[i] + 1;

        // BFS to pick
        std::vector<Cell> pathPick = bfsSingleRobot(grid, R_sr, R_sc, pr, pc);
        if (pathPick.empty())
        {
            std::cerr << "[ERROR] No valid path to pick-up for parcel #" << (i + 1) << ".\n";
            return finalCommands + "NO_PATH|";
        }
        std::string cmdPick = createMovementCommands(pathPick, robotOrientation);
        finalCommands += cmdPick + "PICK|";

        // Robot is now at pick-up
        R_sr = pr;
        R_sc = pc;

        // BFS to drop
        int dr = dropRows[i] + 1;
        int dc = dropCols[i] + 1;
        std::vector<Cell> pathDrop = bfsSingleRobot(grid, R_sr, R_sc, dr, dc);
        if (pathDrop.empty())
        {
            std::cerr << "[ERROR] No valid path to drop-off for parcel #" << (i + 1) << ".\n";
            return finalCommands + "NO_PATH|";
        }
        std::string cmdDrop = createMovementCommands(pathDrop, robotOrientation);
        finalCommands += cmdDrop + "DROP|";

        // Robot now at drop
        R_sr = dr;
        R_sc = dc;
    }

    // Return the final commands for all parcels
    return finalCommands;
}

//------------------------------------------------------------------------------
// Example "main" demonstrating how to call runRobotScenario with *hard-coded* data
int main()
{
    // Example test data:
    // (similar to the sample you gave: 3 rows, 4 columns, plus the robot start, plus 2 parcels)

    int innerRows = 3;
    int innerCols = 4;

    // This is the "innerGrid", 0=free, 1=obstacle
    std::vector<std::vector<int>> myInnerGrid = {
        {0, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 0, 0}};

    // Robot start in *inner* coords
    int startRow = 0;
    int startCol = 0;

    // Number of parcels
    int parcelCount = 2;

    // Each pick-up / drop-off in *inner* coords
    // Parcel #1: pick(2,3), drop(1,2)
    // Parcel #2: pick(2,0), drop(0,3)
    std::vector<int> pickRows = {2, 2};
    std::vector<int> pickCols = {3, 0};
    std::vector<int> dropRows = {1, 0};
    std::vector<int> dropCols = {2, 3};

    // Now we call runRobotScenario with these parameters. We get back the final command string.
    std::string finalCmds = runRobotScenario(
        innerRows, innerCols,
        myInnerGrid,
        startRow, startCol,
        parcelCount,
        pickRows, pickCols,
        dropRows, dropCols);

    // Print only the final string
    std::cout << finalCmds << "\n";

    return 0;
}
