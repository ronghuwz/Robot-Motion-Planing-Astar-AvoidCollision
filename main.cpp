#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Grid Constants
const int GRID_ROWS = 35;
const int GRID_COLS = 35;
const int OBSTACLE = 1;
const int COLLISION_REGION = 2;
const int FREE = 0;

// Directions for A* movement
vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

// Grid Definition
int grid[GRID_ROWS][GRID_COLS];

// Function to Initialize the Grid
void initializeGrid() {
    for (int i = 0; i < GRID_ROWS; i++)
        for (int j = 0; j < GRID_COLS; j++)
            grid[i][j] = FREE;

    // Obstacles Upper and Lower
    for (int i = 0; i < 16; i++) {
        for (int j = 6; j <= 12; j++) grid[i][j] = OBSTACLE;
        for (int j = 14; j <= 20; j++) grid[i][j] = OBSTACLE;
        for (int j = 22; j <= 28; j++) grid[i][j] = OBSTACLE;
    }
    for (int i = 17; i < GRID_ROWS; i++) {
        for (int j = 6; j <= 12; j++) grid[i][j] = OBSTACLE;
        for (int j = 14; j <= 20; j++) grid[i][j] = OBSTACLE;
        for (int j = 22; j <= 28; j++) grid[i][j] = OBSTACLE;
    }
    // Collision Region
    for (int j = 13; j <= 22; j++) grid[16][j] = COLLISION_REGION;
}



// A* Node Structure
namespace AStar {
    struct Node {
        int x, y;
        int g, h;
        shared_ptr<Node> parent;

        Node(int x, int y, int g, int h, shared_ptr<Node> parent = nullptr)
            : x(x), y(y), g(g), h(h), parent(parent) {}

        int f() const { return g + h; }
        bool operator>(const Node& other) const { return f() > other.f(); }
    };
}



// Manhattan Distance heuristic function
int heuristic(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}



// A* Algorithm Implementation with shared_ptr to manage memory
vector<pair<int, int>> astar(pair<int, int> start, pair<int, int> goal) {
    priority_queue<shared_ptr<AStar::Node>, vector<shared_ptr<AStar::Node>>, function<bool(shared_ptr<AStar::Node>, shared_ptr<AStar::Node>)>> open_list(
        [](shared_ptr<AStar::Node> a, shared_ptr<AStar::Node> b) { return a->f() > b->f(); });

    vector<vector<bool>> closed_list(GRID_ROWS, vector<bool>(GRID_COLS, false));

    shared_ptr<AStar::Node> startNode = make_shared<AStar::Node>(start.first, start.second, 0, heuristic(start.first, start.second, goal.first, goal.second));
    open_list.push(startNode);

    map<pair<int, int>, shared_ptr<AStar::Node>> all_nodes;
    all_nodes[start] = startNode;

    shared_ptr<AStar::Node> last_node = nullptr;

    while (!open_list.empty()) {
        shared_ptr<AStar::Node> current = open_list.top();
        open_list.pop();

        if (current->x == goal.first && current->y == goal.second) {
            last_node = current;
            break;
        }

        if (closed_list[current->x][current->y]) continue;
        closed_list[current->x][current->y] = true;

        for (auto& dir : directions) {
            int nx = current->x + dir.first;
            int ny = current->y + dir.second;

            if (nx >= 0 && ny >= 0 && nx < GRID_ROWS && ny < GRID_COLS && grid[nx][ny] != OBSTACLE) {
                if (!closed_list[nx][ny]) {
                    shared_ptr<AStar::Node> new_node = make_shared<AStar::Node>(nx, ny, current->g + 1, heuristic(nx, ny, goal.first, goal.second), current);
                    open_list.push(new_node);
                    all_nodes[{nx, ny}] = new_node;
                }
            }
        }
    }

    vector<pair<int, int>> path;
    while (last_node) {
        path.emplace_back(last_node->x, last_node->y);
        last_node = last_node->parent;
    }
    reverse(path.begin(), path.end());

    return path;
}



// Check if position is in collision region
bool is_in_collision_region(pair<int, int> pos) {
    return grid[pos.first][pos.second] == COLLISION_REGION;
}


// Function to handle collision
void handle_collision(vector<pair<int, int>>& path1, vector<pair<int, int>>& path2,
                      vector<pair<int, int>>& final_path1, vector<pair<int, int>>& final_path2) {
    size_t i = 0, j = 0;

    while (i < path1.size() || j < path2.size()) {
//        cout << "i:" << i << ", " << "j: " << j << endl;
        bool robot1_can_move = (i < path1.size());
        bool robot2_can_move = (j < path2.size());

        bool robot1_in_collision = (robot1_can_move && is_in_collision_region(path1[i]));
        bool robot2_in_collision = (robot2_can_move && is_in_collision_region(path2[j]));
        
//        std::cout << "robot1_in_collision: " << robot1_in_collision
//        << ", robot2_in_collision: " << robot2_in_collision << std::endl;

        // Move both robots simultaneously if no collision
        if (!robot1_in_collision && !robot2_in_collision) {
            if (robot1_can_move) final_path1.push_back(path1[i++]);
            if (robot2_can_move) final_path2.push_back(path2[j++]);
        }
        
        //Robot 1 is in collision region, Robot 2 moves if it's safe
        else if (robot1_in_collision && !robot2_in_collision) {
            if (robot1_can_move) final_path1.push_back(path1[i++]);
     
            if (j < path2.size() - 1 && !is_in_collision_region(path2[j + 1])) {
                // If next step is safe, move Robot 2 forward
                final_path2.push_back(path2[j]);
                j++;
            } else if (j < path2.size()) {
                // Robot 2 stays in place
                final_path2.push_back(path2[j]);  // Keep the last position
            
            }
        }
        
        
        // Robot 2 is in collision region, Robot 1 moves if it's safe
        else if (robot2_in_collision && !robot1_in_collision) {
            if (robot2_can_move) final_path2.push_back(path2[j++]);
   
//            std::cout << "i: " << i << ", is current in collision: " << is_in_collision_region(path1[i])
//            << ", is next in collision: " << is_in_collision_region(path1[i + 1]) << endl;
            
            if (i < path1.size() - 1 && !is_in_collision_region(path1[i + 1])) {
                // If next step is safe, move Robot 1 forward
                final_path1.push_back(path1[i]);
                i++;
            } else if (i < path1.size()) {
//                  cout << "Robot 1 stays in place" << "i: " << i
//                  << ", x: " << path1[i].first << ", y: " << path1[i].second << std::endl;
                // Robot 1 stays in place
                final_path1.push_back(path1[i]);  // Keep the last position
                
            }
        }
             
        
        
        // Both robots are about to enter the collision region in the next step
        if (!robot1_in_collision && !robot2_in_collision && i < path1.size() - 1 && j < path2.size() - 1 &&
            is_in_collision_region(path1[i + 1]) && is_in_collision_region(path2[j + 1])) {

            cout << "Both robots about to enter collision region. Robot 1 waits outside." << endl;

            final_path2.push_back(path2[j++]);
        }
    }
}



void plot_final_paths(const vector<pair<int, int>>& final_path1, const vector<pair<int, int>>& final_path2, pair<int, int> item1, pair<int, int> item2) {
    Mat img = Mat::zeros(GRID_ROWS * 20, GRID_COLS * 20, CV_8UC3);

    // Draw the base grid
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            Scalar color = (grid[i][j] == OBSTACLE) ? Scalar(0, 0, 255) : Scalar(255, 255, 255);
            if (grid[i][j] == COLLISION_REGION) color = Scalar(0, 255, 0);
            rectangle(img, Point(j * 20, i * 20), Point((j + 1) * 20, (i + 1) * 20), color, FILLED);
        }
    }
    
    // Mark Items as Stars (`*`) using yellow color
        putText(img, "*", Point(item1.second * 20 + 5, item1.first * 20 + 15), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);
        putText(img, "*", Point(item2.second * 20 + 5, item2.first * 20 + 15), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 2);

    // Ensure both paths have the same length by padding waiting steps if necessary
    size_t max_steps = max(final_path1.size(), final_path2.size());
    vector<pair<int, int>> path1_padded = final_path1;
    vector<pair<int, int>> path2_padded = final_path2;

    while (path1_padded.size() < max_steps) path1_padded.push_back(path1_padded.back()); // Hold last position
    while (path2_padded.size() < max_steps) path2_padded.push_back(path2_padded.back()); // Hold last position
    
    
    namedWindow("Robot Path Animation", WINDOW_AUTOSIZE);
    // Iterate through both paths together
    for (size_t i = 0; i < max_steps; i++) {
        Mat img_copy = img.clone(); // Keep base image
        
//        std::cout << "During plot, robo1 x: " << path1_padded[i].second
//        << ", robo1 y: " << path1_padded[i].first << std::endl;

        // Draw Robot 1's position (Red)
        circle(img_copy, Point(path1_padded[i].second * 20 + 10, path1_padded[i].first * 20 + 10),
               5, Scalar(0, 0, 255), FILLED);

        // Draw Robot 2's position (Blue)
        circle(img_copy, Point(path2_padded[i].second * 20 + 10, path2_padded[i].first * 20 + 10),
               5, Scalar(255, 0, 0), FILLED);

        // Show updated frame
        imshow("Robot Path Animation", img_copy);
        waitKey(500);  // Adjust timing (100ms per step)
    }

    // Keep final display until user exits
    waitKey(100);
    destroyAllWindows();
}


void printGrid() {
    cout << "Grid Layout (0=Free, 1=Obstacle, 2=Collision Region):" << endl;
    for (int i = 0; i < GRID_ROWS; i++) {
        for (int j = 0; j < GRID_COLS; j++) {
            if (make_pair(i, j) == make_pair(11, 3)) cout << "S "; // Start1
            else if (make_pair(i, j) == make_pair(8, 22)) cout << "G "; // Goal1
            else if (make_pair(i, j) == make_pair(17, 34)) cout << "S2"; // Start2
            else if (make_pair(i, j) == make_pair(6, 14)) cout << "G2"; // Goal2
            else cout << grid[i][j] << " ";
        }
        cout << endl;
    }
}


int main() {
    initializeGrid();

    // Define the start and goal positions
    pair<int, int> start1 = {11, 3}, goal1 = {8, 21};
    pair<int, int> start2 = {17, 34}, goal2 = {6, 13};
    
    // Define item positions
    pair<int, int> item1 = {8, 22};
    pair<int, int> item2 = {6, 14};
    
    // Check if start or goal positions are inside obstacles
    if (grid[start1.first][start1.second] == OBSTACLE || grid[goal1.first][goal1.second] == OBSTACLE) {
        cout << "Error: Start1 or Goal1 is inside an obstacle!" << endl;
        return -1;
    }
    if (grid[start2.first][start2.second] == OBSTACLE || grid[goal2.first][goal2.second] == OBSTACLE) {
        cout << "Error: Start2 or Goal2 is inside an obstacle!" << endl;
        return -1;
    }

    // Compute A* paths dynamically for forward and return trips
    vector<pair<int, int>> path1_to_goal = astar(start1, goal1);
    vector<pair<int, int>> path2_to_goal = astar(start2, goal2);

    vector<pair<int, int>> path1_to_start = astar(goal1, start1);
    vector<pair<int, int>> path2_to_start = astar(goal2, start2);

    // Combine forward and return paths to form round trips
    vector<pair<int, int>> path1 = path1_to_goal;
    path1.insert(path1.end(), path1_to_start.begin(), path1_to_start.end());

    vector<pair<int, int>> path2 = path2_to_goal;
    path2.insert(path2.end(), path2_to_start.begin(), path2_to_start.end());

    cout << "Path1 Round Trip Size: " << path1.size() << endl;
    cout << "Path2 Round Trip Size: " << path2.size() << endl;

    vector<pair<int, int>> final_path1, final_path2;
    handle_collision(path1, path2, final_path1, final_path2);

    // Plot full round-trip paths
    plot_final_paths(final_path1, final_path2, item1, item2);

    cout << "Final Path 1 Size: " << final_path1.size() << endl;
    cout << "Final Path 2 Size: " << final_path2.size() << endl;

    return 0;
}
