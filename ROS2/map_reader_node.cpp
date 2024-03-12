#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <queue>
#include <cmath>

using namespace std;

struct Node {
    int x, y;
    double g, h; // g: cost from start, h: heuristic cost to goal
    Node* parent;

    Node(int x, int y, double g, double h, Node* parent) : x(x), y(y), g(g), h(h), parent(parent) {}

    double f() const {
        return g + h;
    }
};

struct CompareNodes {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->f() > rhs->f(); // smaller f() value has higher priority
    }
};

std::vector<Node*> aStar(const std::vector<std::vector<bool>>& grid, const Node& start, const Node& goal) {
    std::vector<Node*> path;

    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;
    std::vector<Node*> closedSet;

    openSet.push(new Node(start.x, start.y, 0, std::sqrt((goal.x - start.x) * (goal.x - start.x) + (goal.y - start.y) * (goal.y - start.y)), nullptr));

    while (!openSet.empty()) {
        Node* current = openSet.top();
        openSet.pop();

        if (current->x == goal.x && current->y == goal.y) {
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        closedSet.push_back(current);

        // Generate neighbors
        std::vector<Node> neighbors = {
            {current->x - 1, current->y, current->g + 1, std::sqrt((goal.x - (current->x - 1)) * (goal.x - (current->x - 1)) + (goal.y - current->y) * (goal.y - current->y)), current}, // Left
            {current->x + 1, current->y, current->g + 1, std::sqrt((goal.x - (current->x + 1)) * (goal.x - (current->x + 1)) + (goal.y - current->y) * (goal.y - current->y)), current}, // Right
            {current->x, current->y - 1, current->g + 1, std::sqrt((goal.x - current->x) * (goal.x - current->x) + (goal.y - (current->y - 1)) * (goal.y - (current->y - 1))), current}, // Up
            {current->x, current->y + 1, current->g + 1, std::sqrt((goal.x - current->x) * (goal.x - current->x) + (goal.y - (current->y + 1)) * (goal.y - (current->y + 1))), current} // Down
        };

            for (const auto& neighbor : neighbors) {
            if (neighbor.x < 0 || neighbor.x >= grid.size() || neighbor.y < 0 || neighbor.y >= grid[0].size() || !grid[neighbor.x][neighbor.y]) {
                continue; // Skip if out of bounds or obstacle
            }

            // Check if neighbor is in closed set
            bool inClosedSet = false;
            for (Node* node : closedSet) {
                if (node->x == neighbor.x && node->y == neighbor.y) {
                    inClosedSet = true;
                    break;
                }
            }
            if (inClosedSet) {
                continue;
            }

            // Calculate tentative g score
            double tentativeGScore = current->g + 1;

            // Check if neighbor is in open set or if its g score is better
            bool inOpenSet = false;
            Node* neighborNode = nullptr;
            for (Node* node : openSet) {
                if (node->x == neighbor.x && node->y == neighbor.y) {
                    neighborNode = node;
                    inOpenSet = true;
                    break;
                }
            }

            if (!inOpenSet || tentativeGScore < neighborNode->g) {
                if (!neighborNode) {
                    neighborNode = new Node(neighbor.x, neighbor.y, 0, 0, nullptr);
                }
                neighborNode->parent = current;
                neighborNode->g = tentativeGScore;
                openSet.push(neighborNode);
            }
        }
    }

    // Cleanup
    for (Node* node : closedSet) {
        delete node;
    }
    while (!openSet.empty()) {
        delete openSet.top();
        openSet.pop();
    }

    return path;
}
class MapReader {
public:
    MapReader() : nh_("~") {
        map_sub_ = nh_.subscribe("/map", 1, &MapReader::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        int width = msg->info.width;
        int height = msg->info.height;
        vector<vector<int>> vec(height, vector<int> (width));

    // Iterate through the data array
        for (int i = 0; i < height; ++i) {
                for (int j = 0; j < width; ++j) {
                    // Access occupancy probability value of cell (i, j)
                         int index = i * width + j;
                         int occupancy = msg->data[index];

                         // Do something with the occupancy value
                         // For example, print the occupancy value of each cell
                        // ROS_INFO("Cell (%d, %d): Occupancy = %d", j, i, occupancy);
                        if (occupancy != 0){
                                 vec[i][j] = 1;
                        }
                        else{
                                vec[i][j] = 0;
                        }
                         ROS_INFO("Cell (%d, %d): Occupancy = %d", j, i, vec[i][j]);
                }
        }

        Node start(0, 0, 0, 0, nullptr);
        Node goal(3, 3, 0, 0, nullptr);

        vector<Node*> path = aStar(vec, start, goal);

        if (path.empty()) {
                ROS_INFO("No path found");
        }
        else{
                for (const auto& node:path){
                        ROS_INFO("(%d , %d)", node->x, node->y);
                }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_reader_node");
    MapReader map_reader;
    ros::spin();
    return 0;
}
