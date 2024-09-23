#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <utility>
#include <queue>
#include <map>

using namespace cv;


// Struct for node in the A* algorithm
struct AStarNode {
  Point point; // The (x, y) coordinates of the node
  double f, g, h; // A* algorithm values
};

// Custom comparator for priority_queue to determine the order of nodes
// Nodes with lower f-scores are given higher priority in priority queue
struct CompareAStarNode {
  bool operator()(const AStarNode& firstNode, const AStarNode& secondNode) const {
    return firstNode.f > secondNode.f;
  }
};

// Custom comparator for cv::Point to be used in a sorted container like std::map
// We need it because default comparator won't work for comparing cv::Point
struct PointCompare {
  bool operator()(const Point& firstPoint, const Point& secondPoint) const {
    return (firstPoint.y < secondPoint.y) || (firstPoint.y == secondPoint.y && firstPoint.x < secondPoint.x);
  }
};

// Heuristic to estimate the distance between two points in a grid, uses Manhattan distance
double heuristic(const Point &a, const Point &b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// The function implements the A* algorithm
// It takes a graph represented as an adjacency list, a start point and a goal point
// It returns a vector of cv::Point representing the path from the start to the goal
std::vector<Point> aStar(
  const std::map<Point, std::vector<Point>, PointCompare> &graph,
  const Point &start, const Point &goal) {

  // Priority queue to store nodes to be evaluated, sorted by their f-score
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareAStarNode> openSet;

  // Maps to store scores and path information
  std::map<Point, double, PointCompare> gScore;
  std::map<Point, double, PointCompare> fScore;
  std::map<Point, Point, PointCompare> cameFrom;

  // Initializing start node
  gScore[start] = 0;
  fScore[start] = heuristic(start, goal);
  openSet.push({start, fScore[start], gScore[start], heuristic(start, goal)});

  while (!openSet.empty()) {
    // Get the node with the lowest fScore from the priority queue
    AStarNode current = openSet.top();
    openSet.pop();

    // If we reached the goal, reconstruct the path
    if (current.point == goal) {
      std::vector<Point> path;
      Point step = current.point;

      // Reconstruct the path by following the cameFrom map
      while (cameFrom.find(step) != cameFrom.end()) {
        path.push_back(step);
        step = cameFrom[step];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    // Explore the neighbors of the current node and calculate new gScores
    for (const auto &neighbor : graph.at(current.point)) {
      double movementCost = (neighbor.x != current.point.x && neighbor.y != current.point.y) ? 1.41 : 1.0;
      double tentative_gScore = gScore[current.point] + movementCost;

      // If this path to neighbor is better, update scores and path
      if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) {
        cameFrom[neighbor] = current.point;
        gScore[neighbor] = tentative_gScore;
        fScore[neighbor] = gScore[neighbor] + heuristic(neighbor, goal);
        openSet.push({neighbor, fScore[neighbor], gScore[neighbor], heuristic(neighbor, goal)});
      }
    }
  }

  // Return empty vector if no path found
  return std::vector<Point>();
}

void drawPath(Mat &image, const std::vector<Point> &path, const Scalar &color) {
  for (long unsigned int i = 1; i < path.size(); i++) {
    cv::line(image, path[i - 1], path[i], color, 1, cv::LINE_AA);
  }
}

std::pair<Mat, Mat> processMapForAStar(std::string filename) {
    Mat map = imread(filename, cv::IMREAD_GRAYSCALE);
    if (map.empty()) {
        std::cout << "Error: Could not load map. Check the path or filename!" << std::endl;
        return {Mat(), Mat()};
    }

    //original map is too big, we should work with smaller one
    Mat smallerMap(300, 300, CV_8UC1);
    for(int i=1850; i<2150; i++){
        for(int j=1850; j<2150; j++) {
            smallerMap.at<uchar>(i-1850, j-1850) = map.at<uchar>(i, j);
        }
    }

    // Display the rotated smaller map
    rotate(smallerMap, smallerMap, cv::ROTATE_90_COUNTERCLOCKWISE);
    imshow("Smaller map", smallerMap);
    waitKey(1000);

    // Convert to binary image: free space is white, obstacles are black
    Mat binary_map;
    threshold(smallerMap, binary_map, 210, 255, cv::THRESH_BINARY);
    imshow("Binary map", binary_map);
    waitKey(1000);

    // Inflate obstacles by eroding the free space, so the robot doesn't get too close to walls
    int inflation_radius = 3;
    Mat inflated_map;
    erode(binary_map, inflated_map, getStructuringElement(MORPH_ELLIPSE, Size(inflation_radius*2+1, inflation_radius*2+1)));
    imshow("Inflated binary map", inflated_map);
    waitKey(1000);

    return {smallerMap, inflated_map};
}


std::map<Point, std::vector<Point>, PointCompare> constructFreeSpaceGraph(const Mat& binary_map) {

    // Vector to store all the free space coordinates in the binary map
    std::vector<Point> free_spaces;

    // Iterate through the binary map to find free space
    for (int y = 0; y < binary_map.rows; y++) {
        for (int x = 0; x < binary_map.cols; x++) {
            if (binary_map.at<uchar>(y, x) == 255) {
                free_spaces.push_back(Point(x, y));
            }
        }
    }

    // Graph represented as a map where each free space point is connected to its neighbors
    std::map<Point, std::vector<Point>, PointCompare> graph;

    // Initialize the graph by adding each free space point with an empty adjacency list
    for (const auto& point : free_spaces) {
        graph[point] = std::vector<Point>();
    }

    // Connect each free space point to its 8 neighbors
    for (const auto& point : free_spaces) {
        for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
                if (dx == 0 && dy == 0) continue;

                Point neighbor(point.x + dx, point.y + dy);

                // Ensure the neighbor is within bounds and is also free space
                if (neighbor.x >= 0 && neighbor.x < binary_map.cols &&
                    neighbor.y >= 0 && neighbor.y < binary_map.rows &&
                    binary_map.at<uchar>(neighbor.y, neighbor.x) == 255) {

                    graph[point].push_back(neighbor);
                }
            }
        }
    }

    return graph;
}



void runAll(Point start){

    // Load the map and process it for A*
    std::pair<Mat, Mat> maps = processMapForAStar("/home/tarik/Desktop/MR/astarPathfinding/astar/map01.pgm");
    Mat smallerMap = maps.first;
    Mat binary_map = maps.second;
    if(smallerMap.empty() || binary_map.empty()) {
        return;
    }

    std::map<Point, std::vector<Point>, PointCompare> graph = constructFreeSpaceGraph(binary_map);

    // Print the adjacency list for each node
    for (const auto &node : graph) {
        const Point &point = node.first;
        const std::vector<Point> &neighbors = node.second;

        std::cout << "Node (" << point.x << ", " << point.y << ") is connected to: ";
        for (const auto &neighbor : neighbors) {
          std::cout << "(" << neighbor.x << ", " << neighbor.y << ") ";
        }
        std::cout << std::endl;
      }

    // Display the number of nodes in the graph (number of free spaces)
    std::cout << "Graph size (number of nodes): " << graph.size() << std::endl;


    //Point start(98, 149);  // Example start point
    Point goal(164, 192);   // Example goal point

    // Check if the start point is valid (exists in the graph)
    if (graph.find(start) == graph.end()) {
        std::cout << "Error: Start point is outside the free space." << std::endl;
        return;
    }

    // Check if the goal point is valid (exists in the graph)
    if (graph.find(goal) == graph.end()) {
        std::cout << "Error: Goal point is outside the free space." << std::endl;
        return;
    }


    // Perform the A* algorithm
    std::vector<Point> path = aStar(graph, start, goal);

    // If a path is found, display it
    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        for (const auto &p : path) {
          std::cout << "(" << p.x << ", " << p.y << ")" << std::endl;
        }
        drawPath(smallerMap, path, cv::Scalar(0, 0, 255));
        imshow("Optimal path", smallerMap);
        waitKey(0);
      } else {
        std::cout << "No path found!" << std::endl;
      }
}


int main() {
    Point start(103, 140);
    runAll(start);

    return 0;
}
