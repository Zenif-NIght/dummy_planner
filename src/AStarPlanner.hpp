    #include <eigen3/Eigen/Dense>
using namespace Eigen;
class AStarPlanner
{ //https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py

private:
    /* data */
    class M_Node
    {
    private:
        Vector2d *m_parent;
        Vector2d m_position;
        Vector2d m_g;
        Vector2d m_h;
        Vector2d m_f;
    public:
        *Vector2d parent(){return m_parent;};
        Vector2d position(){return m_position;};
        Vector2d g(){return m_g;};
        Vector2d h(){return m_h;};
        Vector2d f(){return m_f;};
        M_Node(){}
        M_Node(Vector2d *parent, Vector2d position)
        : m_parent(parent), m_position(position),m_g(0),m_h(0),m_f(0){}
        ~M_Node(){}
    };

    MatrixXd m_maze;
    Vector2d m_start;
    Vector2d m_end;


    
public:
    AStarPlanner();
    AStarPlanner(Eigen::MatrixXd &map,Vector2d &start,Vector2d &end);
    
    std::vector<Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> run_astar();

    ~AStarPlanner();
};
AStarPlanner::AStarPlanner(){}
AStarPlanner::AStarPlanner(Eigen::MatrixXd &map,
                            Vector2d &start,
                            Vector2d &end)
    :m_maze(map),m_start(start), m_end(end){ }

std::vector<Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> AStarPlanner::run_astar(){
    //      # Create start and end node
    // start_node = Node(None, start)
    M_Node start_node(nullptr,m_start);
    // start_node.g = start_node.h = start_node.f = 0
    // end_node = Node(None, end)
    // end_node.g = end_node.h = end_node.f = 0

    // # Initialize both open and closed list
    // open_list = []
    // closed_list = []

    // # Add the start node
    // open_list.append(start_node)

    // # Loop until you find the end
    // while len(open_list) > 0:

    //     # Get the current node
    //     current_node = open_list[0]
    //     current_index = 0
    //     for index, item in enumerate(open_list):
    //         if item.f < current_node.f:
    //             current_node = item
    //             current_index = index

    //     # Pop current off open list, add to closed list
    //     open_list.pop(current_index)
    //     closed_list.append(current_node)

    //     # Found the goal
    //     if current_node == end_node:
    //         path = []
    //         current = current_node
    //         while current is not None:
    //             path.append(current.position)
    //             current = current.parent
    //         return path[::-1] # Return reversed path

    //     # Generate children
    //     children = []
    //     for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

    //         # Get node position
    //         node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

    //         # Make sure within range
    //         if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
    //             continue

    //         # Make sure walkable terrain
    //         if maze[node_position[0]][node_position[1]] != 0:
    //             continue

    //         # Create new node
    //         new_node = Node(current_node, node_position)

    //         # Append
    //         children.append(new_node)

    //     # Loop through children
    //     for child in children:

    //         # Child is on the closed list
    //         for closed_child in closed_list:
    //             if child == closed_child:
    //                 continue

    //         # Create the f, g, and h values
    //         child.g = current_node.g + 1
    //         child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
    //         child.f = child.g + child.h

    //         # Child is already in the open list
    //         for open_node in open_list:
    //             if child == open_node and child.g > open_node.g:
    //                 continue

    //         # Add the child to the open list
    //         open_list.append(child)
        
        std::vector<Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> path;
        return path;
}
AStarPlanner::~AStarPlanner()
{
}
