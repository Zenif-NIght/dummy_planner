#include <eigen3/Eigen/Dense>
#include "ros/ros.h"

using namespace Eigen;
using namespace std;
class AStarPlanner
{ //https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py

private:
    /* data */
    class M_Node
    {
    private:
        int m_parent;
        Vector2d m_position;
        int m_g;
        double m_h;
        double m_f;
        int m_id;
    public:
        int parent(){return m_parent;};
        bool has_parent() { return m_parent >= 0; }
        const Vector2d position(){return m_position;};
        const int g(){return m_g;};
        const double h(){return m_h;};
        const double f(){return m_f;};
        const int id() { return m_id;}
        void set_g(int v) { m_g = v; }
        void set_h(double v) { m_h = v; }
        void set_f(double v) { m_f = v; }
        void set_id(int v) { m_id = v; };
        M_Node() : m_parent(-1),m_g(0),m_h(0),m_f(0) {}
        M_Node(Vector2d position, int parent=-1)
        : m_parent(parent), m_position(position),m_g(0),m_h(0),m_f(0){
        }
        ~M_Node(){}

        friend bool operator==( M_Node&, M_Node&);
        friend bool operator==(const M_Node&,const M_Node&);

    };


    MatrixXi m_maze;
    Vector2d m_start;
    Vector2d m_end;

    bool open_flag, loop_flag, path_flag;
    
public:
    typedef vector<Vector2d,Eigen::aligned_allocator<Eigen::Vector2d>> LocList;
    typedef vector<M_Node,Eigen::aligned_allocator<Eigen::Vector2d>> NodeList;
    
    AStarPlanner(Eigen::MatrixXi &map,Vector2d &start,Vector2d &end);
    ~AStarPlanner();
    
    LocList run_astar();
    void convertFrame(LocList &path, Vector2d mapPos, double resolution);
    NodeList::iterator find( NodeList&, M_Node&);
    void set_open_flag(bool f) { open_flag = f; }
    void set_loop_flag(bool f) { loop_flag = f; }
    void set_path_flag(bool f) { path_flag = f; }

    friend bool operator==( AStarPlanner::M_Node&, AStarPlanner::M_Node&);
    friend bool operator==(const AStarPlanner::M_Node&, const AStarPlanner::M_Node&);
};

inline bool operator==( const Vector2d&lhs, const Vector2d&rhs)
    { return lhs(0) == rhs(0) && lhs(1) == rhs(1); }

inline bool operator==( AStarPlanner::M_Node&lhs, AStarPlanner::M_Node&rhs)
    { return lhs.position() == rhs.position(); }

AStarPlanner::AStarPlanner(Eigen::MatrixXi &map,
                            Vector2d &start,
                            Vector2d &end)
    :m_maze(map),m_start(start), m_end(end), open_flag(false), loop_flag(false) { }
    

void AStarPlanner::convertFrame(AStarPlanner::LocList &path, Vector2d mapPos, double resolution){
    // ROS_INFO_STREAM("\n\npath convetion path.size():"<<path.size()); 
    for (int i = 0; i < path.size(); i++)
    {
        ROS_INFO_STREAM("PRE path["<<i<<"] " << path[i](0)<<","<<path[i](1));

        path[i] = path[i]*resolution+mapPos;

        ROS_INFO_STREAM("POST path["<<i<<"] " << path[i](0)<<","<<path[i](1));


    }
    // abort();
}

AStarPlanner::LocList AStarPlanner::run_astar(){
    // # Create start and end node
    M_Node start_node(m_start);
    ROS_INFO_STREAM("start_node " << start_node.position()(0)<<","<<start_node.position()(1)); 
    M_Node end_node(m_end);
    if(start_node ==  end_node) return LocList();

    ROS_INFO_STREAM("end_node " << end_node.position()(0)<<","<<end_node.position()(1)); 


    // # Initialize both open and closed list

    NodeList open_list;
    NodeList closed_list;

    // # Add the start node
    open_list.push_back(start_node);

    // # Loop until you find the end
    // ROS_INFO_STREAM("😅  Loop until you find found the goal"); 
    int loop_count =0;
    while(open_list.size() > 0)
    {
        if (loop_flag) {
           ROS_INFO_STREAM("\nloop_count: "<< loop_count<<"\r"); 
        }
        loop_count ++;

        // # Get the current node
        M_Node current_node = open_list[0];
        int current_index = 0;
        // Find node on open list with smallest f score
        for(int index=1; index<open_list.size(); index++)
        {
            M_Node item = open_list[index];
            if (item.f() < current_node.f())
            {
                current_node = item;
                current_index = index;
            }
        }

        if (open_flag) {
            ROS_INFO_STREAM("Current node: ("<<current_node.position()(0)<<","<<current_node.position()(1)<<") f:"<<current_node.f());
        }
        // # Pop current off open list, add to closed list
        open_list.erase(open_list.begin()+current_index);

        closed_list.push_back(current_node);
        current_node.set_id(closed_list.size()-1);



        // # Found the goal
        if (current_node == end_node)
        {
            if (path_flag) {
                ROS_INFO_STREAM("Found the goal!!");
            }
            LocList path;
            int current = current_node.id();
            // while current is not nil
            while(current>=0)
            {
                if (path_flag) {
                    ROS_INFO_STREAM("  path item: " << closed_list[current].position()(0)<<","<<closed_list[current].position()(1)); 
                }
                path.push_back(closed_list[current].position());
                current = closed_list[current].parent(); // parent 
            }
            reverse(path.begin(),path.end()); // reverse path: start is in front
            return path;
        }

        // # Generate Neighbors
        for( Vector2d new_position : LocList{Vector2d(0,-1),Vector2d(0,1),Vector2d(-1,0),Vector2d(1,0),
                                    Vector2d(-1,-1),Vector2d(-1,1),Vector2d(1,-1),Vector2d(1,1)})  //Adjacent squares
        {
            // # Get node position
            Vector2d node_position = (current_node.position() + new_position);

            // # Make sure within maze
            if (node_position[0] > (m_maze.rows()-1) or node_position[0] < 0 or node_position[1] > (m_maze.cols() -1) or node_position[1] < 0)
                continue;

            // # Make sure walkable terrain
            if (m_maze(node_position[0],node_position[1]) != 0)
                continue;

            // # Create new node
            M_Node neighbor = M_Node(node_position,current_node.id());

            // if already seen (closed_list), skip
            NodeList::iterator it = find(closed_list,neighbor);
            if (it != closed_list.end()) continue;

            // # Create the f, g, and h values
            neighbor.set_g( current_node.g()+1 );
            neighbor.set_h( (neighbor.position() - end_node.position()).squaredNorm() );
            neighbor.set_f( neighbor.g() + neighbor.h() );

            // Check if child is in open_list
            // if so, skip, unless g-cost is smaller
            it = find(open_list,neighbor);
            if (it != open_list.end() && neighbor.g() > it->g()) continue;

            // Add to open list
            open_list.push_back(neighbor);
        }
    } 

    // open set is empty, but goal never reached
    // failure
    // return empty path
    ROS_ERROR("A* failed to find goal");
    LocList path;
    return path;
}

AStarPlanner::~AStarPlanner()
{
}

AStarPlanner::NodeList::iterator AStarPlanner::find( NodeList& list, M_Node &node)
{   
    // ROS_INFO("find: while");
    for(NodeList::iterator m = list.begin(); m!=list.end(); m++)
    {
        // ROS_INFO_STREAM("  item: " << m->position()(0)<<","<<m->position()(1) ); 
        if (*m == node) { return m; }
    }
    return list.end();
}
