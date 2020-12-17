#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include <algorithm>

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
        Vector2i m_position;
        int m_g;
        double m_h;
        double m_f;
        int m_id;
    public:
        int parent(){return m_parent;};
        bool has_parent() { return m_parent >= 0; }
        Vector2i position(){return m_position;};
        const int g(){return m_g;};
        const double h(){return m_h;};
        const double f(){return m_f;};
        const int id() { return m_id;}
        void set_g(int v) { m_g = v; }
        void set_h(double v) { m_h = v; }
        void set_f(double v) { m_f = v; }
        void set_id(int v) { m_id = v; };
        M_Node() : m_parent(-1),m_g(0),m_h(0),m_f(0) {}
        M_Node(Vector2i position, int parent=-1)
        : m_parent(parent), m_position(position),m_g(0),m_h(0),m_f(0){
        }
        ~M_Node(){}

        friend bool operator==( M_Node&, M_Node&);
        friend bool operator==(const M_Node&,const M_Node&);

    };


    MatrixXi m_maze;
    Vector2i m_start;
    Vector2i m_end;

    bool open_flag, loop_flag, path_flag, g_cost, h_cost;
    int search_depth;

    
public:
    typedef vector<Vector2i,Eigen::aligned_allocator<Eigen::Vector2i>> LocList;
    typedef vector<M_Node,Eigen::aligned_allocator<Eigen::Vector2i>> NodeList;
    
    AStarPlanner(Eigen::MatrixXi &map,Vector2i &start,Vector2i &end);
    ~AStarPlanner();
    
    LocList run_astar();
    NodeList::iterator find( NodeList&, M_Node&);
    bool isWall(Vector2i& position);
    int next_to_wall(Vector2i position);
    bool valid_position(Vector2i & node_position);
    void set_open_flag(bool f) { open_flag = f; }
    void set_loop_flag(bool f) { loop_flag = f; }
    void set_path_flag(bool f) { path_flag = f; }
    void use_g_cost(bool f) { g_cost = f; }
    void use_h_cost(bool f) { h_cost = f; }
    void set_max_depth(int v) { search_depth = v; }

    friend bool operator==( AStarPlanner::M_Node&, AStarPlanner::M_Node&);
    friend bool operator==(const AStarPlanner::M_Node&, const AStarPlanner::M_Node&);
    
private:
    void dump_list(const char* msg, NodeList &nl);
};

inline bool operator==( const Vector2i&lhs, const Vector2i&rhs)
    { return lhs(0) == rhs(0) && lhs(1) == rhs(1); }

inline bool operator==( AStarPlanner::M_Node&lhs, AStarPlanner::M_Node&rhs)
    { return lhs.position() == rhs.position(); }

AStarPlanner::AStarPlanner(Eigen::MatrixXi &map,
                            Vector2i &start,
                            Vector2i &end)
    :m_maze(map),m_start(start), m_end(end), 
    open_flag(false), loop_flag(false), path_flag(false), 
    g_cost(true), h_cost(true), search_depth(0) { }
    

AStarPlanner::LocList AStarPlanner::run_astar(){
    // # Create start and end node
    M_Node start_node(m_start);
    ROS_INFO_STREAM("A* start_node " << start_node.position()(0)<<","<<start_node.position()(1)); 
    M_Node end_node(m_end);
    ROS_INFO_STREAM("A* end_node " << end_node.position()(0)<<","<<end_node.position()(1)); 
    if(start_node ==  end_node) return LocList();

    // # Initialize both open and closed list
    NodeList::iterator it; //for searching lists;

    NodeList open_list;
    NodeList closed_list;

    // cost constants
    double move_cost = 1; // cost of new neighbor;
    double diag_cost = 1.414; // cost of moving diagonally
    double wall_cost = 3; // cost of being next to a wall
    double h_eps = 0.001; // h-cost tie breaker

    // # Add the start node
    open_list.push_back(start_node);
    // it = find(open_list,start_node);
    // if (it == open_list.end()) {
    //     ROS_ERROR("Cannot find start node on open list!!!");
    //     abort();
    // }

    // # Loop until you find the end
    // ROS_INFO_STREAM("😅  Loop until you find found the goal"); 
    int loop_count =0;
    while(open_list.size() > 0)
    {
        if (loop_flag) {
           ROS_INFO_STREAM("\nloop_count: "<< loop_count<<"\r"); 
        }
        loop_count ++;

        // Check if exceeded max depth
        if ( search_depth > 0 && loop_count>search_depth )
        {
            ROS_WARN("A* max search depth reached - %d",search_depth);
            ROS_INFO("  Start node: (%d,%d), Goal node: (%d,%d)",m_start(0),m_start(1),m_end(0),m_end(1));
            dump_list("  open_list",open_list);
            it = find(open_list,end_node);
            if (it != open_list.end()) ROS_INFO("  goal_node found in the open_list!");
            dump_list("  closed_list",closed_list);
            it = find(closed_list,end_node);
            if (it != open_list.end()) ROS_INFO("  goal_node not found in the closed_list!");
            ROS_INFO("\n");
            return LocList(); // empty list
        }

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

        // Debug check - see if still in the open list
        it = find(open_list,current_node);
        while(it != open_list.end()) {
            ROS_WARN("Node (%d,%d) removed from open list, but (another copy) is still there! The other will be removed.",current_node.position()(0),current_node.position()(1));
            ROS_WARN("Current f-cost=%f, other copy f-cost=%f",current_node.f(),it->f());
            open_list.erase(open_list.begin()+current_index);
            it = find(open_list,current_node);
        }


        it = find(closed_list,current_node);
        if (it != closed_list.end()) {
            ROS_WARN("Node (%d,%d) adding to closed list, but (another copy) is already in there!",current_node.position()(0),current_node.position()(1));
            ROS_WARN("Current f-cost=%f, other copy f-cost=%f",current_node.f(),it->f());
        }

        closed_list.push_back(current_node);
        // The position on the closed list is used as an 'id' or
        // pointer to the node. A child of this node can trace back
        // to its parent by way of the id into the closed_list.
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
        for( Vector2i new_position : LocList{Vector2i(0,-1),Vector2i(0,1),Vector2i(-1,0),Vector2i(1,0),
                                    Vector2i(-1,-1),Vector2i(-1,1),Vector2i(1,-1),Vector2i(1,1)})  //Adjacent squares
        {
            // # Get node position
            Vector2i node_position = (current_node.position() + new_position);

            // # Make sure within maze
            if (!valid_position(node_position)) continue;

            // # Make sure walkable terrain - can't use neighbors that are obstacles
            if (isWall(node_position)) continue;

            // # Create new node
            M_Node neighbor = M_Node(node_position,current_node.id());

            // if already seen (closed_list), skip
            if( find(closed_list,neighbor) != closed_list.end()) continue;

            // # Create the f, g, and h values
            if(g_cost) {
                // if neighbor is diagonal, add a cost.
                double new_g = current_node.g() + move_cost;
                if ((neighbor.position() - current_node.position()).squaredNorm() > 1) new_g += diag_cost;
                new_g += next_to_wall(neighbor.position()) * wall_cost; 
                neighbor.set_g( new_g );
            }
            
            if(h_cost) {
                // manhattan
                double dx = abs(neighbor.position()(0) - end_node.position()(0) );
                double dy = abs(neighbor.position()(1) - end_node.position()(1) );
                
                double new_h = move_cost * (dx +dy);
                // new_h +=  (diag_cost - 2 * move_cost) * min(dx,dy);
                new_h +=  diag_cost * sqrt(dx*dx + dy*dy);

                new_h *= 0.001;
                neighbor.set_h( new_h );
            }
            
            // # Create the f, g, and h values bassed on the use flags
            
            // http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#diagonal-distance
            
            neighbor.set_f( neighbor.g() + neighbor.h() );

            // Check if child is in open_list
            // if so, skip, unless g-cost is smaller
            it = find(open_list,neighbor);
            if (it != open_list.end()) {
                if (neighbor.g() >= it->g()) continue;
                ROS_INFO("Adding (%d,%d) to open_list, even though it's already there",neighbor.position()(0),neighbor.position()(1));
                ROS_INFO("   because new one has better g-cost:%d vs old-cost:%d",neighbor.g(),it->g());
            }

            // Add to open list
            open_list.push_back(neighbor);
        }
    } 

    // open set is empty, but goal never reached
    // failure
    // return empty path
    ROS_ERROR("A* failed to find goal. The open_list of nodes to explore is empty!");
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
        // ROS_INFO_STREAM("  find: item: " << m->position()(0)<<","<<m->position()(1)<<"  compared to "<<node.position()(0)<<","<<node.position()(1) ); 
        if (*m == node) { return m; }
    }
    return list.end();
}

void AStarPlanner::dump_list(const char* msg, AStarPlanner::NodeList &nl)
{
    ROS_INFO("%s",msg);
    for( NodeList::iterator it = nl.begin();
         it!=nl.end();
         it++)
    {
        ROS_INFO_STREAM("   item: ("<<it->position()(0)<<","<<it->position()(1)<<")");
    }
}

bool AStarPlanner::isWall(Vector2i& node_position)
{
    return m_maze(node_position(0),node_position(1)) != 0;
}

int AStarPlanner::next_to_wall(Vector2i position)
{
    int cnt = 0;
    for( Vector2i new_position : LocList{Vector2i(0,-1),Vector2i(0,1),Vector2i(-1,0),Vector2i(1,0)})
    {
        new_position = new_position + position;
        if (!valid_position(new_position)) continue;
        if (isWall(new_position)) cnt++;
    }
    return cnt;
}

bool AStarPlanner::valid_position(Vector2i & node_position)
{
    if (node_position(0) > (m_maze.rows()-1) || node_position(0) < 0 || node_position(1) > (m_maze.cols() -1) || node_position(1) < 0)
        return false;
    return true;
}