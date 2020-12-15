    #include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
class AStarPlanner
{ //https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/AStar/a_star.py

private:
    /* data */
    class M_Node
    {
    private:
        M_Node *m_parent = nullptr;
        Vector2i m_position;
        int m_g;
        double m_h;
        double m_f;
    public:
        M_Node* parent(){return m_parent;};
        const Vector2i position(){return m_position;};
        const int g(){return m_g;};
        const double h(){return m_h;};
        const double f(){return m_f;};
        void set_g(int v) { m_g = v; }
        void set_h(double v) { m_h = v; }
        void set_f(double v) { m_f = v; }
        M_Node() : m_parent(nullptr),m_g(0),m_h(0),m_f(0) {}
        M_Node(M_Node *parent, Vector2i position)
        : m_parent(parent), m_position(position),m_g(0),m_h(0),m_f(0){
        }
        ~M_Node(){}

        friend bool operator==( M_Node&, M_Node&);
        friend bool operator==(const M_Node&,const M_Node&);

    };


    MatrixXi m_maze;
    Vector2i m_start;
    Vector2i m_end;


    
public:
    typedef vector<Vector2i,Eigen::aligned_allocator<Eigen::Vector2i>> LocList;
    typedef vector<M_Node,Eigen::aligned_allocator<Eigen::Vector2i>> NodeList;
    
    // AStarPlanner();
    AStarPlanner(Eigen::MatrixXi &map,Vector2i &start,Vector2i &end);
    ~AStarPlanner();
    
    LocList run_astar();
    M_Node* find( NodeList&, M_Node&);

    friend bool operator==( AStarPlanner::M_Node&, AStarPlanner::M_Node&);
    friend bool operator==(const AStarPlanner::M_Node&, const AStarPlanner::M_Node&);
};

inline bool operator==( const Vector2i&lhs, const Vector2i&rhs)
    // { return lhs.x() == rhs.x() && lhs.y() == rhs.y(); }
    { return lhs(0) == rhs(0) && lhs(1) == rhs(1); }

inline bool operator==( AStarPlanner::M_Node&lhs, AStarPlanner::M_Node&rhs)
    { return lhs.position() == rhs.position(); }
// inline bool operator==(const AStarPlanner::M_Node&lhs, const AStarPlanner::M_Node&rhs)
//     { return lhs.position() == rhs.position(); }

// AStarPlanner::AStarPlanner(){}
AStarPlanner::AStarPlanner(Eigen::MatrixXi &map,
                            Vector2i &start,
                            Vector2i &end)
    :m_maze(map),m_start(start), m_end(end){ }

AStarPlanner::LocList AStarPlanner::run_astar(){
    // # Create start and end node
    // start_node = Node(None, start)
    M_Node start_node(nullptr,m_start);
    ROS_INFO_STREAM("start_node " << start_node.position()(0)<<","<<start_node.position()(1)); 
    // start_node.g = start_node.h = start_node.f = 0
    // end_node = Node(None, end)
    M_Node end_node(nullptr,m_end);
    if(start_node ==  end_node) return LocList();

    ROS_INFO_STREAM("end_node " << end_node.position()(0)<<","<<end_node.position()(1)); 

    // end_node.g = end_node.h = end_node.f = 0

    // # Initialize both open and closed list
    // open_list = []
    // closed_list = []
    NodeList open_list;
    NodeList closed_list;

    // # Add the start node
    // open_list.append(start_node)
    open_list.push_back(start_node);

    // # Loop until you find the end
    // while len(open_list) > 0:
    ROS_INFO_STREAM("😅  Loop until you find found the goal"); 
    int loop_count =0;
    while(open_list.size() > 0)
    {
        ROS_INFO_STREAM("\nloop_count: "<< loop_count<<"\r"); 
        loop_count ++;
        // # Get the current node
        // current_node = open_list[0]
        M_Node current_node = open_list[0];
        if( &current_node == current_node.parent() ){
            ROS_ERROR("\n\n A* FAILED: &current_node == current_node.parent() \n\n");
            ROS_ERROR_STREAM("parent: " << current_node.parent()->position()(0)<<","<<current_node.parent()->position()(1)); 
            ROS_ERROR_STREAM("current_node: " << current_node.position()(0)<<","<<current_node.position()(1)); 
            // abort();
        }
        // current_index = 0
        int current_index = 0;
        // for index, item in enumerate(open_list):
        for(int index=0; index<open_list.size(); index++)
        {
            M_Node item = open_list[index];
            // if item.f < current_node.f:
            if (item.f() < current_node.f())
            {
                // current_node = item
                if(current_node ==  item && start_node == item) 
                { ROS_INFO_STREAM(" #######your not adding yourself");  continue;}
                current_node = item;
                // current_index = index
                current_index = index;
            }
        }
        ROS_INFO_STREAM("Moveing to closed_list.push_back(current_node:" << current_node.position()(0)<<","<<current_node.position()(1)); 

        // # Pop current off open list, add to closed list
        // open_list.pop(current_index)
        ROS_INFO_STREAM("current_index: "<< current_index ); 
        ROS_INFO_STREAM("before erase open_list.size()"<< open_list.size() ); 
        
        ROS_INFO_STREAM("erase: ( "<< (open_list.begin()+current_index)->position()(0)<<","<<(open_list.begin()+current_index)->position()(1)<<" )" ); 

        open_list.erase(open_list.begin()+current_index);
        ROS_INFO_STREAM("after erase open_list.size()"<< open_list.size() ); 

        // closed_list.append(current_node)
        closed_list.push_back(current_node);

        // # Found the goal
        // if current_node == end_node:
        if (current_node == end_node)
        {
            ROS_INFO_STREAM("Starting -> Found the goal");
            // path = []
            LocList path;
            // current = current_node
            M_Node *current = &current_node;
            // while current is not None:
            while(current)
            {
                ROS_INFO_STREAM("Found list current:" << current->position()(0)<<","<<current->position()(1)); 

                // path.append(current.position)
                path.push_back(current->position());
                // current = current.parent
                if(current == current->parent())
                {
                    ROS_ERROR_STREAM("parent: " << current->parent()->position()(0)<<","<<current->parent()->position()(1)); 
                    ROS_ERROR_STREAM("current_node: " << current->position()(0)<<","<<current->position()(1)); 
                    ROS_ERROR("\n\nA* FAILED:  current == current->parent() \n\n");
                    return LocList();
                    // abort();
                }
                current = current->parent();
                ROS_INFO_STREAM(" current = current->parent(); "); 
            }
            // return path[::-1] # Return reversed path
            reverse(path.begin(),path.end());
            ROS_INFO_STREAM(" Found the goal END"); 
            return path;
        }

        // # Generate children
        // children = []
        NodeList children;
        // for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares
        for( Vector2i new_position : LocList{Vector2i(0,-1),Vector2i(0,1),Vector2i(-1,0),Vector2i(1,0),
                                    Vector2i(-1,-1),Vector2i(-1,1),Vector2i(1,-1),Vector2i(1,1)})  //Adjacent squares
        {
            // # Get node position
            // node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])
            Vector2i node_position = (current_node.position() + new_position);

            // # Make sure within range
            // if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                // continue
            if (node_position[0] > (m_maze.rows()-1) or node_position[0] < 0 or node_position[1] > (m_maze.cols() -1) or node_position[1] < 0)
                continue;

            // # Make sure walkable terrain
            // if maze[node_position[0]][node_position[1]] != 0:
                // continue
            if (m_maze(node_position[0],node_position[1]) != 0)
                continue;

            // # Create new node
            // new_node = Node(current_node, node_position)

            M_Node new_node = M_Node(&current_node, node_position);

            //Check if your not adding yourself
            if(current_node ==  new_node && start_node == new_node) 
            { ROS_INFO_STREAM("😅  Few dodged that one, your not adding yourself");  continue;}


            ROS_INFO_STREAM("parent: " << new_node.parent()->position()(0)<<","<<new_node.parent()->position()(1) ); 
            ROS_INFO_STREAM("new_node: " << new_node.position()(0)<<","<<new_node.position()(1)); 

            if( &new_node == new_node.parent() ){
                ROS_WARN("\n\n A* FAILED: &current_node == current_node.parent() \n\n");
                ROS_WARN_STREAM("parent: " << new_node.parent()->position()(0)<<","<<new_node.parent()->position()(1)); 
                ROS_WARN_STREAM("current_node: " << new_node.position()(0)<<","<<new_node.position()(1)); 
                // abort();
            }
 


            // # Append
            // children.append(new_node)
            children.push_back(new_node);
            ROS_INFO_STREAM(" children.append(new_node)"); 
        }

        // # Loop through children
        // for child in children:
        for( M_Node child : children)
        {

            // # Child is on the closed list
            // for closed_child in closed_list:
                // if child == closed_child:
                    // continue
            // NodeList::iterator it = find(closed_list.begin(),closed_list.end(),child);
            // if (it != closed_list.end()) continue;
            // M_Node closed_child;
            ROS_INFO_STREAM("Checking if find(closed_list, child() " << child.position()(0)<<","<<child.position()(1)); 

            if (find(closed_list, child)) continue;

            // # Create the f, g, and h values
            // child.g = current_node.g + 1
            child.set_g(current_node.g()+1);
            // child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.set_h( (child.position() - end_node.position()).squaredNorm() );
            // child.f = child.g + child.h
            child.set_f( child.g() + child.h() );

            // # Child is already in the open list
            // for open_node in open_list:
                // if child == open_node and child.g > open_node.g:
                    // continue
            // it = find(open_list.begin(),open_list.end(),child);
            // if (it != open_list.end() && child.g() > it->g()) continue;
            // M_Node open_node = find(open_list,child);
            M_Node *open_node = find(open_list,child);
            if (open_node  && child.g() > open_node->g()) continue;

            //Check if your not adding yourself
            if(current_node ==  child && start_node == *open_node) 
            { ROS_INFO_STREAM("😅  Few dodged that one, your not adding yourself");  continue;}

            // # Add the child to the open list
            // open_list.append(child)
            open_list.push_back(child);
            ROS_INFO_STREAM("Add the \"child\" to the open_list ( " << child.position()(0)<<","<<child.position()(1) <<" )"); 
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

AStarPlanner::M_Node* AStarPlanner::find( NodeList& list, M_Node &node)
{   
    // for( M_Node m : list)
    M_Node* m = &*(list.begin());
    int i = 0;
    // ROS_INFO("find: while");
    while(list.begin()+i != list.end())
    {
        m = &*(list.begin()+i);
        // ROS_INFO_STREAM("  item: " << m->position()(0)<<","<<m->position()(1) ); 
        if (*m == node) {
            if( m->parent() != node.parent()){
                ROS_ERROR("\n m.parent() != node.parent()"); 
                ROS_ERROR_STREAM("m " << m->position()(0)<<","<<m->position()(1)); 
                // ROS_ERROR("node " << node.position()(0)<<","<<node.position()(1)); 


            }

            return m; 
        }
        i++;
    }
    return nullptr;
}
