#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <string>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <utility>
#include <tuple>
#include <cmath>
using namespace std;


                                                                                      /* =============================================== CONSTANTS =============================================== */
const int N = 3; // size of the board (the board's dimension)
const int NN = 9; // total count of cells on the board


                                                                                    /* =============================================== FUNDAMENTAL DATA STRUCTURES =============================================== */
struct state{ // start of State struct definition

    array<array<int,N>, N> board; // 2D matrix to represent the current state of the board [row][column]

    // location of the blank cell on the board as a pair (r,c)
       // the blank's value is denoted as `0`
    int blank_s_row; 
    int blank_s_col;
      

}; // end of State struct definition

struct Node{ // start of Node struct definition
    
    state s; // current state of the board
    int g;   // path cost --- cost from the initial state to the current state
    int h;   // heuristic cost --- estimated cost from the current state to the goal state
    int f;   // total cost --- f = g + h
    char move; // the last immediate move that led to the current state (L, R, U, D)
    const Node* parent; // pointer to the parent node of this current node in the search tree
    
}; // end of Node struct definition




                                                                        /* =============================================== HEURISTICS =============================================== */

struct goal_positions{ // start of goal_positions struct definition

    /* 
       the purpose of this struct is to store the goal positions (r,c) for each tile in the board so we can get the (r,c) of any tile quickly instead of having to iterate through the board every time we need an (r,c)
       the array itself needs to index up to NN-1 but the values within the array will be from 0 to N-1 representing r and c

       example: goal_positions.row_positions[8] = 2 means that the tile with value 8 is in row 2 of the board
                goal_positions.col_positions[8] = 1 means that the tile with value 8 is in column 1 of the board
    */

    array<int, NN> row_positions;
    array<int, NN> col_positions;
}; // end of goal_positions struct definition

static int heuristic_h1(const array<array<int,N>, N>& current_board, const goal_positions& gps){ // start of heuristic_h1 function definition
                                                                           // the h1 heuristic is the Manhattan Distance heuristic

        int total_manh_dist = 0; // `total` because we are including the Manhattan Distances of ALL tiles in the board
        
        // iterate through each cell in the board
        for(int each_row = 0; each_row < N; ++each_row){
            for(int each_col = 0; each_col < N; ++each_col){
                
                int tile_value = current_board[each_row][each_col]; // grab the value of the current tile
                if(tile_value == 0) continue; // skip the blank cell
                total_manh_dist += ( abs(each_row - gps.row_positions[tile_value]) + abs(each_col - gps.col_positions[tile_value]) );

            }
        }
        // done iterating through each cell in the board

        return total_manh_dist;

} // end of heuristic_h1 function definition








static int wrong_ordering_counter(const vector<int>& tiles_goal_line){ // start of wrong_ordering_counter function definition

    /*
      we look at each position

      then we look at all the positions after the current positions

      if the value at the current position is greater than the value at the position after the current position, we have a wrong ordering
         this is like we have a tile who's goal position is 2 but on the board it comes before a tile who's goal position is 1
    
         so we increment the wrong ordering count
    */
    
    int wrong_ordering_count = 0;

    for(int i = 0; i < tiles_goal_line.size(); ++i){
        for(int j = i + 1; j < tiles_goal_line.size(); ++j){if(tiles_goal_line[i] > tiles_goal_line[j]){++wrong_ordering_count;}}
    }
    return wrong_ordering_count;

} // end of wrong_ordering_counter function definition


static int linear_conflicts_counter(const array<array<int,N>, N>& current_board, const goal_positions& gps){ // start of linear_conflicts function definition
    
    /* 

    we go through each line (line being a row or a column) ---- line pass

    we only consider (collect up) tiles that are in their correct (relative to the goal positions) line
       we collect up their goal position

    feed the collection (of goal positions) into a function that counts the number of wrong orderings in the collection
        // KEY: this function count pairs of goal positions that are in the wrong order such as <2,1> would be a wrong order because 2 is larger than 1
    
    */

    int total_linear_conflicts = 0;

    // row pass
    for(int each_row = 0; each_row < N; ++each_row){
        
        vector<int> tiles_goal_col; // collection of goal p

        for(int each_col = 0; each_col < N; ++each_col){
            int tile_value = current_board[each_row][each_col]; // grab the value of the current tile
            if(tile_value == 0){continue;} // skip the blank cell

            if(gps.row_positions[tile_value] == each_row){tiles_goal_col.push_back(gps.col_positions[tile_value]);} // only consider tiles that are in their correct row
                                                                                                                       // push this tile's goal column into the collection

        }
        
        total_linear_conflicts += wrong_ordering_counter(tiles_goal_col); // pass the collection into the function that counts the number of wrong orderings in the collection

    }
    // end of row pass


    // column pass
    for(int each_col = 0; each_col < N; ++each_col){
        
        vector<int> tiles_goal_row;

        for(int each_row = 0; each_row < N; ++each_row){
            int tile_value = current_board[each_row][each_col]; // grab the value of the current tile
            if(tile_value == 0){continue;} // skip the blank cell

            if(gps.col_positions[tile_value] == each_col){tiles_goal_row.push_back(gps.row_positions[tile_value]);} // only consider tiles that are in their correct column
                                                                                                                       // push this tile's goal row into the collection
        }

        total_linear_conflicts += wrong_ordering_counter(tiles_goal_row); // pass the collection into the function that counts the number of wrong orderings in the collection

    }
    // end of column pass

    return total_linear_conflicts;

} // end of linear_conflicts function definition


static int heuristic_h2(const array<array<int,N>, N>& current_board, const goal_positions& gps){ // start of heuristic_h2 function definition
                                                                                                               // the h2 heuristic is the sum of the h1 heuristic and 2 times the number of linear conflicts
    int h1 = heuristic_h1(current_board, gps);
    int linear_conflict_count = linear_conflicts_counter(current_board, gps);

    return h1 + 2 * linear_conflict_count;
} // end of heuristic_h2 function definition



                                                     /* =============================================== A* SEARCH ALGORITHM =============================================== */


static string board_to_string(const array<array<int,N>, N>& board){ // start of board_to_string function definition
    
    /* converts the board state to a string representation of the board */

    string board_str;

    for(int each_row = 0; each_row < N; ++each_row){
        
        for(int each_col = 0; each_col < N; ++each_col){

            board_str += to_string(board[each_row][each_col]);
        }


    }

    return board_str;

} // end of board_to_string function definition



static bool is_goal_state(const state& current_state, const state& gs){ // start of is_goal_state function definition
    
    /* checks if the current board state is the goal state */

    for(int each_row = 0; each_row < N; ++each_row){
        
        for(int each_col = 0; each_col < N; ++each_col){
            
            if(current_state.board[each_row][each_col] != gs.board[each_row][each_col]){return false;}

        }


    }
    // if we get here, then we found a match for every tile in the current board state with the goal board state

    return true; 

} // end of is_goal_state function definition




static vector<pair<char, state>> generate_children(const state& current_state){ // start of generate_children function definition
     
    /* the purpose of this function is to generate all successor (aka child) states from the current state */

    vector<pair<char, state>> children; // a collection to hold all of the actions and the resulting states

    // all the possible actions that can be taken from the current state
       // {dr, dc, action char} --- dr is the change in row, dc is the change in column, action char is the action that was taken
    vector<tuple<int, int, char>> actions = {
        {0, -1, 'L'},
        {0, 1, 'R'},
        {-1, 0, 'U'},
        {1, 0, 'D'}
    };


    for(const auto& [dr, dc, action] : actions){ // start of processing each action

        int blank_row = current_state.blank_s_row;
        int blank_col = current_state.blank_s_col;

        int new_blank_row = blank_row + dr;
        int new_blank_col = blank_col + dc;

        if(new_blank_row < 0 || new_blank_row >= N || new_blank_col < 0 || new_blank_col >= N){continue;} // skip if the new position is out of bounds
        

        // reaching here means the new position of the blank cell is within the bounds of the board


        // create a new state that is the result of the action
        state new_state = current_state;
        swap(new_state.board[blank_row][blank_col], new_state.board[new_blank_row][new_blank_col]); // swap the blank cell with the tile in the new position
                                                                                                       // simulate the move that was made
        // update the blank cell's position in the new state
        new_state.blank_s_row = new_blank_row;
        new_state.blank_s_col = new_blank_col;


        children.push_back({action, new_state}); // add the new state to the children vector

    } // end of processing each action

    return children;


} // end of generate_children function definition




static const Node* a_star_search(const state& initial_state, const state& goal_state, int heuristic_choice, long long& nodes_generated){ // start of a_star_search function definition
    

    // record the goal positions for each tile
    goal_positions gps;
    for(int each_row = 0; each_row < N; ++each_row){

        for(int each_col = 0; each_col < N; ++each_col){
            
            int tile_value = goal_state.board[each_row][each_col];

            gps.row_positions[tile_value] = each_row;
            gps.col_positions[tile_value] = each_col;

        }
    }
    // end of recording the goal positions for each tile



    auto node_comparator = [](const Node* n1, const Node* n2){return n1->f > n2->f;}; // `greater than` for min heap
    priority_queue<Node*, vector<Node*>, decltype(node_comparator)> frontier(node_comparator); // INITIALIZE FRONTIER --- FRONTIER is a priority queue of nodes sorted by the f value of the nodes


    // WE NEED THESE SINCE WE'RE DOING A GRAPH SEARCH SO THESE HELP US TRACK REPEATS
    unordered_set<string> explored; // set to keep track of explored state using a string representation of the board
    unordered_map<string, Node*> frontier_map; // hash map to keep track of the board state and the node that represents it in the frontier



    
    // creating the root node
    Node* root_node = new Node();
    root_node->s = initial_state;
    root_node-> g = 0;
    root_node->h = (heuristic_choice == 1) ? heuristic_h1(initial_state.board, gps) : heuristic_h2(initial_state.board, gps); // heuristic depends on what the user chose
    root_node->f = root_node->g + root_node->h;
    root_node->move = '\0'; 
    root_node->parent = nullptr;
    
    // adding the root node (aka initial node) to the frontier --- get things started
    frontier.push(root_node);
    frontier_map[board_to_string(initial_state.board)] = root_node;
    ++nodes_generated;



    // CORE A* SEARCH LOOP
    while(!frontier.empty()){ // start of the core A* search loop

        // grab the node with the lowest f value from the frontier
        Node* current_node = frontier.top(); frontier.pop();

        string current_state_str = board_to_string(current_node->s.board);
        frontier_map.erase(current_state_str); // remove the current node from the frontier map

        
        if(is_goal_state(current_node->s, goal_state)){return current_node;} // GOAL TEST --- did we find the goal state?

        
        explored.insert(current_state_str); // add this current node to the explored set --- marking it as explored

        vector<pair<char, state>> children = generate_children(current_node->s);// GENERATE CHILDREN NODES (AKA NEXT ACTION NODES)

        for(const auto& [move, next_state] : children){ // start of processing each child node

            string child_state_str = board_to_string(next_state.board);

            if(explored.count(child_state_str) > 0){continue;} // skip this child node's state if it has already been explored
            
            // computing child's costs (g, h and ultimately f)
            int child_g = current_node->g + 1; // path cost --- cost from the initial state to the current state
            int child_h = (heuristic_choice == 1) ? heuristic_h1(next_state.board, gps) : heuristic_h2(next_state.board, gps);
            int child_f = child_g + child_h;

            auto frontier_map_iter = frontier_map.find(child_state_str);
            if(frontier_map_iter!= frontier_map.end()){

                // reaching here means that this child node's state is already in the frontier
                // we need to check if the child node's f value is less than the f value of the node in the frontier with the same state
                
                Node* existing_node = frontier_map_iter->second;
                if(child_f < existing_node->f){
                    // update the existing node in the frontier with better costs
                    existing_node->g = child_g;
                    existing_node->f = child_f;
                    existing_node->move = move;
                    existing_node->parent = current_node;
                }
            }
            else{
               
                // reaching here means that this child node's state is NOT in the frontier yet
                
                // creating the child node
                Node* child_node = new Node();
                child_node->s = next_state;
                child_node->g = child_g;
                child_node->h = child_h;
                child_node->f = child_f;
                child_node->move = move;
                child_node->parent = current_node;


                // adding the child node to the frontier
                frontier.push(child_node);
                frontier_map[child_state_str] = child_node;
                ++nodes_generated;

            }

        } // end of processing each child node

    } // end of the core A* search loop
    
    return nullptr; // no solution found

} // end of a_star_search function definition















/* =============================================== MAIN =============================================== */


static void print_board(const array<array<int,N>, N>& board){ // start of print_board function definition
    
    /* print the board in the required format */

    for(int each_row = 0; each_row < N; ++each_row){
        for(int each_col = 0; each_col < N; ++each_col){
            cout << board[each_row][each_col];
        }
        cout << '\n';
    }
} // end of print_board function definition

static void reconstruct_solution(const Node* final_node, vector<char>& actions, vector<int>& fvalues){ // start of reconstruct_solution function definition
     
    /* reconstruct the solution path by manipulating the `actions` and `fvalues` collections */

    const Node* current_node = final_node;

    while(current_node->parent != nullptr){
        actions.push_back(current_node->move); // reminder: `move` is the action that led to the current node from its parent node
        fvalues.push_back(current_node->f);   // put the f values of nodes on the solution path into the collection
        current_node = current_node->parent; // simulate going up the tree to the parent node
    }
    // when current_node is the root node, we don't want to put the action into the collection but we do want to put the f value into the collection
    fvalues.push_back(current_node->f);   // put the f value of the root node into the collection



    // reaching here, our solution path collections are in reverse order

    reverse(actions.begin(), actions.end()); // correct order is root --> n1 --> n2 --> ... --> final_node
    reverse(fvalues.begin(), fvalues.end());

} // end of reconstruct_solution function definition



static bool read_in(const string& filename, state& initial_state, state& goal_state){ // start of read_in function definition
    
    ifstream input_file(filename); // open the input file

    if(!input_file){cerr << "Failed to open the input file. Please retry." << endl; return false;}

    
    // read in the initial state
    for(int each_row = 0; each_row < N; ++each_row){

        for(int each_col = 0; each_col < N; ++each_col){
            
            input_file >> initial_state.board[each_row][each_col]; // READ INTO THE INITIAL STATE BOARD
            
            // record the location of the blank (represented as `0`)
            if(initial_state.board[each_row][each_col] == 0){
                initial_state.blank_s_row = each_row;
                initial_state.blank_s_col = each_col;
            }

        }
    }


    // the blank line in the input file is ignored by the >> operator


    // read in the goal state
    for(int each_row = 0; each_row < N; ++each_row){
        
        for(int each_col = 0; each_col < N; ++each_col){

            input_file >> goal_state.board[each_row][each_col]; // READ INTO THE GOAL STATE BOARD

            // record the location of the blank (represented as `0`)
            if(goal_state.board[each_row][each_col] == 0){
                goal_state.blank_s_row = each_row;
                goal_state.blank_s_col = each_col;
            }
        }
    }


    // done reading in the input file
    input_file.close();
    return true;

} // end of read_in function definition




static void create_output(const array<array<int,N>, N>& start_board, const array<array<int,N>, N>& goal_board, int depth, long long nodes_generated, vector<char>& actions, vector<int>& fvalues){ // start of create_output function definition
    
    /* create the output files in the required format */

    print_board(start_board); // lines 1-3 of the output file
    cout << '\n';
    print_board(goal_board); // lines 5-7 of the output file
    cout << '\n';
    cout << depth << '\n'; // line 9 of the output file
    cout << nodes_generated << '\n'; // line 10 of the output file
    for(int i = 0; i < actions.size(); ++i){cout << actions[i] << ' ';} cout << '\n'; // lines 11 of the output file
    for(int i = 0; i < fvalues.size(); ++i){cout << fvalues[i] << ' ';} // lines 12 of the output file

} // end of create_output function definition






int main(int argc, char* argv[]){ // start of main function
                                      // WHERE ALL THE ACTION OCCURS

    // checking correct command line arguments
        // the source code
        // the input file
        // the heuristic choice
    if(argc != 3){
        cerr << "Wrong number of arguments. Please enter the input file and the heuristic choice." << endl;
        return 1;
    }


    // grab the command line arguments
       // there should be two arguments: the input file and the heuristic choice
    string input_file = argv[1];
    int h = stoi(argv[2]);
    if(h != 1 && h != 2){
        cerr << "Invalid heuristic choice. Please enter 1 or 2." << endl;
        return 1;
    }

    // read in 
    state initial_state{}, goal_state{}; // initialze empty `state` instances
    if(!read_in(input_file, initial_state, goal_state)){
        cerr << "Read in failed. Please retry." << endl;
        return 1;
    }



    // reaching here means we have successfuly read-in


    // run the A* search algorithm
    long long nodes_generated = 0;
    const Node* solution_node = a_star_search(initial_state, goal_state, h, nodes_generated); // RUNNING THE A* SEARCH ALGORITHM


    

    /* preparation for the output files */

    vector<char> actions;
    vector<int> fvalues;
    reconstruct_solution(solution_node, actions, fvalues); // RECONSTRUCT THE SOLUTION PATH


    int depth = static_cast<int>(actions.size()); // the depth of the solution path is the number of actions taken

    create_output(initial_state.board, goal_state.board, depth, nodes_generated, actions, fvalues); // GENERATE THE OUTPUT FILES

    return 0;


} // end of main function





