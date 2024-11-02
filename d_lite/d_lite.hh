#ifndef D_LITE_H
#define D_LITE_H

#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <stdio.h>
#include <ext/hash_map>

using namespace std;
using namespace __gnu_cxx; // hash maps

// small margin of error for our decimals
const double K_TOLERANCE = 0.00001;
// hash prime - must be > max_x, less than (size_t / max_y) + max_x
const size_t HASH_PRIME = 349121;
// used for 'untraversable' states (wall, huge boulders, etc.)
const double INF = numeric_limits<double>::infinity();

// graph nodes
class state {
    public:
        // pos
        int x;
        int y;
        // key values, used for prioqueue ordering
        // k.first is first comparision, k.second is tiebreaker
        pair<double,double> k;
        // k1(s) = min(g(s), rhs(s) + h(s_start, s)) + k_m
        // k2(s) = min(g(s), rhs(s))
        // see everything explained below
        // h is the huersitic estimate

        // defining operations on g nodes (using in prioqueue and hashmap)
        // equal states = both nodes same
        bool operator == (const state &s2) const {
            return ((x == s2.x) && (y == s2.y));
        }
        // not equal states = at least one node diff
        bool operator != (const state &s2) const {
            return ((x != s2.x) || (y != s2.y));
        }

        // >, <=, < operators, using K_TOLERANCE for (more accurate) decimal comps
        bool operator > (const state &s2) const {
            if (k.first-K_TOLERANCE > s2.k.first) return true;
            else if (k.first < s2.k.first-K_TOLERANCE) return false;
            return k.second > s2.k.second;
        }
        bool operator <= (const state &s2) const {
            if (k.first < s2.k.first) return true;
            else if (k.first > s2.k.first) return false;
            return k.second < s2.k.second + K_TOLERANCE;
        }
        bool operator < (const state &s2) const {
            if (k.first + K_TOLERANCE < s2.k.first) return true;
            else if (k.first - K_TOLERANCE > s2.k.first) return false;
            return k.second < s2.k.second;
        }

};

struct ipoint2 {
    int x,y;
};


struct cellInfo {

    double g;    // curr cost estimate from start to s
    double rhs;  // one-step lookahead cost estimate
    double cost; // cost of traversing the cell

};

// hash map to efficiently store states
class state_hash {
    public:
        size_t operator()(const state &s) const {
            return s.x + HASH_PRIME * s.y;
        }
};

// prioqueue for node calcs 
typedef priority_queue<state, vector<state>, greater<state> > dsl_pq;
// hashes for state info
typedef hash_map<state, cellInfo, state_hash, equal_to<state> > dsl_ch;
typedef hash_map<state, float, state_hash, equal_to<state> > dsl_oh;


// D* lite algo itself
class Dstar_lite {

    // these should be all we need to interface with and run D* lite
    public:

        Dstar_lite();
        void init(int sX, int sY, int gX, int gY);  // start and goal pos
        void updateCell(int x, int y, double val);  // pos to update
        void updateStart(int x, int y);             // new start pos
        void updateGoal(int x, int y);              // new goal pos
        bool replan();                              // calc new path
        
        list<state> getPath();  // ret curr path

    private:

        list<state> path;       // curr path

        double C1;       // init vals of every node 
        double k_m;      // key modifer, for changes in start state (robot movement)
        state s_start, s_goal, s_last;
        int maxSteps;    // to prevent infinite loops

        dsl_pq prio_queue;
        dsl_ch cell_info_hash;
        dsl_oh cell_open_hash;

        // intrenal functions that we need, explained in d_lite.cc
        bool   close(double x, double y);
        void   makeNewCell(state u);
        double getG(state u);
        double getRHS(state u);
        void   setG(state u, double g);
        double setRHS(state u, double rhs);
        double eightCondist(state a, state b);
        int    computeShortestPath(); // path computation
        void   updateVertex(state u);
        void   insert(state u);
        void   remove(state u);
        double trueDist(state a, state b);
        double heuristic(state a, state b); // heursitc function
        state  calculateKey(state u); // key value
        void   getSucc(state u, list<state> &s); // next nodes
        void   getPred(state u, list<state> &s); // prev nodes
        double cost(state a, state b);
        bool   occupied(state u);   // inaccesible 
        bool   isValid(state u);
        float  keyHashCode(state u);

};


#endif