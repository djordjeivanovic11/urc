#include "d_lite.hh"

// this stuff is so I can maybe vizualize it at some point
#ifdef USE_OPEN_GL
#ifdef MACOS
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#endif

/* void DstarLite::DstarLite 
* Set constraints
*/
DstarLite::DstarLite() {
    maxSteps = 4000000;  // steps before we give up - 4,000,000 cells, each is 1 meter 4 km^2 total
    C1       = 1;        // cost of an unknown cell
}


/* float DstarLite::keyHashCode(state s)
* Returns the key hash code for state s, used
* to compare updated states
*/
float DstarLite::keyHashCode(state s) {
    return (float)(s.k.first + HASH_PRIME*s.k.second);
}

/* bool DstarLite::isValid(state s)
* True iff state s in on the open list
*/
bool DstarLite::isValid(state s) {

    dsl_oh::iterator c = cellOpenHash.find(s);
    if (c == cellOpenHash.end()) return false; // not in hash, return false
    if (!close(keyHashCode(s), c->second)) return false; // state in hash, but key is different
    return true;
}

/* void DstarLite::getPath() 
* returns the path
*/
list<state> DstarLite::getPath() {
    return path;
}

/* bool DstarLite::notTraversable(state u) 
* true iff u is occupied (cost = INF)
*/
bool DstarLite::notTraversable(state s) {
    dsl_ch::iterator c = cellInfoHash.find(s);

    if (c == cellInfoHash.end()) return false;
    return (c->second.cost == INF);
}

/* void DstarLite::init(int sX, int sY, int gX ,int gY) 
* inits dstar with start and goal conditions
*/
void DstarLite::init(int sX, int sY, int gX ,int gY) {

    // clears our current data structures
    cellInfoHash.clear();
    path.clear();
    cellOpenHash.clear();
    while(!prioQueueHash.empty()) prioQueueHash.pop();

    k_m = 0;
    
    s_start.x, s_start.y = sX, sY;
    s_goal.x, s_goal.y = gX, gY;

    // stuff below is per the algorithm in the paper
    // update goal info
    cellInfo temp;
    temp.g = temp.rhs = 0;
    temp.cost = C1;
    cellInfoHash[s_goal] = temp;

    // update start info
    temp.g = temp.rhs = heuristic(s_start, s_goal);
    cellInfoHash[s_start] = temp;
    s_start = calculateKey(s_start);

    s_last = s_start;
}

/* void DstarLite::makeNewCell(state s) 
* adds a cell to the hash table if it is not already present
*/
void DstarLite::makeNewCell(state s) {

    if (cellInfoHash.find(s) != cellInfoHash.end()) return;

    cellInfo temp;
    temp.g = temp.rhs = heuristic(s, s_goal);
    temp.cost = C1;
    cellInfoHash[s] = temp;

}

/* double DstarLite::getG(state_s) 
* gets G value for s
*/
double DstarLite::getG(state s) {
    
    if (cellInfoHash.find(s) == cellInfoHash.end()) {
        return heuristic(s, s_goal);
    }
    return cellInfoHash[s].g;
}

/* double DstarLite::getRHS(state_s) 
* gets rhs value for s
*/
double DstarLite::getRHS(state s) {
    
    if (s == s_goal) return 0;

    if (cellInfoHash.find(s) == cellInfoHash.end()) {
        return heuristic(s, s_goal);
    }
    return cellInfoHash[s].rhs;
}

/* void DstarLite::setG(state_s, double g) 
* sets G value for s to g
*/
void DstarLite::setG(state s, double g) {
    
    makeNewCell(s);
    cellInfoHash[s].g = g;
}

/* void DstarLite::setRHS(state_s, double rhs) 
* sets rhs value for s to rhs
*/
void DstarLite::setRHS(state s, double rhs) {
    
    makeNewCell(s);
    cellInfoHash[s].rhs = rhs;
}

/* double DstarLite::eightDist(state a, state b)
* returns 8-way distance between a and b
*/
double DstarLite::eightDist(state a, state b) {
    
    double min = fabs(a.x - b.x);
    double max = fabs(a.y - b.y);

    if (min > max) {
        double temp = min;
        min = max;
        max = temp;
    }
            // diagonol movement //straight movevent
    return ((M_SQRT1_2-1.0)*min + max);
}

/* double DstarLite::trueDist(state a, state b)
* euclidean distance between two states
*/
double DstarLite::trueDist(state a, state b) {
    float x = a.x - b.x;
    float y = a.y - b.y;
    return sqrt(x*x + y*y);
}

/* int DstarLite::computeShortestPath()
* computes the shortest path from start to goal,
* per the paper
*/
int DstarLite::computeShortestPath() {
    list<state> s;
    list<state>::iterator i;

    if (prioQueueHash.empty()) return 1;

    int k = maxSteps;
    while ((!prioQueueHash.empty()) && 
            (prioQueueHash.top() < (s_start = calculateKey(s_start))) ||
            (getRHS(s_start) != getG(s_start))) {
        
        if (k-- == 0) {
            fprintf(stderr, "max steps\n");
            return -1;
        }

        state s;
        bool test = (getRHS(s_start) != getG(s_start));

        // get our next state with lazy removal
        while (true) {
            s = prioQueueHash.top();
            prioQueueHash.pop();

            // if state isn't valid, try again
            if (!isValid(s)) continue;
            if (!(s < s_start) && (!test)) return 2;
            break;
        }

        // no longer open, remove from open hash
        cellOpenHash.erase(cellOpenHash.find(s));

        state k_old = s;
        if (k_old < calculateKey(s)) { 
            insert(s);  // key is innnacurate, put it back in w/ updated key
        } else {
            double sRHS = getRHS(s);
            
            if (getG(s) > sRHS) { // estimate got better
                setG(s, sRHS);

                // update predicate verticies
                updatePredVerts(s);

            } else { // estimate got worse
                setG(s, INF);

                // update predicate verticies and s itself
                updatePredVerts(s);
                updateVertex(s);
            }

        }
    }
    return 0;
}

/* void DstarLite::updatePredVerts(state s)
* udates the pred verticies
*/
void DstarLite::updatePredVerts(state s) {
    list<state> pred;
    getPred(s, pred);

    for (state pS : pred) {
        updateVertex(pS);
    }
}

/* bool DstarLite::close(double x, double y)
* true if x y within K_TOLERANCE
*/
bool close(double x, double y) {
    if (isinf(x) && isinf(y)) return true;
    return abs(x-y) <= K_TOLERANCE;
}


/* void DstarLite::updateVertex(state s)
* updates vertex s, as per the paper
*/
// todo finish
void DstarLite::updateVertex(state s){
    if (s != s_goal) {
        double min_rhs = INF;
        double temp_rhs;
        list<state> succ;
        getSucc(s, succ);

        for (state sS : succ){
            temp_rhs = getG(sS) + cost(s, sS);
            if (temp_rhs < min_rhs) min_rhs = temp_rhs;
        }
        if (!close(getRHS(s), min_rhs)) setRHS(s, min_rhs);

    // todo i did this but i don't remember why
    } if (cellOpenHash.find(s) != cellOpenHash.end()) {
        cellOpenHash.erase(s);
        // will probably just make insert rmove it for us
    } if (!close(getG(s), getRHS(s))) {
        insert(s);
    }
}

/* void DstarLite::insert(state s)
* calculates key (maybe), inserts, and removes any existing options (maybe or I might leave them?)
*/
void DstarLite::insert(state s){

    dsl_oh::iterator curr = cellOpenHash.find(s);
    s = calculateKey(s);
    float csum = keyHashCode(s);

    // return if cell is already in list with updated key
    if ((curr != cellOpenHash.end()) && (close(csum,curr->second))) return;

    cellOpenHash[s] = csum;
    prioQueueHash.push(s);
}

/* void DstarLite::remove(state s)
* removes state from cellOpenHash
*/
void DstarLite::remove(state s) {

    dsl_oh::iterator curr = cellOpenHash.find(s);
    if (curr == cellOpenHash.end()) return;
    cellOpenHash.erase(curr);
}

/* double DstarLite::heuristic(state a, state b)
* 8-way distance scaled by a constant
*/
double DstarLite::heuristic(state a, state b) {
    return eightDist(a, b) * C1;
}

/* state DstarLite::calculateKey(state s)
* as per the research paper
*/
state DstarLite::calculateKey(state s) {
    
    double min_val = fmin(getG(s), getRHS(s));

    s.k.first = min_val + heuristic(s_start, s) + k_m;
    s.k.second = min_val;

    return s;
}

/* double DstarLite::cost(state a, state b)
* cost to traverse from state a to state b (off state a, could also do onto state b?)
*/
double DstarLite::cost(state a, state b) {

    int xd = fabs(a.x-b.x);
    int yd = fabs(a.y-b.y);
    double scale = 1;

    if(xd+yd>1) scale = M_SQRT2;

    if (cellInfoHash.count(a) == 0) return scale*C1;
    return scale*cellInfoHash[a].cost;
}

/* void DstarLite::updateCell(int x, int y, double val)
* see paper
*/
void DstarLite::updateCell(int x, int y, double val) {

    state s;

    s.x = x;
    s.y = y;

    if ((s == s_start) || (s == s_goal)) return;

    makeNewCell(s);
    cellInfoHash[s].cost = val;

    updateVertex(s);
}

/* void DstarLite::getSucc(state s, list<state> &states)
* returns list of cell neighbors
*/
void DstarLite::getSucc(state s, list<state> &states) {
    
    states.clear();
    s.k.first = INF;
    s.k.second = INF;

    if (notTraversable(s)) return;
    
    s.x += 1;
    states.push_front(s);
    s.y += 1;
    states.push_front(s);
    s.x -= 1;
    states.push_front(s);
    s.x -= 1;
    states.push_front(s);
    s.y -= 1; 
    states.push_front(s);
    s.y -= 1;
    states.push_front(s);
    s.x += 1;
    states.push_front(s);
    s.x += 1;
    states.push_front(s);
}

/* void DstarLite::getPred(state s, list<state> &states)
* same as above but for pred
*/
void DstarLite::getPred(state s, list<state> &states) {
    states.clear();
    s.k.first = INF;
    s.k.second = INF;

    s.x += 1;
    if (!notTraversable(s)) states.push_front(s);
    s.y += 1;
    if (!notTraversable(s)) states.push_front(s);
    s.x -= 1;
    if (!notTraversable(s)) states.push_front(s);
    s.x -= 1;
    if (!notTraversable(s)) states.push_front(s);
    s.y -= 1; 
    if (!notTraversable(s)) states.push_front(s);
    s.y -= 1;
    if (!notTraversable(s)) states.push_front(s);
    s.x += 1;
    if (!notTraversable(s)) states.push_front(s);
    s.x += 1;
    if (!notTraversable(s)) states.push_front(s);

}

/* void DstarLite::updateStart(int x, int y)
* changes the pos of the robot
*/
void DstarLite::updateStart(int x, int y) {
    s_start.x = x;
    s_start.y = y;

    k_m += heuristic(s_last, s_start);

    s_start = calculateKey(s_start);
    s_last = s_start;
}

// todo
/* void DstarLite::updateGoal(int x, int y)
* very much need this for the autonomous phase, don't know how im gonna do it yet
* should save portions of the map that we have data on already, clear the map, 
* then move the goal and put the non-empty cells back
* also need it for when we first enter autonomous state
*/

/* bool DstarLite::replan()
* updates the costs for all cells, computes shortest path to goal
*/
bool DstarLite::replan() {
    path.clear();

    int res = computeShortestPath();

    if (isinf(res)) {
        fprintf(stderr, "no path to goal");
        return false;
    }

    list<state> s;
    list<state>::iterator i;

    state cur = s_start;

    if (isinf(getG(s_start))) {
        fprintf(stderr, "no path to goal");
        return false;
    }

    while(cur != s_goal) {
        path.push_back(cur);
        getSucc(cur, s);

        if (s.empty()) {
            fprintf(stderr, "no path to goal");
            return false;
        }

        double cmin = INF;
        double tmin;
        state smin;

        for (i=s.begin(); i!=s.end(); i++) {

            if (notTraversable(*i)) continue;
            double val = cost(cur, *i);
            double val2 = trueDist(*i, s_goal) + trueDist(s_start, *i);
            val += getG(*i);

            if (close(val, cmin)) {
                if (tmin > val2) {
                    tmin = val2;
                    cmin = val;
                    smin = *i;
                }
            } else if (val < cmin) {
                tmin = val2;
                cmin = val; 
                smin = *i;
            }
        }
        s.clear();
        cur = smin;
    }
    path.push_back(s_goal);
    return true;

}