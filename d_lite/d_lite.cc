#include "d_lite.hh"

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
    maxSteps = 2000000;  // steps before we give up
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
    while(!prioQueue.empty()) prioQueue.pop();

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

/* int DstarLite::computeShortestPath()
* computes the shortest path from start to goal,
* per the paper
*/
// todo finish
int DstarLite::computeShortestPath() {
    list<state> s;
    list<state>::iterator i;

    if (prioQueue.empty()) return 1;

    int k = maxSteps;
    while ((!prioQueue.empty()) && 
            (prioQueue.top() < (s_start = calculateKey(s_start))) ||
            (getRHS(s_start) != getG(s_start))) {
        
        // todo
        // probably should have better error handeling here?
        // maybe make it recalculate some of the states, or 'forget'
        // any states that are blocking it? Not sure yet
        if (k-- == 0) {
            fprintf(stderr, "max steps\n");
            return -1;
        }

        state s;

        // get our next state with lazy removal
        while (true) {
            s = prioQueue.top();
            prioQueue.pop();

            // if state isn't valid, try again
            if (!isValid(s)) continue;
            // todo look into this, i should be checking more stuff here
            break;
        }

        // no longer open, remove from open hash
        cellOpenHash.erase(cellOpenHash.find(s));

        state k_old = s;
        if (k_old < calculateKey(s)) { 
            insert(s);  // key is innnacurate, put it back in w/ updated key
        } else {
            double sRHS = getRHS(s);
            
            // todo lowkey don't understand this part
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

    } if (cellOpenHash.find(s) != cellOpenHash.end()) {
        cellOpenHash.erase(s);
        // todo this is wrong? I honestly don't know if i need this what is the point
        // will probably just make insert rmove it for us
    } if (!close(getG(s), getRHS(s))) {
        calculateKey(s); // todo do i need this (will probably end up adding this to insert) (but then again maybe not)
        insert(s);
    }
}

// todo finalize this and finnish it
/* void DstarLite::insert(state s)
* calculates key (maybe), inserts, and removes any existing options (maybe or I might leave them?)
*/
void DstarLite::insert(state s){

    dsl_oh::iterator curr = cellOpenHash.find(s);
    s = calculateKey(s);
    float csum = keyHashCode(s);

    // if 
}


