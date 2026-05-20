/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-03-09 19:43:31
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2022-03-17 17:21:05
 */

#include "DStarLite.h"
#include <iostream>
#include "xutils.h"

#define DSTARLITE_OPTIMISE

/* void DStarLite::DStarLite()
 * --------------------------
 * Constructor sets constants.
 */
DStarLite::DStarLite() {

  maxSteps = 80000;  // node expansions before we give up
  C1       = 1;      // cost of an unseen cell
  width=0; 

}

/* double DStarLite::keyHashCode(state u)
 * --------------------------
 * Returns the key hash code for the state u, this is used to compare
 * a state that have been updated
 */
double DStarLite::keyHashCode(state u) {

  return (double)(u.k.first + 1193*u.k.second);

}

/* bool DStarLite::isValid(state u)
 * --------------------------
 * Returns true if state u is on the open list or not by checking if
 * it is in the hash table.
 */
bool DStarLite::isValid(state u) {

  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return false;
  if (!close(keyHashCode(u), cur->second)) return false;
  return true;

}

/* void DStarLite::getPath()
 * --------------------------
 * Returns the path created by int &resultError
 */
std::list<state> DStarLite::getPath()
{
	return path;
}

/* bool DStarLite::occupied(state u)
 * --------------------------
 * returns true if the cell is occupied (non-traversable), false
 * otherwise. non-traversable are marked with a cost < 0.
 */
bool DStarLite::occupied(state u) {

  ds_ch::iterator cur = cellHash.find(u);

  if (cur == cellHash.end()) return false;
  return (cur->second.cost < 0);
}

bool DStarLite::occupiedAndNoWall(state u) {

  ds_ch::iterator cur = cellHash.find(u);

  if (isOntheWall(u))
  {
		return true;
	}

  if (cur == cellHash.end()) return false;
  return (cur->second.cost < 0);
}

/* void DStarLite::init(int sX, int sY, int gX, int gY)
 * --------------------------
 * Init DStarLite with start and goal coordinates, rest is as per
 * [S. Koenig, 2002]
 */
void DStarLite::init(int sX, int sY, int gX, int gY) {

  cellHash.clear();
  path.clear();
  openHash.clear();
  //while(!openList.empty()) openList.pop();
  openList = ds_pq();

  k_m = 0;

  s_start.x = sX;
  s_start.y = sY;
  s_goal.x  = gX;
  s_goal.y  = gY;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

}
/* void DStarLite::makeNewCell(state u)
 * --------------------------
 * Checks if a cell is in the hash table, if not it adds it in.
 */
void DStarLite::makeNewCell(state u) {

  if (cellHash.find(u) != cellHash.end()) return;

  cellInfo tmp;
  tmp.g       = tmp.rhs = heuristic(u,s_goal);
  tmp.cost    = C1;
  cellHash[u] = tmp;
}

/* double DStarLite::getG(state u)
 * --------------------------
 * Returns the G value for state u.
 */
double DStarLite::getG(state u) {

  if (cellHash.find(u) == cellHash.end())
    return heuristic(u,s_goal);
  return cellHash[u].g;

}

/* double DStarLite::getRHS(state u)
 * --------------------------
 * Returns the rhs value for state u.
 */
double DStarLite::getRHS(state u) {

  if (u == s_goal) return 0;

  if (cellHash.find(u) == cellHash.end())
    return heuristic(u,s_goal);
  return cellHash[u].rhs;

}

/* void DStarLite::setG(state u, double g)
 * --------------------------
 * Sets the G value for state u
 */
void DStarLite::setG(state u, double g) {

  makeNewCell(u);
  cellHash[u].g = g;
}

/* void DStarLite::setRHS(state u, double rhs)
 * --------------------------
 * Sets the rhs value for state u
 */
void DStarLite::setRHS(state u, double rhs) {

  makeNewCell(u);
  cellHash[u].rhs = rhs;

}

/* double DStarLite::eightCondist(state a, state b)
 * --------------------------
 * Returns the 8-way distance between state a and state b.
 */
double DStarLite::eightCondist(state a, state b) {
  double min = fabs(a.x - b.x);
  double max = fabs(a.y - b.y);
  if (min > max) {
    double temp = min;
    min = max;
    max = temp;
  }
  return ((M_SQRT2-1.0)*min + max);
}

/* int DStarLite::computeShortestPath()
 * --------------------------
 * As per [S. Koenig, 2002] except for 2 main modifications:
 * 1. We stop planning after a number of steps, 'maxsteps' we do this
 *    because this algorithm can plan forever if the start is
 *    surrounded by obstacles.
 * 2. We lazily remove states from the open list so we never have to
 *    iterate through it.
 */
int DStarLite::computeShortestPath(void)
{

	std::list<state> s;
	std::list<state>::iterator i;

	if (openList.empty()) 
	{
		printf("openList empty\n");
		return 1;
	}

	if (isOntheWall(s_start) || isOntheWall(s_goal))
		return -1;

	int k = 0;
#ifdef DSTARLITE_OPTIMISE
	while (1)
	{
		state u;

		// 0. begin while condition judge
		bool condition = !openList.empty();

		// calculate key
		double start_rhs, start_g;
		if (cellHash.find(s_start) != cellHash.end())
		{
			cellInfo start_info = cellHash[s_start];
			start_rhs = start_info.rhs;
			start_g = start_info.g;
		}
		else
		{
			start_rhs = start_g = heuristic(s_start, s_goal);
		}
		if (s_start == s_goal)
			start_rhs = 0;

		double start_g_rhs_min = fmin(start_rhs,start_g);
		if (condition)
		{
			s_start.k.first = start_g_rhs_min + k_m;
			s_start.k.second = start_g_rhs_min;
			u = openList.top();
			if (u < s_start)
			{
				condition = true;
			}
			else
			{
				condition = false;
			}
		}

		if (!condition)
		{
			if (start_g != start_rhs)
				condition = true;
			else
				condition = false;
		}
		if (!condition)
			break;
		// end while condition judge

		bool test = (start_rhs != start_g);
#else
	while ((!openList.empty()) &&
		(openList.top() < (s_start = calculateKey(s_start))) ||
		(getRHS(s_start) != getG(s_start)))
	{
		state u;
		bool test = (getRHS(s_start) != getG(s_start));
#endif

		if (k++ > maxSteps)
		{
			//std::cout << "At maxsteps  ××××××××××××××××××-------------×××××××××××××\n" << std::endl;
			fprintf(stderr, "At maxsteps %d\n", maxSteps);
			return -1;
		}

		// lazy remove
		while (1)
		{
			if (openList.empty()) return 1;
			u = openList.top();
			openList.pop();

			if (!isValid(u)) continue;
			if (!(u < s_start) && (!test)) return 2;
			break;
		}

		ds_oh::iterator cur = openHash.find(u);
		openHash.erase(cur);

		state k_old = u;

#ifdef DSTARLITE_OPTIMISE
		double u_rhs, u_g;
		bool bUExistinCellHash = true;
		if (cellHash.find(u) == cellHash.end())
		{
			bUExistinCellHash = false;
			if (u == s_goal)
			{
				u_rhs = 0;
				u_g = heuristic(u, s_goal);
			}
			else
			{
				u_rhs = u_g = heuristic(u, s_goal);
			}
		}
		else
		{
			cellInfo u_info = cellHash[u];
			if (u == s_goal)
			{
				u_rhs = 0;
				u_g = u_info.g;
			}
			else
			{
				u_rhs = u_info.rhs;
				u_g = u_info.g;
			}
		}
		state u_new = u;
		double u_rhs_g_min = fmin(u_rhs, u_g);
		u_new.k.first = u_rhs_g_min + heuristic(u, s_start) + k_m;
		u_new.k.second = u_rhs_g_min;

		if (k_old < u_new)
		{
			// u is out of date
			if (!isOntheWall(u))
			{
				insert(u);
			}
		}
		else
		{
			bool bGetBetter = (u_g > u_rhs);
			double update_G;
			if (bGetBetter)
			{
				update_G = u_rhs;
			}
			else
			{
				update_G = INFINITY;
			}
			
			
			// a. setG(u, getRHS(u));
			if (!bUExistinCellHash)
			{
				cellInfo u_info;
				u_info.rhs = u_g; // u_g == heuristic(u, s_goal);
				u_info.g  = update_G;
				u_info.cost = C1;
				cellHash[u] = u_info;
			}
			else
			{				
				cellHash[u].g = update_G;
			}

			// b. getPred(u, s); and updateVertex 8 neighbor if available
			for (int y = -1; y <= 1; y++)
			{
				for (int x = -1; x <= 1; x++)
				{
					if ((x == 0) && (y == 0))
						continue;

					state u_tmp = u;
					u_tmp.x += x;
					u_tmp.y += y;
					if (u_tmp.x<0||u_tmp.x>width||u_tmp.y<0||u_tmp.y>height)
					{
						continue;
					}
					if (!isOntheWall(u_tmp))
					{						
						updateVertex(u_tmp);
					}
				}
			}

			// c. updateVertex u
			if (!bGetBetter)
			{
				updateVertex(u);
			}
		}
#else
		if (k_old < calculateKey(u))
		{ // u is out of date
			if (!isOntheWall(u))
			{
				insert(u);
			}
			/*if (isOntheWall(u))
			{
				static int i = 0;
				i++;
				printf("unexpected situation!");
			}*/
		}
		else if (getG(u) > getRHS(u))
		{ // needs update (got better)
			setG(u, getRHS(u));
			getPred(u, s);
			for (i = s.begin(); i != s.end(); i++)
			{
				updateVertex(*i);
			}
		}
		else
		{   // g <= rhs, state has got worse
			setG(u, INFINITY);
			getPred(u, s);
			for (i = s.begin(); i != s.end(); i++)
			{
				updateVertex(*i);
			}
			updateVertex(u);
		}
#endif
		//draw();
	}
	printf("DstarLite search path interation count is: %d\n", k);
	return 0;
}

/* bool DStarLite::close(double x, double y)
 * --------------------------
 * Returns true if x and y are within 10E-5, false otherwise
 */
bool DStarLite::close(double x, double y) {

  if (std::isinf(x) && std::isinf(y)) return true;
  return (fabs(x-y) < 0.00001);

}

/* void DStarLite::updateVertex(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void DStarLite::updateVertex(state u) {

#ifdef DSTARLITE_OPTIMISE
	double u_rhs, u_g, u_info_cost;
	bool bUExistinCellHash = true;
	if (cellHash.find(u) == cellHash.end())
	{
		bUExistinCellHash = false;
		if (u == s_goal)
		{
			u_rhs = 0;
			u_g = heuristic(u, s_goal);
		}
		else
		{
			u_rhs = u_g = heuristic(u, s_goal);
		}

		u_info_cost = C1;
	}
	else
	{
		cellInfo u_info = cellHash[u];
		if (u == s_goal)
		{
			u_rhs = 0;
			u_g = u_info.g;
		}
		else
		{
			u_rhs = u_info.rhs;
			u_g = u_info.g;
		}

		u_info_cost = u_info.cost;
	}

	if (u != s_goal)
	{
		double tmp = INFINITY;
		double tmp2 = 0.0;
        // int cnt = 0;

		std::list<state> s;
  		std::list<state>::iterator i;
 
   		getSucc(u,s);
		
		//if (!isOntheWall(u))
		{
			for (i = s.begin();i != s.end(); i++) 
			{
				//tmp2 = getG(u_tmp) + cost(u, u_tmp);
				double cost_u_u_tmp = 0;
				int xd = abs(u.x - (*i).x);
				int yd = abs(u.y - (*i).y);
				double scale = 1;
				//if (xx*yy != 0) 
				if (xd+yd>1)
					scale = M_SQRT2;
				if (!bUExistinCellHash)
					cost_u_u_tmp = scale * C1;
				else
					cost_u_u_tmp = scale * u_info_cost;
				
				tmp2 = getG(*i) + cost_u_u_tmp;
				if (tmp2 < tmp)
					tmp = tmp2;
			}
					
			float dist = calDistToWall(u,3);
            if (dist<10)
            {
                //tmp+=(3/(dist*dist)) +cnt;
				//tmp+=(3/(dist*dist));
				tmp+=(9/(dist*dist));
            } 
		}

		if (!close(u_rhs, tmp))
		{
			////setRHS(u, tmp);
			if (!bUExistinCellHash)
			{
				cellInfo u_info;
				u_info.rhs = tmp;				
				u_info.g = u_g;
				u_info.cost = C1;
				cellHash[u] = u_info;
			}
			else
			{							
				cellHash[u].rhs = tmp;
			}
			u_rhs = ((u == s_goal) ? 0 : tmp);
		}
	}
	if (!close(u_g, u_rhs))
		insert(u);
#else
  std::list<state> s;
  std::list<state>::iterator i;
  //s.reserve(8);

  if (u != s_goal) {
    getSucc(u,s);
    double tmp = INFINITY;
    double tmp2=0.0;

    for (i=s.begin();i != s.end(); i++) {
      tmp2 = getG(*i) + cost(u,*i);
      if (tmp2 < tmp) tmp = tmp2;
    }

    if (!isOntheWall(u))
	{
		float dist = calDistToWall(u,3);
		if (dist<10)
		{
			tmp+=(1/(dist*dist))+(8-s.size());
		}
	}

    if (!close(getRHS(u),tmp)) setRHS(u,tmp);
  }

 
  if (!close(getG(u),getRHS(u))) insert(u);
#endif
}

/* void DStarLite::insert(state u)
 * --------------------------
 * Inserts state u into openList and openHash.
 */
void DStarLite::insert(state u) {

  ds_oh::iterator cur;
  double csum;

  u    = calculateKey(u);
  cur  = openHash.find(u);
  csum = keyHashCode(u);
  // return if cell is already in list. TODO: this should be
  // uncommented except it introduces a bug, I suspect that there is a
  // bug somewhere else and having duplicates in the openList queue
  // hides the problem...
  if ((cur != openHash.end()) && (close(csum,cur->second))) return;

  openHash[u] = csum;
  openList.push(u);
}

/* void DStarLite::remove(state u)
 * --------------------------
 * Removes state u from openHash. The state is removed from the
 * openList lazilily (in replan) to save computation.
 */
void DStarLite::remove(state u) {

  ds_oh::iterator cur = openHash.find(u);
  if (cur == openHash.end()) return;
  openHash.erase(cur);
}


/* double DStarLite::trueDist(state a, state b)
 * --------------------------
 * Euclidean cost between state a and state b.
 */
double DStarLite::trueDist(state a, state b) {

  double x = a.x-b.x;
  double y = a.y-b.y;
  return sqrt(x*x + y*y);

}

/* double DStarLite::heuristic(state a, state b)
 * --------------------------
 * Pretty self explanitory, the heristic we use is the 8-way distance
 * scaled by a constant C1 (should be set to <= min cost).
 */
double DStarLite::heuristic(state a, state b) {
  return eightCondist(a,b)*C1;
}

/* state DStarLite::calculateKey(state u)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
state DStarLite::calculateKey(state u) {

  double val = fmin(getRHS(u),getG(u));

  u.k.first  = val + heuristic(u,s_start) + k_m;
  u.k.second = val;

  return u;

}

/* double DStarLite::cost(state a, state b)
 * --------------------------
 * Returns the cost of moving from state a to state b. This could be
 * either the cost of moving off state a or onto state b, we went with
 * the former. This is also the 8-way cost.
 */
double DStarLite::cost(state a, state b) {

  int xd = fabs(a.x-b.x);
  int yd = fabs(a.y-b.y);
  double scale = 1;

  if (xd+yd>1) scale = M_SQRT2;

  if (cellHash.find(a) == cellHash.end()) return scale*C1;
  return scale*cellHash[a].cost;

}
/* void DStarLite::updateCell(int x, int y, double val)
 * --------------------------
 * As per [S. Koenig, 2002]
 */
void DStarLite::updateCell(int x, int y, double val) {

  state u;

  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;

  makeNewCell(u);
  cellHash[u].cost = val;

  updateVertex(u);
}

void DStarLite::initCell(int x, int y, double val) {

  state u;

  u.x = x;
  u.y = y;

  if ((u == s_start) || (u == s_goal)) return;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = val;

  cellHash[u] = tmp;
}


/* void DStarLite::getSucc(state u,list<state> &s)
 * --------------------------
 * Returns a list of successor states for state u, since this is an
 * 8-way graph this list contains all of a cells neighbours. Unless
 * the cell is occupied in which case it has no successors.
 */
#if 0
void DStarLite::getSucc(state u, std::list<state> &s) {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (isOntheWall(u)) return;

  u.x += 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.y += 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.y -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.y -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x += 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x += 1;
  if (!isOntheWall(u)) s.push_front(u);

}
#else
void DStarLite::getSucc(state u, std::list<state> &s) {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;

  if (isOntheWall(u)) return;

  state u_left=u;
  u_left.x = u.x - 1;
  state u_right = u;
  u_right.x = u.x + 1;
  state u_top = u;
  u_top.y = u.y - 1;
  state u_bottom = u;
  u_bottom.y = u.y + 1;

  if (isOntheWall(u_left)||isOntheWall(u_right)||isOntheWall(u_top)||isOntheWall(u_bottom))
  {
	//return;
  }
  
  
  u.x += 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.y += 1;
  //if (!isOntheWall(u)&&(!isOntheWall(u_right)|| !isOntheWall(u_bottom))) s.push_front(u);
  u.x -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x -= 1;
  //if (!isOntheWall(u)&&(!isOntheWall(u_left)|| !isOntheWall(u_bottom))) s.push_front(u);
  u.y -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.y -= 1;
  //if (!isOntheWall(u)&&(!isOntheWall(u_left)|| !isOntheWall(u_top))) s.push_front(u);
  u.x += 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x += 1;
  //if (!isOntheWall(u)&&(!isOntheWall(u_right)|| !isOntheWall(u_top))) s.push_front(u);

}

#endif

/* void DStarLite::getPred(state u,list<state> &s)
 * --------------------------
 * Returns a list of all the predecessor states for state u. Since
 * this is for an 8-way connected graph the list contails all the
 * neighbours for state u. Occupied neighbours are not added to the
 * list.
 */
#if 0
void DStarLite::getPred(state u, std::list<state> &s) {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;
  
  state u_left=u;
  u_left.x = u.x - 1;
  state u_right = u;
  u_right.x = u.x + 1;
  state u_top = u;
  u_top.y = u.y - 1;
  state u_bottom = u;
  u_bottom.y = u.y + 1;


  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.y += 1;
  if (!occupied(u)||(!occupied(u_right)|| !occupied(u_bottom)))  s.push_front(u);
  u.x -= 1;
  if (!occupied(u)) s.push_front(u);
  u.x -= 1;
  if (!occupied(u)||(!occupied(u_left) || !occupied(u_bottom))) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)) s.push_front(u);
  u.y -= 1;
  if (!occupied(u)||(!occupied(u_left) || !occupied(u_top))) s.push_front(u);
  u.x += 1;
  if (!occupied(u)) s.push_front(u);
  u.x += 1;
  if (!occupied(u)|| (!occupied(u_right) || !occupied(u_top))) s.push_front(u);

}
#else
void DStarLite::getPred(state u, std::list<state> &s) {

  s.clear();
  u.k.first  = -1;
  u.k.second = -1;
  
  u.x += 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.y += 1;
  if (!isOntheWall(u))  s.push_front(u);
  u.x -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.y -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.y -= 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x += 1;
  if (!isOntheWall(u)) s.push_front(u);
  u.x += 1;
  if (!isOntheWall(u)) s.push_front(u);

}

#endif


/* void DStarLite::updateStart(int x, int y)
 * --------------------------
 * Update the position of the robot, this does not force a replan.
 */
void DStarLite::updateStart(int x, int y) {

  s_start.x = x;
  s_start.y = y;

  k_m += heuristic(s_last,s_start);

  s_start = calculateKey(s_start);
  s_last  = s_start;

}

/* void DStarLite::updateGoal(int x, int y)
 * --------------------------
 * This is somewhat of a hack, to change the position of the goal we
 * first save all of the non-empty on the map, clear the map, move the
 * goal, and re-add all of non-empty cells. Since most of these cells
 * are not between the start and goal this does not seem to hurt
 * performance too much. Also it free's up a good deal of memory we
 * likely no longer use.
 */
void DStarLite::updateGoal(int x, int y) {

	//uint64_t t0 = getTimeStap_us();
  std::list< std::pair<ipoint2, double> > toAdd;
  std::pair<ipoint2, double> tp;

  ds_ch::iterator i;
  std::list< std::pair<ipoint2, double> >::iterator kk;

  // uint64_t st0 = getTimeStap_us();

  for(i=cellHash.begin(); i!=cellHash.end(); i++) {
    if (!close(i->second.cost, C1)) {
      tp.first.x = i->first.x;
      tp.first.y = i->first.y;
      tp.second = i->second.cost;
      toAdd.push_back(tp);
    }
  }

  // uint64_t st1 = getTimeStap_us();

  cellHash.clear();
  openHash.clear();
  //uint64_t t2 = getTimeStap_us();

  // while(!openList.empty())
  //  openList.pop();
  openList= ds_pq();

  //uint64_t t3 = getTimeStap_us();

  k_m = 0;

  s_goal.x  = x;
  s_goal.y  = y;

  cellInfo tmp;
  tmp.g = tmp.rhs =  0;
  tmp.cost = C1;

  cellHash[s_goal] = tmp;

  tmp.g = tmp.rhs = heuristic(s_start,s_goal);
  tmp.cost = C1;
  cellHash[s_start] = tmp;
  s_start = calculateKey(s_start);

  s_last = s_start;

  // uint64_t st2 = getTimeStap_us();

  for (kk=toAdd.begin(); kk != toAdd.end(); kk++) {
    updateCell(kk->first.x, kk->first.y, kk->second);
  }

  insert(s_goal);


  // uint64_t st3 = getTimeStap_us();
  // uint32_t diff_t0= st1-st0;
  // uint32_t diff_t1= st2-st1;
  // uint32_t diff_t2= st3-st2;

  //printf("updategoal cost time is : %d   %d   %d us \n",diff_t0,diff_t1,diff_t2);


}

/* bool DStarLite::int &resultError
 * --------------------------
 * Updates the costs for all cells and computes the shortest path to
 * goal. Returns true if a path is found, false otherwise. The path is
 * computed by doing a greedy search over the cost+g values in each
 * cells. In order to get around the problem of the robot taking a
 * path that is near a 45 degree angle to goal we break ties based on
 *  the metric euclidean(state, goal) + euclidean(state,start).
 */
bool DStarLite::replan(int& resultError) {

  path.clear();
  resultError = 0;

  int res = computeShortestPath();
  printf("res is %d \n",res);
  //printf("res: %d ols: %d ohs: %d tk: [%f %f] sk: [%f %f] sgr: (%f,%f)\n",res,openList.size(),openHash.size(),openList.top().k.first,openList.top().k.second, s_start.k.first, s_start.k.second,getRHS(s_start),getG(s_start));
  if (res < 0) {
    fprintf(stderr, "NO PATH TO GOAL 1\n");
	resultError = 1;
    return false;
  }
  std::list<state> n;
  std::list<state>::iterator i;
  //n.reserve(8);

  state cur = s_start;

  if (std::isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL 2\n");
	resultError = 2;
    return false;
  }

  int32_t timeCnt=0;
  while(cur != s_goal) {

    timeCnt++;
	if (timeCnt>500)
	{
		printf("^^^^^^^^^^^^^^^^^^dstar path is failed \n");
		resultError = 4;
		path.clear();
  		return false;
	}
	
	printf("cur is %d:  %d %d \n",timeCnt,cur.x,cur.y);
	path.push_back(cur);
    getSucc(cur, n);

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL 3\n");
	  resultError = 3;
      return false;
    }

    double cmin = INFINITY;
	double tmin = INFINITY;        // WQ: TODO!!!  default value 0 is OK ?
    state smin = {0, 0, {0.0, 0.0}};

    for (i=n.begin(); i!=n.end(); i++) {

      //if (occupied(*i)) continue;
      double val  = cost(cur,*i);
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);

      if (close(val,cmin)) {
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
    n.clear();
    cur = smin;
  }
  path.push_back(s_goal);
  /*for (std::list<state>::iterator iter = path.begin(); iter != path.end(); iter++)
  {
	  printf("%d %d\n", iter->x, iter->y);
  }*/
  //printf("dstar path is %d \n",path.size());
  return true;
}



bool DStarLite::replanPath() {

  path.clear();

  
  std::list<state> n;
  std::list<state>::iterator i;
  //n.reserve(8);

  state cur = s_start;

  if (std::isinf(getG(s_start))) {
    fprintf(stderr, "NO PATH TO GOAL 2\n");
    //return false;
  }

  int32_t timeCnt=0;
  while(cur != s_goal) {

    timeCnt++;
	if (timeCnt>500)
	{
		printf("^^^^^^^^^^^^^^^^^^dstar path is failed \n");
		path.clear();
  		return false;
	}
	
	printf("cur is %d:  %d %d \n",timeCnt,cur.x,cur.y);
	path.push_back(cur);
    getSucc(cur, n);

    if (n.empty()) {
      fprintf(stderr, "NO PATH TO GOAL 3\n");
      return false;
    }

    double cmin = INFINITY;
	double tmin = INFINITY;        // WQ: TODO!!!  default value 0 is OK ?
    state smin = {0, 0, {0.0, 0.0}};

    for (i=n.begin(); i!=n.end(); i++) {

      //if (occupied(*i)) continue;
      double val  = cost(cur,*i);
      double val2 = trueDist(*i,s_goal) + trueDist(s_start,*i); // (Euclidean) cost to goal + cost to pred
      val += getG(*i);

      if (close(val,cmin)) {
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
    n.clear();
    cur = smin;
  }
  path.push_back(s_goal);
  /*for (std::list<state>::iterator iter = path.begin(); iter != path.end(); iter++)
  {
	  printf("%d %d\n", iter->x, iter->y);
  }*/
  //printf("dstar path is %d \n",path.size());
  return true;
}


/* ADD BY WQ */
#ifdef _WIN32
#include "opencv2/opencv.hpp"
static cv::Mat dstartlite_img;
static cv::Point dstartlite_img_offset;
static cv::Vec3b dstartlite_brushcolor;
#endif
void DStarLite::draw()
{
#ifdef _WIN32 

	// draw img
	dstartlite_img = cv::Mat(320 * 5, 320 * 5, CV_8UC3, cv::Scalar(0));

	ds_ch::iterator iter;
	ds_oh::iterator iter1;
	state t;

	std::list<state>::iterator iter2;
	for (iter = cellHash.begin(); iter != cellHash.end(); iter++) 
	{
		
		if (iter->second.cost == 1)
			dstartlite_brushcolor = cv::Vec3b(0, 255, 0);
		else if (iter->second.cost < 0) 
			dstartlite_brushcolor = cv::Vec3b(0, 0, 255);
		else 
			dstartlite_brushcolor = cv::Vec3b(255, 0, 0);
		drawCell(iter->first, 2);
	}

	dstartlite_brushcolor = cv::Vec3b(0, 255, 255);
	drawCell(s_start, 2);
	dstartlite_brushcolor = cv::Vec3b(255, 0, 255);
	drawCell(s_goal, 2);

	for (iter1 = openHash.begin(); iter1 != openHash.end(); iter1++)
	{
		dstartlite_brushcolor = cv::Vec3b(204, 0, 102);
		drawCell(iter1->first, 1);
	}

	dstartlite_brushcolor = cv::Vec3b(255, 255, 255);
	for (iter2 = path.begin(); iter2 != path.end(); iter2++)
	{
		drawCell(*iter2, 0);
	}

	//cv::imshow("123", dstartlite_img);
	//cv::waitKey(30);
	return;
#endif
}

void DStarLite::drawCell(state s, float size)
{
#ifdef _WIN32
	cv::rectangle(dstartlite_img, dstartlite_img_offset +  cv::Point(5 * s.x - size, 5 * s.y - size), dstartlite_img_offset + cv::Point(5 * s.x + size, 5 * s.y + size), dstartlite_brushcolor);
#endif
}

int DStarLite::DStarLiteSave(const char* filepath)
{
	FILE* fp = fopen(filepath,"wb");

	// 1. list<state> path;
	int path_size = path.size();
	fwrite(&path_size, sizeof(int), 1, fp);
	auto path_iter = path.begin();
	for (int i = 0; i < path_size; i++, path_iter++)
	{
		path_iter->save(fp);
	}

  //return true;

	// 2. ...
	fwrite(&C1, sizeof(double), 1, fp);
	fwrite(&k_m, sizeof(double), 1, fp);
	s_start.save(fp);
	s_goal.save(fp);
	s_last.save(fp);
	fwrite(&maxSteps, sizeof(int), 1, fp);

	ds_pq openList1 = openList;   // priority openList
    ds_ch cellHash1 = cellHash;   // 
    ds_oh openHash1 = openHash;
	// 3. openList
	int openListSize = openList1.size();
	printf("openListSize is %d \n",openListSize);
	fwrite(&openListSize, sizeof(int), 1, fp);
	for (int i = 0; i < openListSize; i++)
	{
		openList1.top().save(fp);
		openList1.pop();
	}

	printf("openListSize after is %d \n",openListSize);

	// 4. cellHash
	int cellHashSize = cellHash1.size();
	fwrite(&cellHashSize, sizeof(int), 1, fp);
	printf("cellHashSize is %d \n",cellHashSize);
	auto cellHash_iter = cellHash1.begin();
	for (int i = 0; i < cellHashSize; i++, cellHash_iter++)
	{
		cellHash_iter->first.save(fp);
		cellHash_iter->second.save(fp);
	}

	// 5. openHash
	int openHashSize = openHash1.size();
	fwrite(&openHashSize, sizeof(int), 1, fp);
	printf("openHashSize is %d \n",openHashSize);
	auto openHash_iter = openHash1.begin();
	for (int i = 0; i < openHashSize; i++, openHash_iter++)
	{
		openHash_iter->first.save(fp);
		fwrite(&(openHash_iter->second), sizeof(double), 1, fp);
	}

	fclose(fp);
	return 0;
}

int DStarLite::DStarLiteLoad(const char* filepath)
{

	FILE* fp = fopen(filepath, "rb");
  if (fp==NULL)
  {
    return false;
  }
  
	// 1. list<state> path;
	path.clear();
	int path_size = 0;
	fread(&path_size, sizeof(int), 1, fp);
	for (int i = 0; i < path_size; i++)
	{
		state tmp;
		tmp.load(fp);
		path.push_back(tmp);	
	}

 // return true;

	// 2. ...
	fread(&C1, sizeof(double), 1, fp);
	fread(&k_m, sizeof(double), 1, fp);
	s_start.load(fp);
	s_goal.load(fp);
	s_last.load(fp);
	fread(&maxSteps, sizeof(int), 1, fp);

	// 3. openList
	while (!openList.empty())
	{
		openList.pop();
	}
	int openListSize = 0;
	fread(&openListSize, sizeof(int), 1, fp);
	for (int i = 0; i < openListSize; i++)
	{
		state tmp;
		tmp.load(fp);
		openList.push(tmp);
	}

	// 4. cellHash
	cellHash.clear();
	int cellHashSize = 0;
	fread(&cellHashSize, sizeof(int), 1, fp);
	for (int i = 0; i < cellHashSize; i++)
	{
		state tmp;
		cellInfo tmpinfo;
		tmp.load(fp);
		tmpinfo.load(fp);
		cellHash[tmp] = tmpinfo;
	}

	// 5. openHash
	openHash.clear();
	int openHashSize = 0;
	fread(&openHashSize, sizeof(int), 1, fp);
	for (int i = 0; i < openHashSize; i++)
	{
		state tmp;
		double val;
		tmp.load(fp);
		fread(&val, sizeof(double), 1, fp);
		openHash[tmp] = val;
	}

	fclose(fp);
	return 0;
}

int DStarLite::DstarLiteDrawObstacles(uint8_t* img_data, int img_width, int img_height)
{
	for (auto iter = cellHash.begin(); iter != cellHash.end(); iter++)
	{

		if (iter->second.cost < 0)
		{
			if (iter->first.x >= 0 && iter->first.x < img_width &&
				iter->first.y >= 0 && iter->first.y < img_height)
			{
				img_data[iter->first.y * img_width + iter->first.x] = 255;
			}
		}
	}
	return 0;
}

void DStarLite::setMap(uint8_t* map_data,int img_width,int img_height)
{
  obs_map=map_data;
  width=img_width;
  height = img_height;
}


bool DStarLite::isOntheWall(state u)
{
	if(width==0)
    return false;

  if (obs_map[u.y * width + u.x]!=0)
	{
		return true;
	}

	return false;
}


float DStarLite::calDistToWall(state u,int radius)
{
	float minDist = 1000;
	for (int yy = -radius; yy <= radius; yy++)
	{
		for (int xx = -radius; xx <= radius; xx++)
		{
			state newS;
			newS.x = u.x + xx;
			newS.y = u.y + yy;

			if (xx == 0 && yy == 0)
			{
				continue;
			}

			if (newS.x<0||newS.x>width||newS.y<0||newS.y>height)
			{
				continue;
			}

			if (isOntheWall(newS))
			{
				float dist = sqrt(xx * xx + yy * yy);
				//if (dist <= radius&&dist < minDist)
				if (/* dist <= radius&& */dist < minDist)
				{
					minDist = dist;
				}
			}
		}
	}
	return minDist;
}