/*
 * @Description: 
 * @version: 
 * @Author: Jephy Zhang
 * @Email: jephy.zhang@any-eye.com
 * @Date: 2020-03-09 11:19:48
 * @LastEditors: Jephy Zhang
 * @LastEditTime: 2022-03-11 18:02:25
 */

#ifndef     __DStarLite_H__
#define     __DStarLite_H__

#include <math.h>
#include <stack>
#include <queue>
#include <list>
#include <stdio.h>
#include <unordered_map>
#include <stdint.h>



class state {
 public:
  int x;
  int y;
  std::pair<double,double> k;

  bool operator == (const state &s2) const {
    return ((x == s2.x) && (y == s2.y));
  }

  bool operator != (const state &s2) const {
    return ((x != s2.x) || (y != s2.y));
  }

  bool operator > (const state &s2) const {
    if (k.first-0.00001 > s2.k.first) return true;
    else if (k.first < s2.k.first-0.00001) return false;
    return k.second > s2.k.second;
  }

  bool operator <= (const state &s2) const {
    if (k.first < s2.k.first) return true;
    else if (k.first > s2.k.first) return false;
    return k.second < s2.k.second + 0.00001;
  }


  bool operator < (const state &s2) const {
    if (k.first + 0.000001 < s2.k.first) return true;
    else if (k.first - 0.000001 > s2.k.first) return false;
    return k.second < s2.k.second;
  }

  int save(FILE* fp) const
  {
	  fwrite(&x, sizeof(int), 1, fp);
	  fwrite(&y, sizeof(int), 1, fp);
	  fwrite(&k.first, sizeof(double), 1, fp);
	  fwrite(&k.second, sizeof(double), 1, fp);

	  return 0;
  }
  int load(FILE* fp)
  {
	  fread(&x, sizeof(int), 1, fp);
	  fread(&y, sizeof(int), 1, fp);
	  fread(&k.first, sizeof(double), 1, fp);
	  fread(&k.second, sizeof(double), 1, fp);

	  return 0;
  }
};

struct ipoint2 {
  int x,y;
};

struct cellInfo {

  double g;
  double rhs;
  double cost;
  int save(FILE* fp) const
  {
	  fwrite(&g, sizeof(double), 1, fp);
	  fwrite(&rhs, sizeof(double), 1, fp);
	  fwrite(&cost, sizeof(double), 1, fp);

	  return 0;
  }
  int load(FILE* fp)
  {
	  fread(&g, sizeof(double), 1, fp);
	  fread(&rhs, sizeof(double), 1, fp);
	  fread(&cost, sizeof(double), 1, fp);

	  return 0;
  }

};

class state_hash {
 public:
  size_t operator()(const state &s) const {
    return s.x + 34245*s.y;
  }
};


typedef std::priority_queue<state, std::vector<state>, std::greater<state> > ds_pq;
typedef std::unordered_map<state, cellInfo, state_hash, std::equal_to<state> > ds_ch;
typedef std::unordered_map<state, double, state_hash, std::equal_to<state> > ds_oh;


class DStarLite
{
public:

  DStarLite();
  void   init(int sX, int sY, int gX, int gY);
  void   updateCell(int x, int y, double val);
  void   initCell(int x, int y, double val);
  void   updateStart(int x, int y);
  void   updateGoal(int x, int y);
  bool   replan(int& resultError);
  bool   replanPath();
  void   draw();
  void   drawCell(state s,float z);
  std::list<state> getPath();
  void   setMap(uint8_t* map_data,int img_width,int img_height);


  /**************  START: ADD BY XT **********************/
  // Add by WQ
  int DStarLiteSave(const char* filepath);
  int DStarLiteLoad(const char* filepath);
  int DstarLiteDrawObstacles(uint8_t* img_data, int img_width, int img_height);
  /**************  END: ADD BY XT **********************/
 private:

  std::list<state> path;

  double C1;
  double k_m;
  state s_start, s_goal, s_last;
  int maxSteps;
  ds_pq openList;   // priority openList
  ds_ch cellHash;   // 
  ds_oh openHash;

  uint8_t* obs_map;
  int width;
  int height;


  bool   close(double x, double y);
  void   makeNewCell(state u);
  double  getG(state u);
  double  getRHS(state u);
  void   setG(state u, double g);
  void  setRHS(state u, double rhs);
  double  eightCondist(state a, state b);
  int    computeShortestPath(void);
  void   updateVertex(state u);
  void   insert(state u);
  void   remove(state u);
  double  trueDist(state a, state b);
  double  heuristic(state a, state b);
  state  calculateKey(state u);
  void   getSucc(state u, std::list<state> &s);
  void   getPred(state u, std::list<state> &s);
  double  cost(state a, state b);
  bool   occupied(state u);
  bool   occupiedAndNoWall(state u);
  bool   isValid(state u);
  double  keyHashCode(state u);
  bool   isOntheWall(state u);
  float  calDistToWall(state u,int radius);

};









#endif