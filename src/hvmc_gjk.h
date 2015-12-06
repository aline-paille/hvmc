#include "hvmc_math.h"
#include <vector>
#ifndef hvmc_gjk_h
using namespace std;

struct vertex{
  vector<vec2> pts;
  
};

struct polygon{
  vector<vec2> pts;
  int nb;
};

vec2 dist_ligne(vec2 pt, vertex ver);
vertex point_vertex_trimax(vec2 pt, vertex ver, vec2& pp);
vec2 dist_point_vertex_trimax(vec2 pt, vertex ver);
void test_dist_ligne();
void test_dist_point_vertex_trimax();
#endif
