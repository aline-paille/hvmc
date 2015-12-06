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


//tester si le vecteur est nul pour savoir si il y a collision
vec2 gjk(polygon poly1, polygon poly2);
void test_gjk();
void test_gjk_pt_poly();
void test_dist_ligne();
void test_dist_point_vertex_trimax();
#endif
