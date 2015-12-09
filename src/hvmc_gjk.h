#include "hvmc_math.h"
#include <vector>
#ifndef hvmc_gjk_h
#define hvmc_gjk_h
using namespace std;


//vertex : point, segment ou triangle
struct vertex{
  vector<vec2> pts;
  
};


//polygone : vecteur de points ordonnés
struct polygon{
  
  polygon() {};
  ~polygon() {};
  
  vector<vec2> pts;
  int nb;
};


vertex point_vertex_trimax(vec2 pt, vertex ver, vec2& pp);
//tester si le vecteur est nul pour savoir si il y a collision
vec2 gjk(polygon poly1, polygon poly2);



//fonctions de test de gjk et des sous-fonctions
void test_gjk();
void test_gjk_pt_poly();
void test_dist_ligne();
void test_dist_point_vertex_trimax();
#endif
