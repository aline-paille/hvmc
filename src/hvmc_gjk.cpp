#include "hvmc_math.h"
#include "hvmc_gjk.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <ctime>
using namespace std;

vec2 dist_ligne(vec2 pt, vertex ver){
  //calcul vecteur AB
  vec2 ab = ver.pts[1]-ver.pts[0];
  //normale au vecteur
  vec2 n=ab/(Length(ab));
  //Calcul de u et v 
  f32 v=Dot(pt-ver.pts[0],n)/Length(ab);
  f32 u=Dot(ver.pts[1]-pt,n)/Length(ab);
  //soit le poit le plus proche est a droite de A ou a droite de B
  if (u <= 0){
    return ver.pts[1];
  }
  if(v <= 0){
    return ver.pts[0];
  }
  //sinon le pt le plus proche est sur le segment AB et c'est une pondération des valeurs de A et de B
  return u*ver.pts[0] + v*ver.pts[1];
}


//fonction de conversion/test pour point_vertex_trimax
vec2 dist_point_vertex_trimax(vec2 pt, vertex ver){
  vec2 pp;
  vertex res=point_vertex_trimax(pt, ver, pp);
  switch(res.pts.size()){
    case 1 : return res.pts[0];
    case 2 : 
    {
    return dist_ligne(pt,res);
	
    }
    default : return {-1.f,-1.f};
  }
}

//fonction de calcul du support du point le plus proche entre un point et un autre point, un segment, un triangle
vertex point_vertex_trimax(vec2 pt, vertex ver, vec2& pp){
  vertex res;
  //selon la taille du vertex 
  switch(ver.pts.size()){
    case 1 : 
    {
      //si c'est un point on renvoie le point
      pp=ver.pts[0];
      return ver;
    }
    case 2 : {
      //si c'est un segment on utilise dist_ligne
      pp=dist_ligne(pt,ver);
      return ver;
    }
    case 3 : 
    {
      //si c'est un triangle
      //calcul des segments du triangle et de QA, QB, QC ou Q est le point dont on cherche a connaitre la distance au vertex
      vec2 ab = ver.pts[1]-ver.pts[0];
      vec2 bc = ver.pts[2]-ver.pts[1];
      vec2 ac = ver.pts[2]-ver.pts[0];
      vec2 qa = ver.pts[0]-pt;
      vec2 qb = ver.pts[1]-pt;
      vec2 qc = ver.pts[2]-pt;
      //calcul des normales
      vec2 nAB = ab/(Length(ab));
      vec2 nBC = bc/(Length(bc));
      vec2 nCA = -ac/(Length(ac));
      //calcul des aires
      f32 areaABC = Cross(ab,ac)/2.f;
      f32 areaQBC = Cross(qb,qc)/2.f;
      f32 areaQCA = Cross(qc,qa)/2.f;
      f32 areaQAB = Cross(qa,qb)/2.f;
      //Calcul des u,v pr chaque segment
      f32 vAB = Dot(pt-ver.pts[0],nAB)/Length(ab);
      f32 uAB = Dot(ver.pts[1]-pt,nAB)/Length(ab);
      f32 vBC = Dot(pt-ver.pts[1],nBC)/Length(bc);
      f32 uBC = Dot(ver.pts[2]-pt,nBC)/Length(bc);
      f32 vCA = Dot(pt-ver.pts[2],nCA)/Length(ac);
      f32 uCA = Dot(ver.pts[0]-pt,nCA)/Length(ac);
      //calcul de u,v,w pour le triangle
      f32 uABC = areaQBC/areaABC;
      f32 vABC = areaQCA/areaABC;
      f32 wABC = areaQAB/areaABC;
      //si le pt le plus proche est un point
      if(uAB <= 0 && vBC <= 0){
	res.pts.push_back(ver.pts[1]);
	pp=ver.pts[1];
	return res;
      }
      if(uBC <= 0 && vCA <= 0){
	res.pts.push_back(ver.pts[2]);
	pp=ver.pts[2];
	return res;
      }
      if(vAB <= 0 && uCA <= 0){
	res.pts.push_back(ver.pts[0]);
	pp=ver.pts[0];
	return res;
      }
      //si le point le plus proche est une ligne
      vertex ligne;
      if(uABC > 0 && vABC > 0 && wABC <= 0){
	ligne.pts.push_back(ver.pts[0]);
	ligne.pts.push_back(ver.pts[1]);
	pp=dist_ligne(pt,ligne);
	return ligne;
      }
      if(uABC > 0 && vABC <= 0 && wABC > 0){
	ligne.pts.push_back(ver.pts[0]);
	ligne.pts.push_back(ver.pts[2]);
	pp=dist_ligne(pt,ligne);
	return ligne;
      }
      if(uABC <= 0 && vABC > 0 && wABC > 0){
	ligne.pts.push_back(ver.pts[1]);
	ligne.pts.push_back(ver.pts[2]);
	pp=dist_ligne(pt,ligne);
	return ligne;
      }
      //sinon il est sur le triangle ou a l'interieur
      if(uABC > 0 && vABC > 0 && wABC > 0){
	res.pts.push_back(pt);
	pp=pt;
	return res;
      }
    }
    default : 
    {
      //si la taille du vecteur est mauvaise
      res.pts.push_back({-1.f,-1.f});
      pp={-1.f,-1.f};
      return res;
    }
  }
}

//Fonction de calcul du vecteur support d
int support(polygon &poly, vec2& d){
  int index =0;
  float max = Dot (d, poly.pts[index]);
  //calcul du point du polygone qui maximize le Dot
  for (unsigned int i=0; i<poly.pts.size(); i++){
    float val = Dot(d,poly.pts[i]);
    if(val > max){
      index = i;
      max=val;
    }
  }
  return index;
}


//Fonction de calcul de la distance entre un point et un polygone
vec2 gjk_pt_poly(vec2 pt, polygon poly){
  int r=0;
  vec2 d;
  vec2 pp;
  vertex test;
  vertex ver;
  vertex res;
  ver.pts.push_back(poly.pts[r]);
  int i_suiv;
  while(true){
    //calculer le pt le plus proche
    res = point_vertex_trimax(pt, ver, pp);
    //calculer le nouveau d
    if(res.pts.size()==2){
      vec2 a =  res.pts[0]-res.pts[1];
      d ={-a.y,a.x};
      if(Dot(d,pt-res.pts[0]) <0)
	d=-d;
    }
    else
      d = pt - pp; 
    if(d.x == 0.f && d.y == 0.f){
      return pt;
    }
    //enlever les pts inutiles
    ver=res;
    //trouver le pt a ajouter
    i_suiv=support(poly, d);
    for (unsigned int i=0; i<= ver.pts.size(); i++){
      vec2 pv = ver.pts[i]- poly.pts[i_suiv];
      if(pv.x == 0.f && pv.y == 0.f){
	return pp;
      }
    }
    //ajouter le pt
    ver.pts.push_back(poly.pts[i_suiv]);
  }
  return pp; 
}


//Fonction de calcul du vecteur support d augmentee avec la difference de Minkovski
vec2 support2(polygon &poly1, polygon &poly2, vec2& d){
  int index1 =0;
  int index2 =0;
  float max1 = Dot (d, poly1.pts[index1]);
  float max2 = Dot (-d, poly2.pts[index2]);
  //trouver le max dans la direction d pour poly1
  for (unsigned int i=0; i<poly1.pts.size(); i++){
    float val = Dot(d,poly1.pts[i]);
    if(val > max1){
      index1 = i;
      max1=val;
    }
  }
  //trouver le max dans la direction -d pour poly2
  for (unsigned int i=0; i<poly2.pts.size(); i++){
    float val = Dot(-d,poly2.pts[i]);
    if(val > max2){
      index2 = i;
      max2=val;
    }
  }
  return poly1.pts[index1] - poly2.pts[index2];
}


//gjk de polygone a polygone
vec2 gjk(polygon poly1, polygon poly2){
  int r=0;
  vec2 d;
  vec2 pp;
  vec2 pt = {0.f,0.f};
  vec2 pt_suiv;
  vertex ver;
  vertex res;
  ver.pts.push_back(poly1.pts[r]-poly2.pts[r]);
  while(true){
    //calculer le pt le plus proche
    res = point_vertex_trimax(pt, ver, pp);
    //calculer le nouveau d
    if(res.pts.size()==2){
      vec2 a =  res.pts[0]-res.pts[1];
      d ={-a.y,a.x};
      if(Dot(d,pt-res.pts[0]) <0)
	d=-d;
    }
    else
      d = pt - pp; 
    if(d.x == 0.f && d.y == 0.f){
      return pt;
    }
    //enlever les pts inutiles
    ver=res;
    //trouver le pt a ajouter
    pt_suiv=support2(poly1, poly2, d);
    for (unsigned int i=0; i<= ver.pts.size(); i++){
      vec2 pv = ver.pts[i]- pt_suiv;
      if(pv.x == 0.f && pv.y == 0.f){
	return pp;
      }
    }
    //ajouter le pt
    ver.pts.push_back(pt_suiv);
  }
  return pp; 
}

void test_dist_ligne(){
  cout << "Test de la fonction dist_ligne\n";
  vec2 pt = {2.f,2.f};
  vertex ver;
  vec2 res;
  ver.pts.push_back({2.f,4.f});
  ver.pts.push_back({5.f,2.f});
  res = dist_ligne(pt, ver);
  
  cout << "resultat attendu : ~3,3\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  pt = {4.f,1.f};
  res = dist_ligne(pt, ver);
  cout << "resultat attendu : ~5,2\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {7.f,0.f};
  res = dist_ligne(pt, ver);
  cout << "resultat attendu : 5,2\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {0.f,6.f};
  res = dist_ligne(pt, ver);
  cout << "resultat attendu : 2,4\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  cout << "Fin de test de la fonction dist_ligne\n";
}


void test_dist_point_vertex_trimax(){
  cout << "Test de la fonction dist point vertex trimax\n";
  vec2 pt;
  vertex ver;
  vec2 res;
  ver.pts.push_back({2.f,4.f});
  ver.pts.push_back({5.f,2.f});
  ver.pts.push_back({5.f,4.f});
  
  pt = {2.f,2.f};
  res=dist_point_vertex_trimax(pt, ver);
  cout << "resultat attendu : ~3,3\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {4.f,1.f};
  res=dist_point_vertex_trimax(pt, ver);
  cout << "resultat attendu : ~5,2\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {3.f,5.f};
  res=dist_point_vertex_trimax(pt, ver);
  cout << "resultat attendu : 3,4\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {6.f,3.f};
  res=dist_point_vertex_trimax(pt, ver);
  cout << "resultat attendu : 5,3\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {4.f,3.f};
  res=dist_point_vertex_trimax(pt, ver);
  cout << "resultat attendu : 4,3\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {0.f,5.f};
  res=dist_point_vertex_trimax(pt, ver);
  cout << "resultat attendu : ~2,4\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {6.f,1.f};
  res=dist_point_vertex_trimax(pt, ver);
  cout << "resultat attendu : ~5,2\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {6.f,5.f};
  res=dist_point_vertex_trimax(pt, ver);
  cout << "resultat attendu : ~5,4\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  cout << "Fin de test de la fonction dist point vertex trimax\n";

}


void test_gjk_pt_poly(){
  cout << "Test de la fonction gjk_pt_poly\n";
  vec2 pt;
  polygon poly;
  vec2 res;
  poly.pts.push_back({4.f,3.f});
  poly.pts.push_back({3.f,5.f});
  poly.pts.push_back({5.f,7.f});
  poly.pts.push_back({7.f,2.f});
  poly.pts.push_back({7.f,5.f});
  
  
  pt = {1.f,2.f};
  res=gjk_pt_poly(pt, poly);
  cout << "resultat attendu : ~4,3\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {5.f,1.f};
  res=gjk_pt_poly(pt, poly);
  cout << "resultat attendu : ~6,2\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {7.f,5.f};
  res=gjk_pt_poly(pt, poly);
  cout << "resultat attendu : 7,5\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {6.f,6.f};
  res=gjk_pt_poly(pt, poly);
  cout << "resultat attendu : ~6,6\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {5.f,5.f};
  res=gjk_pt_poly(pt, poly);
  cout << "resultat attendu : ~5,5\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  pt = {8.f,1.f};
  res=gjk_pt_poly(pt, poly);
  cout << "resultat attendu : 7,2\n";
  cout << "resultat obtenu : " << res.x << ", " << res.y  << "\n";
  
  cout << "Fin de test de la fonction gjk_pt_poly";
}

void test_gjk(){
  cout << "Test de la fonction gjk\n";
  polygon poly1;
  polygon poly2;
  vec2 res;
  poly1.pts.push_back({4.f,3.f});
  poly1.pts.push_back({3.f,5.f});
  poly1.pts.push_back({5.f,7.f});
  poly1.pts.push_back({7.f,2.f});
  poly1.pts.push_back({7.f,5.f});
  
  cout << "test polygones séparés : \n";
  
  poly2.pts.push_back({0.f,0.f});
  poly2.pts.push_back({0.f,2.f});
  poly2.pts.push_back({2.f,2.f});
  poly2.pts.push_back({2.f,0.f});
  res = gjk(poly1, poly2);
  cout << "resultat attendu : pas de collision\n";
  if(res.x == 0.f && res.y == 0.f){
    cout << "resultat obtenu : collision \n";
  }else
    cout << "resultat obtenu : pas de collision \n";
  poly2.pts.clear();
  
  cout << "\ntest polygones en contact :\n";
  
  poly2.pts.push_back({7.f,3.f});
  poly2.pts.push_back({9.f,5.f});
  poly2.pts.push_back({10.f,3.f});
  res = gjk(poly1, poly2);
  cout << "resultat attendu : collision\n";
  if(res.x == 0.f && res.y == 0.f){
    cout << "resultat obtenu : collision \n";
  }else
    cout << "resultat obtenu : pas de collision \n";
  poly2.pts.clear();
  

  cout << "\ntest polygones en inclusion :\n";
  
  
  poly2.pts.push_back({3.f,6.f});
  poly2.pts.push_back({5.f,10.f});
  poly2.pts.push_back({7.f,6.f});
  res = gjk(poly1, poly2);
  cout << "resultat attendu : collision\n";
  if(res.x == 0.f && res.y == 0.f){
    cout << "resultat obtenu : collision \n";
  }else
    cout << "resultat obtenu : pas de collision \n";
  poly2.pts.clear();
  
  cout << "Fin de test de la fonction gjk";
}

