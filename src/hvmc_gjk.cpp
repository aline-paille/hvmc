#include "hvmc_math.h"
#include "hvmc_gjk.h"
#include <math.h>
#include <vector>
#include <iostream>
using namespace std;

vec2 dist_ligne(vec2 pt, vertex ver){
  vec2 ab = ver.pts[1]-ver.pts[0];
  vec2 n=ab/(Length(ab));
  f32 v=Dot(pt-ver.pts[0],n)/Length(ab);
  //cout << "u : " << u;
  f32 u=Dot(ver.pts[1]-pt,n)/Length(ab);
  //cout << "v : " << v;
  if (u <= 0){
    //cout << "\nA\n";
    return ver.pts[1];
  }
  if(v <= 0){
    //cout << "\nB\n";
    return ver.pts[0];
  }
  //cout << "\nmélange\n";
  return u*ver.pts[0] + v*ver.pts[1];
}

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


vertex point_vertex_trimax(vec2 pt, vertex ver, vec2& pp){
  vertex res;
  switch(ver.pts.size()){
    case 1 : 
    {
      pp=ver.pts[0];
      return ver;
    }
    case 2 : {
      pp=dist_ligne(pt,ver);
      return ver;
    }
    case 3 : 
    {
      vec2 ab = ver.pts[1]-ver.pts[0];
      cout << "\n" << ab.x << ", " << ab.y << "\n";
      vec2 bc = ver.pts[2]-ver.pts[1];
      vec2 ac = ver.pts[2]-ver.pts[0];
      vec2 qa = ver.pts[0]-pt;
      vec2 qb = ver.pts[1]-pt;
      vec2 qc = ver.pts[2]-pt;
      vec2 nAB = ab/(Length(ab));
      vec2 nBC = bc/(Length(bc));
      vec2 nCA = -ac/(Length(ac));
      f32 areaABC = Cross(ab,ac)/2.f;
      f32 areaQBC = Cross(qb,qc)/2.f;
      f32 areaQCA = Cross(qc,qa)/2.f;
      f32 areaQAB = Cross(qa,qb)/2.f;
      cout << "\n cross(ab,ac) " << Cross(ab,ac) << "\n";
      f32 vAB = Dot(pt-ver.pts[0],nAB)/Length(ab);
      f32 uAB = Dot(ver.pts[1]-pt,nAB)/Length(ab);
      f32 vBC = Dot(pt-ver.pts[1],nBC)/Length(bc);
      f32 uBC = Dot(ver.pts[2]-pt,nBC)/Length(bc);
      f32 vCA = Dot(pt-ver.pts[2],nCA)/Length(ac);
      f32 uCA = Dot(ver.pts[0]-pt,nCA)/Length(ac);
      f32 uABC = areaQBC/areaABC;
      cout << "\n" << uABC << "\n";
      f32 vABC = areaQCA/areaABC;
      cout << "\n" << vABC << "\n";
      f32 wABC = areaQAB/areaABC;
      cout << "\n" << wABC << "\n";
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
      vertex ligne;
      if(uABC > 0 && vABC > 0 && wABC <= 0){
	cout << "w\n";
	ligne.pts.push_back(ver.pts[0]);
	ligne.pts.push_back(ver.pts[1]);
	pp=dist_ligne(pt,ligne);
	return ligne;
      }
      if(uABC > 0 && vABC <= 0 && wABC > 0){
	cout << "v\n";
	ligne.pts.push_back(ver.pts[0]);
	ligne.pts.push_back(ver.pts[2]);
	pp=dist_ligne(pt,ligne);
	return ligne;
      }
      if(uABC <= 0 && vABC > 0 && wABC > 0){
	cout << "u\n";
	ligne.pts.push_back(ver.pts[1]);
	ligne.pts.push_back(ver.pts[2]);
	pp=dist_ligne(pt,ligne);
	return ligne;
      }
      if(uABC > 0 && vABC > 0 && wABC > 0){
	res.pts.push_back(pt);
	pp=pt;
	return res;
      }
    }
    default : 
    {
      res.pts.push_back({-1.f,-1.f});
      pp={-1.f,-1.f};
      return res;
    }
  }
}


int support(polygon &poly, vec2& d){
  int index =0;
  float max = Dot (d, poly.pts[index]);
  for (unsigned int i=0; i<poly.pts.size(); i++){
    float val = Dot(d,poly.pts[i]);
    if(val > max){
      index = i;
      max=val;
    }
  }
  return index;
}

vec2 gjk(vec2 pt, polygon poly){
  int r=rand()* poly.pts.size();
  vec2 d=pt;
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
    //enlever les pts inutiles
    ver=res;
    //trouver le pt a ajouter
    i_suiv=support(poly, d);
    //ajouter le pt
    ver.pts.push_back(poly.pts[i_suiv]);
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