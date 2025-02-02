// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_MATERIAL_COLORS_H
#define IGL_MATERIAL_COLORS_H
#include <Eigen/Core>
// Define constant material colors for use with opengl glMaterialfv
// Most of these colors come from IGL publications
namespace igl
{
  // Gold/Silver used in BBW/MONO/STBS/FAST
  const float GOLD_AMBIENT[4] =   {  51.0/255.0, 43.0/255.0,33.3/255.0,1.0f };
  const float GOLD_DIFFUSE[4] =   { 255.0/255.0,228.0/255.0,58.0/255.0,1.0f };
  const float GOLD_SPECULAR[4] =  { 255.0/255.0,235.0/255.0,80.0/255.0,1.0f };
  const float SILVER_AMBIENT[4] = { 0.2f, 0.2f, 0.2f, 1.0f };
  const float SILVER_DIFFUSE[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
  const float SILVER_SPECULAR[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
  const float BLUE_DIFFUSE[4] = { 29.0f/ 255.0, 48.0f / 255.0, 102.0f / 255.0, 1.0f };

  const float obs_ambient[4] = { 0.05375f, 0.05f, 0.06625f, 0.82f };
  const float obs_diffuse[4] = { 0.18275f, 0.17f, 0.22525f, 0.82f };
  const float obs_specular[4] = { 0.332741f, 0.328634f, 0.346435f, 0.82f};


  const float rubber_ambient[4]= { 0.05f, 0.05f, 0.00f, 1.0f };
  const float rubber_diffuse[4]= { 0.5f,0.5f,0.4f,1.0f };
  const float rubber_specular[4] = { 0.7f,0.7f,0.04f,1.0f };

  const float rubby_ambient[4] = { 0.1745f, 0.01175f, 0.01175f, 0.55f };
  const float rubby_diffuse[4] = { 0.61424f, 0.04136f, 0.04136f, 0.55f };
  const float rubby_specular[4] = { 0.727811f, 0.626959f, 0.626959f, 0.55f };

  //Green plastic
  float grass_mat_ambient[4] = { 0.0f,0.0f,0.0f,1.0f };
  float grass_mat_diffuse[4] = { 0.1f,0.35f,0.1f,1.0f };
  float grass_mat_specular[4] = { 0.45f,0.55f,0.45f,1.0f };
  float shine = 32.0f;


  //Silver
  float grey_mat_ambient[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
  float grey_mat_diffuse[4] = { 0.50754f, 0.50754f, 0.50754f, 1.0f };
  float grey_mat_specular[4] = { 0.508273f, 0.508273f, 0.508273f, 1.0f };

  // Blue/Cyan more similar to Jovan Popovic's blue than to Mario Botsch's blue
  const float CYAN_AMBIENT[4] =   {  59.0/255.0, 68.0/255.0,255.0/255.0,1.0f };
  const float CYAN_DIFFUSE[4] =   {  94.0/255.0,185.0/255.0,238.0/255.0,1.0f };
  const float CYAN_SPECULAR[4] =   { 163.0/255.0,221.0/255.0,255.0/255.0,1.0f };
  const float DENIS_PURPLE_DIFFUSE[4] =   { 80.0/255.0,64.0/255.0,255.0/255.0,1.0f };
  const float LADISLAV_ORANGE_DIFFUSE[4] = {1.0f, 125.0f / 255.0f, 19.0f / 255.0f, 0.0f};
  // FAST armadillos colors
  const float FAST_GREEN_DIFFUSE[4] = { 113.0f/255.0f, 239.0f/255.0f,  46.0f/255.0f, 1.0f};
  const float FAST_RED_DIFFUSE[4]   = { 255.0f/255.0f,  65.0f/255.0f,  46.0f/255.0f, 1.0f};
  const float FAST_BLUE_DIFFUSE[4]  = { 106.0f/255.0f, 106.0f/255.0f, 255.0f/255.0f, 1.0f};
  const float FAST_GRAY_DIFFUSE[4]  = { 150.0f/255.0f, 150.0f/255.0f, 150.0f/255.0f, 1.0f};
  // Basic colors
  const float WHITE[4] =   { 255.0/255.0,255.0/255.0,255.0/255.0,1.0f };
  const float BLACK[4] =   { 0.0/255.0,0.0/255.0,0.0/255.0,1.0f };
  const float WHITE_AMBIENT[4] =   { 255.0/255.0,255.0/255.0,255.0/255.0,1.0f };
  const float WHITE_DIFFUSE[4] =   { 255.0/255.0,255.0/255.0,255.0/255.0,1.0f };
  const float WHITE_SPECULAR[4] =  { 255.0/255.0,255.0/255.0,255.0/255.0,1.0f };
  const float BBW_POINT_COLOR[4] = {239./255.,213./255.,46./255.,255.0/255.0};
  const float BBW_LINE_COLOR[4] = {106./255.,106./255.,255./255.,255./255.};
  const float MIDNIGHT_BLUE_DIFFUSE[4]  = { 21.0f/255.0f, 27.0f/255.0f, 84.0f/255.0f, 1.0f};
  // Winding number colors
  const float EASTER_RED_DIFFUSE[4] = {0.603922,0.494118f,0.603922f,1.0f};
  const float WN_OPEN_BOUNDARY_COLOR[4] = {154./255.,0./255.,0./255.,1.0f};
  const float WN_NON_MANIFOLD_EDGE_COLOR[4] = {201./255., 51./255.,255./255.,1.0f};
  const Eigen::Vector4f 
    MAYA_GREEN(128./255.,242./255.,0./255.,1.),
    MAYA_YELLOW(255./255.,247./255.,50./255.,1.),
    MAYA_RED(234./255.,63./255.,52./255.,1.),
    MAYA_BLUE(0./255.,73./255.,252./255.,1.),
    MAYA_PURPLE(180./255.,73./255.,200./255.,1.),
    MAYA_VIOLET(31./255.,15./255.,66./255.,1.),
    MAYA_GREY(0.5,0.5,0.5,1.0),
    MAYA_CYAN(131./255.,219./255.,252./255.,1.),
    MAYA_SEA_GREEN(70./255.,252./255.,167./255.,1.);
}
#endif
