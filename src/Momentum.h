#ifndef __MOMENTUM_H__
#define __MOMENTUM_H__

#include "bulletSimulation.h"
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <vector>

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.
   
   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/lgpl.html>.
   
   Author:     Helge Mathee      helge.mathee@gmx.net
   Company:    Studio Nest (TM)
   Date:       2010 / 09 / 21
*/

class mesh_UD
{
    public:
       rbdID rbd;
       rbdID shape;

       int nbPoints;
       int nbTriangles;
       int nbEdges;
       int nbIslands;
       int * nbPerIsland;
       int ** islands;
       btAlignedObjectArray<btVector3> offset;

       mesh_UD()
       {
          nbPoints = nbTriangles = nbEdges = nbIslands = 0;
          nbPerIsland = NULL;
          islands = NULL;
       }

       void Clear()
       {
          if(islands != NULL)
          {
             for(int i=0;i<nbIslands;i++)
                free(islands[i]);
             free(islands);
             islands = NULL;
          }
          if(nbPerIsland!=NULL)
          {
             free(nbPerIsland);
             nbPerIsland = NULL;
          }
          nbPoints = nbTriangles = nbEdges = nbIslands = 0;

          offset.clear();
       }
};


class cons_UD
{
public:
   rbdID cons;
};

class kine_UD
{
public:
   rbdID rbd;
   rbdID shape;
};

#endif
