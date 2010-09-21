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

#include <xsi_application.h>

#include <xsi_context.h>

#include <xsi_pluginregistrar.h>

#include <xsi_status.h>

#include <xsi_customoperator.h>

#include <xsi_operatorcontext.h>

#include <xsi_ppglayout.h>

#include <xsi_ppgeventcontext.h>

#include <xsi_selection.h>

#include <xsi_command.h>

#include <xsi_factory.h>
#include <xsi_primitive.h>
#include <xsi_kinematics.h>
#include <xsi_outputport.h>
#include <xsi_polygonmesh.h>
#include <xsi_x3dobject.h>
#include <xsi_iceattribute.h>
#include <xsi_command.h>
#include <xsi_math.h>
#include <xsi_uitoolkit.h>
#include <xsi_progressbar.h>
#include <xsi_geometryaccessor.h>
#include <xsi_triangle.h>
#include <xsi_vertex.h>
#include <xsi_edge.h>
#include <xsi_polygonface.h>
#include <xsi_null.h>
#include <xsi_model.h>
#include <xsi_icenode.h>
#include <vector>

#include "Momentum.h"
#include "Plotting.h"
#include "bulletSimulation.h"
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

using namespace XSI;
using namespace XSI::MATH;

XSIPLUGINCALLBACK CStatus MomentumDeform_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;

   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();

   oPDef = oFactory.CreateParamDef(L"frame",CValue::siInt4,siAnimatable | siPersistable,L"frame",L"frame",1,-100000,100000,0,1);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"state",CValue::siInt4,siAnimatable | siPersistable,L"state",L"state",DISABLE_DEACTIVATION,0,1000,0,1000);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"mode",CValue::siInt4,siAnimatable | siPersistable,L"mode",L"mode",4,0,1000,0,1000);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"mass",CValue::siFloat,siAnimatable | siPersistable,L"mass",L"mass",1,0,10000000,0,1);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"margin",CValue::siFloat,siAnimatable | siPersistable,L"margin",L"margin",0,-100,100,-.1,.1);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"friction",CValue::siFloat,siPersistable,L"friction",L"friction",.9,0,1000,0,1);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"bounce",CValue::siFloat,siPersistable,L"bounce",L"bounce",0,0,1000,0,1);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linThreshold",CValue::siFloat,siAnimatable | siPersistable,L"linThreshold",L"linThreshold",0.8f,0.0f,1000.0f,0.0f,0.01f);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angThreshold",CValue::siFloat,siAnimatable | siPersistable,L"angThreshold",L"angThreshold",1.0f,0.0f,1000.0f,0.0f,0.01f);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linDamping",CValue::siFloat,siAnimatable | siPersistable,L"linDamping",L"linDamping",0.3f,0.0f,1000.0f,0.0f,0.01f);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angDamping",CValue::siFloat,siAnimatable | siPersistable,L"angDamping",L"angDamping",0.3f,0.0f,1000.0f,0.0f,0.01f);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"plotted",CValue::siBool,siPersistable,L"plotted",L"plotted",0,0,1,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oCustomOperator.PutAlwaysEvaluate(false);

   oCustomOperator.PutDebug(0);

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumDeform_DefineLayout( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );

   PPGLayout oLayout;

   PPGItem oItem;

   oLayout = ctxt.GetSource();

   oLayout.Clear();
   CValueArray stateItems(6);
   stateItems[0] = L"Active";
   stateItems[1] = (LONG)DISABLE_DEACTIVATION;
   stateItems[2] = L"Sleeping";
   stateItems[3] = (LONG)WANTS_DEACTIVATION;
   stateItems[4] = L"Disabled";
   stateItems[5] = (LONG)DISABLE_SIMULATION;
   oLayout.AddEnumControl(L"state",stateItems,L"Initial State");
   CValueArray modeItems(6);
   modeItems[0] = L"Box Shape";
   modeItems[1] = (LONG)0l;
   modeItems[2] = L"Convex Hull Shape";
   modeItems[3] = (LONG)4l;
   modeItems[4] = L"GImpact Actual Shape";
   modeItems[5] = (LONG)25l;
   oLayout.AddEnumControl(L"mode",modeItems,L"Mode");
   oLayout.AddItem(L"mass",L"Mass");
   oLayout.AddItem(L"margin",L"Margin");
   oLayout.AddItem(L"friction",L"Friction");
   oLayout.AddItem(L"bounce",L"Bounce");
   oLayout.AddItem(L"linDamping",L"Linear Damping");
   oLayout.AddItem(L"angDamping",L"Angular Damping");

   oLayout.AddGroup(L"Sleeping Mode Controls");
   oLayout.AddItem(L"linThreshold",L"Linear Threshold");
   oLayout.AddItem(L"angThreshold",L"Angular Threshold");
   oLayout.EndGroup();

   oLayout.AddItem(L"plotted",L"Plotted");

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus apply_MomentumDeform_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of MomentumDeform operator");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_MomentumDeform_Execute( CRef& in_ctxt )
{

   Context ctxt( in_ctxt );
   Selection l_pSelection = Application().GetSelection();

   // search the selection for any compatible 3d object

   for(long i=0;i<l_pSelection.GetCount();i++)
   {
      X3DObject l_pSelObj(l_pSelection.GetItem(i));

      if(l_pSelObj.GetType().IsEqualNoCase(L"polymsh"))
      {
         // add the plotting data
         UserDataBlob plotBlob;
         l_pSelObj.AddProperty( L"UserDataBlob", false, L"plottingData", plotBlob) ;

         // create the operator
         CustomOperator newOp = Application().GetFactory().CreateObject(L"MomentumDeform");
         newOp.AddOutputPort(l_pSelObj.GetActivePrimitive().GetRef());
         newOp.AddInputPort(l_pSelObj.GetActivePrimitive().GetRef());
         newOp.AddInputPort(plotBlob.GetRef());
         newOp.Connect();

         // set the expression for the frame!
         CValueArray args(2);
         CValue returnVal;
         args[0] = newOp.GetFullName()+L".frame";
         args[1] = L"fc";
         Application().ExecuteCommand(L"SetExpr",args,returnVal);
      }
      else
      {
         Application().LogMessage(L"[MOMENTUM] Object "+l_pSelObj.GetFullName()+L" is not valid for deforming.",siWarningMsg);
      }

   }

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumDeform_Update( CRef& in_ctxt )
{
   // check if we have a simulation
   if(gSimulation == NULL)
      return CStatus::OK;

   OperatorContext ctxt( in_ctxt );

   // query some of the parameters
   // we do that here so we don't have to it in the loop several times
   float margin = ctxt.GetParameterValue(L"margin");
   float mass = ctxt.GetParameterValue(L"mass");
   float friction = ctxt.GetParameterValue(L"friction");
   float bounce = ctxt.GetParameterValue(L"bounce");
   float linTh = ctxt.GetParameterValue(L"linThreshold");
   float angTh = ctxt.GetParameterValue(L"angThreshold");
   float linDamping = ctxt.GetParameterValue(L"linDamping");
   float angDamping = ctxt.GetParameterValue(L"angDamping");
   int state = (LONG)ctxt.GetParameterValue(L"state");
   int frame = (LONG)ctxt.GetParameterValue(L"frame");
   frame--;
   bool plotted = (bool)ctxt.GetParameterValue(L"plotted");

   // get the geometry
   Primitive prim = (CRef)ctxt.GetInputValue(0);
   PolygonMesh mesh = prim.GetGeometry();
   CVertexRefArray points = mesh.GetVertices();
   CVector3Array pos = points.GetPositionArray();
   CEdgeRefArray edges = mesh.GetEdges();
   CTriangleRefArray triangles = mesh.GetTriangles();

   // get the dbID and the rigid body for it
   CValue udVal = ctxt.GetUserData();
   mesh_UD * p = (mesh_UD*)(CValue::siPtrType)udVal;
   if(p==NULL)
      return CStatus::Unexpected;

   // ok, we have the user data
   // let's ensure that we need to build it
   if(frame == 0 || p->nbPoints != (int)points.GetCount() || p->nbEdges != (int)edges.GetCount() || p->nbTriangles != (int)triangles.GetCount() || p->nbIslands==0)
   {
      int mode = (LONG)ctxt.GetParameterValue(L"mode");

      // first, let's clear the userdata
      p->Clear();

      // now let's remove all collision shapes and all rigidbodies of this primary index
      gSimulation->DeleteAllCollisionShapes(p->shape);
      gSimulation->DeleteAllRigidBodies(p->rbd);

      // create an array to mark which
      LONG pointsLeft = points.GetCount();
      CLongArray pointsDone(pointsLeft);
      for(LONG i=0;i<pointsDone.GetCount();i++)
         pointsDone[i] = 0;

      // create LONG arrays for the results
      CLongArray islandCount;
      CLongArray islandIndices;
      CLongArray currentIsland;
      LONG currentIslandCount = 0;

      // now walk all polygons and try to find the islands
      while(pointsLeft>0)
      {
         // if the current island is empty
         // find the first unused point!
         if(currentIslandCount==0)
         {
            for(LONG i=0;i<pointsDone.GetCount();i++)
            {
               if(pointsDone[i]==0)
               {
                  pointsDone[i] = 1;
                  currentIsland.Add(i);
                  islandIndices.Add(i);
                  currentIslandCount++;
                  pointsLeft--;
                  break;
               }
            }
         }

         // now as we have a current island to search, let's do this
         CLongArray newIsland;
         for(LONG i=0;i<currentIsland.GetCount();i++)
         {
            CLongArray neighbors = Vertex(points[currentIsland[i]]).GetNeighborVertices().GetIndexArray();
            for(LONG j=0;j<neighbors.GetCount();j++)
            {
               if(pointsDone[neighbors[j]]==0)
               {
                  pointsDone[neighbors[j]] = 1;
                  newIsland.Add(neighbors[j]);
                  islandIndices.Add(neighbors[j]);
                  currentIslandCount++;
                  pointsLeft--;
               }
            }
         }

         // now, if there was no island more, we can close this island
         if(newIsland.GetCount() == 0 || pointsLeft == 0)
         {
            islandCount.Add(currentIslandCount);
            currentIslandCount = 0;
         }
         else
         {
            // otherwise, keep going!
            currentIsland = newIsland;
         }
      }

      // we have the islands, let's construct the ud
      p->nbPoints = (int)points.GetCount();
      p->nbEdges = (int)edges.GetCount();
      p->nbTriangles = (int)triangles.GetCount();
      p->nbIslands = (int)islandCount.GetCount();

      // allocate the memory and copy the islandindex data
      p->nbPerIsland = (int*)malloc(sizeof(int)*p->nbIslands);
      p->islands = (int**)malloc(sizeof(int*)*p->nbIslands);
      LONG offset =0;
      for(int i=0;i<p->nbIslands;i++)
      {
         p->nbPerIsland[i] = (int)islandCount[i];
         p->islands[i] = (int*)malloc(sizeof(int)*p->nbPerIsland[i]);
         for(int j=0;j<p->nbPerIsland[i];j++)
            p->islands[i][j] = islandIndices[offset++];
      }

      // now we have to construct the collision shapes
      LONG nbCreated = 0;
      if(mode == 0) // btBoxShape
      {
         for(int i=0;i<p->nbIslands;i++)
         {
            // create an id for this
            rbdID shape_id;
            shape_id.primary = p->shape.primary;
            shape_id.secondary = i;

            // create a new shape reference
            btCollisionShapeReference * shapeRef = new btCollisionShapeReference();
            shapeRef->op = ctxt.GetSource();
            shapeRef->prim = prim.GetRef();

            // compute the bbox
            btVector3 bbMin(100000,100000,100000);
            btVector3 bbMax(-100000,-100000,-100000);

            for(int j=0;j<p->nbPerIsland[i];j++)
            {
               // merge the bbox
               CVector3 p1(pos[p->islands[i][j]]);
               if(p1.GetX() < bbMin.getX())bbMin.setX(p1.GetX());
               if(p1.GetY() < bbMin.getY())bbMin.setY(p1.GetY());
               if(p1.GetZ() < bbMin.getZ())bbMin.setZ(p1.GetZ());
               if(p1.GetX() > bbMax.getX())bbMax.setX(p1.GetX());
               if(p1.GetY() > bbMax.getY())bbMax.setY(p1.GetY());
               if(p1.GetZ() > bbMax.getZ())bbMax.setZ(p1.GetZ());
            }

            // push back the bbox offset
            btVector3 scale;
            scale = (bbMax - bbMin) * .5;
            bbMin += bbMax;
            bbMin *= 0.5f;
            p->offset.push_back(bbMin);

            if(!plotted)
            {
               shapeRef->shape = new btBoxShape(scale);

               // let's insert the shape into the simulation
               gSimulation->AddCollisionShape(shape_id,shapeRef);
               nbCreated++;
            }
         }
      }
      else if(mode == 4) // btConvexHullShape
      {
         // convex hull shapes!
         for(int i=0;i<p->nbIslands;i++)
         {
            // create an id for this
            rbdID shape_id;
            shape_id.primary = p->shape.primary;
            shape_id.secondary = i;

            // create a new shape reference
            btCollisionShapeReference * shapeRef = new btCollisionShapeReference();
            shapeRef->op = ctxt.GetSource();
            shapeRef->prim = prim.GetRef();

            // compute the bbox
            btVector3 bbMin(100000,100000,100000);
            btVector3 bbMax(-100000,-100000,-100000);

            for(int j=0;j<p->nbPerIsland[i];j++)
            {
               // merge the bbox
               CVector3 p1(pos[p->islands[i][j]]);
               if(p1.GetX() < bbMin.getX())bbMin.setX(p1.GetX());
               if(p1.GetY() < bbMin.getY())bbMin.setY(p1.GetY());
               if(p1.GetZ() < bbMin.getZ())bbMin.setZ(p1.GetZ());
               if(p1.GetX() > bbMax.getX())bbMax.setX(p1.GetX());
               if(p1.GetY() > bbMax.getY())bbMax.setY(p1.GetY());
               if(p1.GetZ() > bbMax.getZ())bbMax.setZ(p1.GetZ());
            }

            // push back the bbox offset
            bbMin += bbMax;
            bbMin *= 0.5f;
            p->offset.push_back(bbMin);

            if(!plotted)
            {
               CVector3 center(bbMin.getX(),bbMin.getY(),bbMin.getZ());

               btConvexHullShape * complexShape = new btConvexHullShape();
               int offset = 0;
               for(int j=0;j<p->nbPerIsland[i];j++)
               {
                  CVector3 pos1 = pos[p->islands[i][j]];
                  pos1.SubInPlace(center);
                  complexShape->addPoint(btVector3(pos1.GetX(),pos1.GetY(),pos1.GetZ()));
               }

               //This optmization fails for low poly objects, so we'll remove it for now.
               // now create the convex hull shape
               /*
               // now create the convex hull shape
               btShapeHull * hullUtility = new btShapeHull(complexShape);
               hullUtility->buildHull(complexShape->getMargin());
               // copy from the simpler shape into a new hull
               btConvexHullShape * simpleShape = new btConvexHullShape();
               for(int j=0;j<hullUtility->numVertices();j++)
                  simpleShape->addPoint(hullUtility->getVertexPointer()[j]);

               delete hullUtility;
               delete complexShape;

               shapeRef->shape = simpleShape;
               */
               complexShape->setMargin(margin);
               shapeRef->shape = complexShape;

               // let's insert the shape into the simulation
               gSimulation->AddCollisionShape(shape_id,shapeRef);
               nbCreated++;
            }
         }
      }
      else if(mode == 25) // btGImpactMeshShape
      {
         // first let's find out which point belongs to which island
         // while we do that we can also merge the bbox
         CLongArray islandTriangleCount(p->nbIslands);
         CLongArray pointIslandIndex(points.GetCount());
         CLongArray pointIslandPoint(points.GetCount());
         for(int i=0;i<p->nbIslands;i++)
         {
            islandTriangleCount[i] = 0;

            btVector3 bbMin(100000,100000,100000);
            btVector3 bbMax(-100000,-100000,-100000);

            for(int j=0;j<p->nbPerIsland[i];j++)
            {
               pointIslandIndex[p->islands[i][j]] = (LONG)i;
               pointIslandPoint[p->islands[i][j]] = (LONG)j;

               // merge the bbox
               CVector3 p1(pos[p->islands[i][j]]);
               if(p1.GetX() < bbMin.getX())bbMin.setX(p1.GetX());
               if(p1.GetY() < bbMin.getY())bbMin.setY(p1.GetY());
               if(p1.GetZ() < bbMin.getZ())bbMin.setZ(p1.GetZ());
               if(p1.GetX() > bbMax.getX())bbMax.setX(p1.GetX());
               if(p1.GetY() > bbMax.getY())bbMax.setY(p1.GetY());
               if(p1.GetZ() > bbMax.getZ())bbMax.setZ(p1.GetZ());
            }

            // push back the bbox offset
            bbMin += bbMax;
            bbMin *= 0.5f;
            p->offset.push_back(bbMin);
         }

         if(!plotted)
         {
            // now let's loop over all triangles
            // and find out which island the triangle belongs to
            CLongArray triangleIslandIndex(triangles.GetCount());
            for(LONG i=0;i<triangles.GetCount();i++)
            {
               triangleIslandIndex[i] = pointIslandIndex[Triangle(triangles[i]).GetIndexArray()[0]];
               islandTriangleCount[triangleIslandIndex[i]]++;
            }

            // great, now since we know the triangles we can create the collision and the rigid bodies
            for(int i=0;i<p->nbIslands;i++)
            {
               // create an id for this
               rbdID shape_id;
               shape_id.primary = p->shape.primary;
               shape_id.secondary = i;

               // create a new shape reference
               btCollisionShapeReference * shapeRef = new btCollisionShapeReference();
               shapeRef->op = ctxt.GetSource();
               shapeRef->prim = prim.GetRef();

               // allocate the memory
               shapeRef->triId = (int*)malloc(sizeof(int)*islandTriangleCount[i]*3);
               shapeRef->triPos = (btScalar*)malloc(sizeof(btScalar)*p->nbPerIsland[i]*3);

               // copy all indices of all triangles for this island
               int offset = 0;
               for(LONG j=0;j<triangleIslandIndex.GetCount();j++)
               {
                  // if this triangle is not in this island
                  if(triangleIslandIndex[j]!=(LONG)i)
                     continue;
                  CLongArray pnts = Triangle(triangles[j]).GetIndexArray();
                  shapeRef->triId[offset++] = pointIslandPoint[pnts[0]];
                  shapeRef->triId[offset++] = pointIslandPoint[pnts[1]];
                  shapeRef->triId[offset++] = pointIslandPoint[pnts[2]];
               }

               // now copy all of the data for the points!
               offset = 0;
               for(int j=0;j<p->nbPerIsland[i];j++)
               {
                  CVector3 pos1 = pos[p->islands[i][j]];
                  shapeRef->triPos[offset++] = (btScalar)pos1.GetX() - p->offset[i].getX();
                  shapeRef->triPos[offset++] = (btScalar)pos1.GetY() - p->offset[i].getY();
                  shapeRef->triPos[offset++] = (btScalar)pos1.GetZ() - p->offset[i].getZ();
               }

               shapeRef->tris = new btTriangleIndexVertexArray(
                  islandTriangleCount[i],shapeRef->triId,3*sizeof(int),
                  p->nbPerIsland[i], shapeRef->triPos,3*sizeof(btScalar));

               btGImpactMeshShape * trimesh = new btGImpactMeshShape(shapeRef->tris);
               trimesh->updateBound();

               shapeRef->shape = trimesh;

               // let's insert the shape into the simulation
               gSimulation->AddCollisionShape(shape_id,shapeRef);
               nbCreated++;
            }
         }
      }

      if(!plotted)
         Application().LogMessage(L"[MOMENTUM] Created "+CValue(nbCreated).GetAsText()+L" collisionshapes for dbID "+CValue((LONG)p->shape.primary).GetAsText(),siVerboseMsg);
   }

   // if you are plotted, let's deform based on the deform data
   if(plotted)
   {
      CQuaternion q;
      CRotation r;
      CMatrix3 m;
      CVector3 t,offset;

      // let's retrieve the plotted data
      UserDataBlob udb(ctxt.GetInputValue(1));
      PLOT_UD plotData;
      udb.GetValue(plotData.buffer,plotData.size);
      if(plotData.size > 0)
      {
         // clamp frame
         if(frame < plotData.GetFrameIn())
            frame = plotData.GetFrameIn();
         else if(frame > plotData.GetFrameOut())
            frame = plotData.GetFrameOut();

         for(int i=0;i<p->nbIslands;i++)
         {
            // construct the id
            rbdID body_id;
            body_id.primary = p->rbd.primary;
            body_id.secondary = i;

            // let's skip invalid islands
            if(i >= plotData.GetNbTransforms())
               break;

            // get the transform from the plotdata
            btVector3 rbdPos = plotData.GetPosition(frame,i);
            btQuaternion rbdRot = plotData.GetRotation(frame,i);
            t.Set(rbdPos.getX(),rbdPos.getY(),rbdPos.getZ());
            q.Set(rbdRot.getW(),rbdRot.getX(),rbdRot.getY(),rbdRot.getZ());
            r.SetFromQuaternion(q);
            m = r.GetMatrix();
            offset.Set(p->offset[i].getX(),p->offset[i].getY(),p->offset[i].getZ());

            // now loop over all points of this island!
            for(int j=0;j<p->nbPerIsland[i];j++)
            {
               int index = p->islands[i][j];
               pos[index].SubInPlace(offset);
               pos[index].MulByMatrix3InPlace(m);
               pos[index].AddInPlace(t);
            }
         }
      }
   }
   else
   {
      // loop over all islands and try to create the rigid body
      LONG nbCreated = 0;
      for(int i=0;i<p->nbIslands;i++)
      {
         // create an id for this
         rbdID body_id;
         body_id.primary = p->rbd.primary;
         body_id.secondary = i;
         rbdID shape_id;
         shape_id.primary = p->shape.primary;
         shape_id.secondary = i;

         btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(body_id);
         if(bodyRef == NULL)
         {
            // if we have a shape, let's create the rigid body
            btCollisionShapeReference * shapeRef = gSimulation->GetCollisionShape(shape_id);
            if(shapeRef != NULL)
            {
               bodyRef = new btRigidBodyReference();

               // setup the transform on the object!
               btTransform transform;
               transform.setIdentity();
               transform.setOrigin(p->offset[i]);

               // create the default motion state
               btDefaultMotionState* motionState = new btDefaultMotionState(transform);

               // compute the inertia of the obkcet
               bodyRef->mass = mass;
               btVector3 inertia(0,0,0);
               if(bodyRef->mass > 0.0f)
                  shapeRef->shape->calculateLocalInertia(mass,inertia);

               // create the rbd construction info
               btRigidBody::btRigidBodyConstructionInfo rbInfo(bodyRef->mass,motionState,shapeRef->shape,inertia);

               // set some more things
               rbInfo.m_friction = friction;
               rbInfo.m_restitution = bounce;

               //add the body to the dynamics world
               bodyRef->body = new btRigidBody(rbInfo);

               //Set the sleeping thresholds
               bodyRef->body->setSleepingThresholds(linTh,angTh);

               //Set damping params
               bodyRef->body->setDamping(linDamping,angDamping);

               // set the activation state
               bodyRef->body->setActivationState(state);

               gSimulation->AddRigidBody(body_id,bodyRef);
               nbCreated++;
            }
         }
      }
      if(nbCreated>0)
         Application().LogMessage(L"[MOMENTUM] Created "+CValue(nbCreated).GetAsText()+L" rigidbodies for dbID "+CValue((LONG)p->rbd.primary).GetAsText(),siVerboseMsg);

      // now as we have all of the rigid bodies
      // we can actually deform this!
      if(mass != 0.0f && frame != 0)
      {
         // fantastic!
         CQuaternion q;
         CRotation r;
         CMatrix3 m;
         CVector3 t,offset;

         // loop over all islands
         for(int i=0;i<p->nbIslands;i++)
         {
            // construct the id
            rbdID body_id;
            body_id.primary = p->rbd.primary;
            body_id.secondary = i;

            // try to find the rbd for this
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(body_id);
            if(bodyRef == NULL)
               continue;

            bodyRef->body->getCollisionShape()->setMargin(margin);

            // query the transform of the rbd!
            btTransform rbdTransform = bodyRef->GetWorldTransform();
            btVector3 rbdPos = rbdTransform.getOrigin();
            btQuaternion rbdRot = rbdTransform.getRotation();
            t.Set(rbdPos.getX(),rbdPos.getY(),rbdPos.getZ());
            q.Set(rbdRot.getW(),rbdRot.getX(),rbdRot.getY(),rbdRot.getZ());
            r.SetFromQuaternion(q);
            m = r.GetMatrix();
            offset.Set(p->offset[i].getX(),p->offset[i].getY(),p->offset[i].getZ());

            // now loop over all points of this island!
            for(int j=0;j<p->nbPerIsland[i];j++)
            {
               int index = p->islands[i][j];
               pos[index].SubInPlace(offset);
               pos[index].MulByMatrix3InPlace(m);
               pos[index].AddInPlace(t);
            }
         }
      }
   }

   Primitive outPrim(ctxt.GetOutputTarget());
   PolygonMesh outMesh = outPrim.GetGeometry();
   CVertexRefArray outPoints = outMesh.GetVertices();
   outPoints.PutPositionArray(pos);

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumDeform_Init( CRef& in_ctxt )
{
   gInstanceCount++;
   Context ctxt( in_ctxt );
   CustomOperator op(ctxt.GetSource());

   // inject the dbid of the operator into
   // the userdata of the operator
   mesh_UD * p = new mesh_UD;
   p->rbd.primary = (int)op.GetObjectID();
   p->rbd.secondary = -1;
   p->shape.primary = (int)op.GetObjectID();
   p->shape.secondary = -1;
   p->Clear();

   CValue val = (CValue::siPtrType) p;
   ctxt.PutUserData( val ) ;

   // if this is the first time we run it, let's create the world
   if(gSimulation == NULL)
   {
      gSimulation = new bulletSimulation();
      Application().LogMessage(L"[MOMENTUM] Created a new simulation world.",siVerboseMsg);
   }

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumDeform_Term( CRef& in_ctxt )
{
   gInstanceCount--;
   Context ctxt( in_ctxt );

   // get the dbid from the userdata
   CValue udVal = ctxt.GetUserData();
   mesh_UD * p = (mesh_UD*)(CValue::siPtrType)udVal;
   if(gSimulation->DeleteAllCollisionShapes(p->shape))
      Application().LogMessage(L"[MOMENTUM] Destroyed all collision shape for dbID "+CValue((LONG)p->shape.primary).GetAsText(),siVerboseMsg);
   if(gSimulation->DeleteAllRigidBodies(p->rbd))
      Application().LogMessage(L"[MOMENTUM] Destroyed all rigidbodies for dbID "+CValue((LONG)p->rbd.primary).GetAsText(),siVerboseMsg);
   p->Clear(); // free memory
   delete p;

   // if this is the terminate of the last operator, clear the world
   if(gInstanceCount == 0)
   {
      delete(gSimulation);
      gSimulation = NULL;
      Application().LogMessage(L"[MOMENTUM] Destroyed the simulation world.",siVerboseMsg);
   }

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_MomentumDeform_CreateControls_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create control particles for a deform rigidbody");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus apply_MomentumDeform_CreateControls_Execute( CRef& in_ctxt )
{

   Context ctxt( in_ctxt );
   Selection l_pSelection = Application().GetSelection();

   X3DObject l_pSelObj;
   CRef opRef;

   // search the selection for any compatible 3d object
   for(long i=0;i<l_pSelection.GetCount();i++)
   {
      l_pSelObj = X3DObject(l_pSelection.GetItem(i));
      if(l_pSelObj.GetType().IsEqualNoCase(L"polymsh"))
      {
         // find the operator
         opRef.Set(l_pSelObj.GetFullName()+L".polymsh.MomentumDeform");
         if(opRef.IsValid())
            break;
      }
   }

   // check if we found a valid mesh
   if(!opRef.IsValid())
   {
      Application().LogMessage(L"Please select a deform rigid body mesh!",siErrorMsg);
      return CStatus::OK;
   }

   // prepare some variables
   CValueArray cmdArgs;
   CValue returnVal;

   // great, now create the particles
   cmdArgs.Resize(1);
   cmdArgs[0] = L"PointCloud";
   Application().ExecuteCommand(L"SIGetPrim",cmdArgs,returnVal);
   CRef outputCloud = (CRef)((CValueArray&)returnVal)[0];

   // name the output mesh according to the source object
   X3DObject(Primitive(outputCloud).GetParent()).PutName(l_pSelObj.GetName()+L"_Controls");

   // apply the ice op for create the controls!
   cmdArgs.Resize(4);
   cmdArgs[0] = L"MOM Emit Deform Control";
   cmdArgs[1] = Primitive(outputCloud).GetParent().GetAsText();
   Application().ExecuteCommand(L"ApplyICEOp",cmdArgs,returnVal);
   CRef iceTree = (CRef)returnVal;

   // add the get data node!
   cmdArgs.Resize(2);
   cmdArgs[0] = L"GetDataNode";
   cmdArgs[1] = iceTree.GetAsText();
   Application().ExecuteCommand(L"AddICENode",cmdArgs,returnVal);

   // connect the get data node
   cmdArgs.Resize(2);
   cmdArgs[0] = iceTree.GetAsText()+L".MOM_Emit_Deform_Control.Fracture_Mesh";
   cmdArgs[1] = iceTree.GetAsText()+L".SceneReferenceNode.outname";
   Application().ExecuteCommand(L"ConnectICENodes",cmdArgs,returnVal);

   // set the reference on the get data node
   cmdArgs.Resize(2);
   cmdArgs[0] = iceTree.GetAsText()+L".SceneReferenceNode.reference";
   cmdArgs[1] = l_pSelObj.GetFullName();
   Application().ExecuteCommand(L"SetValue",cmdArgs,returnVal);

   CustomOperator op(opRef);

   // now let's create the scripted operator!
   Factory factory = Application().GetFactory();
   CustomOperator scriptedOp = factory.CreateObject(L"MomentumCopyID");
   opRef.Set(iceTree.GetAsText()+L".MOM_Emit_Deform_Control.PassThroughNode");
   scriptedOp.AddOutputPort(opRef,L"OutPort");
   scriptedOp.AddInputPort(op.GetRef(),L"in_Operator");
   scriptedOp.Connect().GetDescription();

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumCopyID_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   CustomOperator oCustomOperator;
   Parameter oParam;
   CRef oPDef;
   Factory oFactory = Application().GetFactory();
   oCustomOperator = ctxt.GetSource();
   oCustomOperator.PutAlwaysEvaluate(false);
   oCustomOperator.PutDebug(0);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumCopyID_DefineLayout( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   PPGLayout oLayout;
   PPGItem oItem;
   oLayout = ctxt.GetSource();
   oLayout.Clear();
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumCopyID_Update( CRef& in_ctxt )
{
   OperatorContext ctxt( in_ctxt );
   ProjectItem inItem(ctxt.GetInputValue(0));
   ICENode passThrough(ctxt.GetOutputTarget());
   passThrough.PutParameterValue(L"in",(LONG)inItem.GetObjectID());

   return CStatus::OK;
}

