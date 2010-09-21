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
#include <xsi_factory.h>
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
#include <vector>

#include "Momentum.h"
#include "Plotting.h"
#include "bulletSimulation.h"
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

using namespace XSI;
using namespace XSI::MATH;

XSIPLUGINCALLBACK CStatus apply_MomentumKinematics_Init( CRef& in_ctxt )
{

   Context ctxt( in_ctxt );

   Command oCmd;

   oCmd = ctxt.GetSource();

   oCmd.PutDescription(L"Create an instance of MomentumKinematics operator");

   oCmd.SetFlag(siNoLogging,false);

   return CStatus::OK;
}



XSIPLUGINCALLBACK CStatus apply_MomentumKinematics_Execute( CRef& in_ctxt )

{

   Context ctxt( in_ctxt );

   Selection l_pSelection = Application().GetSelection();



   // search the selection for any compatible 3d object

   for(long i=0;i<l_pSelection.GetCount();i++)

   {

      X3DObject l_pSelObj(l_pSelection.GetItem(i));

      if(l_pSelObj.GetType().IsEqualNoCase(L"polymsh") ||
         l_pSelObj.GetType().IsEqualNoCase(L"cube") ||

         l_pSelObj.GetType().IsEqualNoCase(L"cylinder") ||

         l_pSelObj.GetType().IsEqualNoCase(L"grid") ||

         l_pSelObj.GetType().IsEqualNoCase(L"sphere"))

      {

         // set keys on the transform of the object
         // this way we can edit the transform lateron (with fcurves)
         CValue returnVal;
         CValueArray args(1);
         args[0] = l_pSelObj.GetKinematics().GetLocal().GetFullName()+L".posx,"+
                   l_pSelObj.GetKinematics().GetLocal().GetFullName()+L".posy,"+
                   l_pSelObj.GetKinematics().GetLocal().GetFullName()+L".posz,"+
                   l_pSelObj.GetKinematics().GetLocal().GetFullName()+L".rotx,"+
                   l_pSelObj.GetKinematics().GetLocal().GetFullName()+L".roty,"+
                   l_pSelObj.GetKinematics().GetLocal().GetFullName()+L".rotz";
         Application().ExecuteCommand(L"Savekey",args,returnVal);

         // add the plotting data
         UserDataBlob plotBlob;
         l_pSelObj.AddProperty( L"UserDataBlob", false, L"plottingData", plotBlob) ;

         // create the operator
         CustomOperator newOp = Application().GetFactory().CreateObject(L"MomentumKinematics");
         newOp.AddOutputPort(l_pSelObj.GetKinematics().GetGlobal().GetRef());
         newOp.AddInputPort(l_pSelObj.GetKinematics().GetGlobal().GetRef());
         newOp.AddInputPort(l_pSelObj.GetActivePrimitive().GetRef());
         newOp.AddInputPort(plotBlob.GetRef());
         newOp.Connect();

         // if we are a plane, become passive by default!
         if(l_pSelObj.GetType().IsEqualNoCase(L"grid"))
            newOp.PutParameterValue(L"mass",(float)0);

         // set the expression for the frame!
         args.Resize(2);
         args[0] = newOp.GetFullName()+L".frame";
         args[1] = L"fc";
         Application().ExecuteCommand(L"SetExpr",args,returnVal);
      }
      else
      {
         Application().LogMessage(L"[MOMENTUM] Object "+l_pSelObj.GetFullName()+L" is not valid for dynamics.",siWarningMsg);
      }

   }

   return CStatus::OK;

}


XSIPLUGINCALLBACK CStatus MomentumKinematics_Init( CRef& in_ctxt )
{

   gInstanceCount++;
   Context ctxt( in_ctxt );
   CustomOperator op(ctxt.GetSource());

   // inject the dbid of the operator into
   // the userdata of the operator
   kine_UD * p = new kine_UD;
   p->rbd.primary = (int)op.GetObjectID();
   p->rbd.secondary = -1;
   p->shape.primary = (int)op.GetObjectID();
   p->shape.secondary = -1;
   CValue val = (CValue::siPtrType) p;
   ctxt.PutUserData( val ) ;

   // if this is the first time we run it, let's create the world
   if(gSimulation == NULL)
   {
      gSimulation = new bulletSimulation();
      Application().LogMessage(L"[MOMENTUM] Created a new bullet simulation world.",siVerboseMsg);
   }

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumKinematics_Term( CRef& in_ctxt )
{

   gInstanceCount--;
   Context ctxt( in_ctxt );

   // get the dbid from the userdata
   CValue udVal = ctxt.GetUserData();
   kine_UD * p = (kine_UD*)(CValue::siPtrType)udVal;
   if(gSimulation->DeleteCollisionShape(p->shape))
      Application().LogMessage(L"[MOMENTUM] Destroyed the collision shape for dbID "+CValue((LONG)p->shape.primary).GetAsText(),siVerboseMsg);
   if(gSimulation->DeleteRigidBody(p->rbd))
      Application().LogMessage(L"[MOMENTUM] Destroyed the rigidbody for dbID "+CValue((LONG)p->rbd.primary).GetAsText(),siVerboseMsg);
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


XSIPLUGINCALLBACK CStatus MomentumKinematics_Define( CRef& in_ctxt )

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

   oPDef = oFactory.CreateParamDef(L"recreate",CValue::siBool,siAnimatable | siPersistable,L"recreate",L"recreate",1,0,1,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"mass",CValue::siFloat,siAnimatable | siPersistable,L"mass",L"mass",1,0,10000000,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"margin",CValue::siFloat,siAnimatable | siPersistable,L"margin",L"margin",0,0,100,0,.1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"friction",CValue::siFloat,siPersistable,L"friction",L"friction",0.9,0,1000,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"bounce",CValue::siFloat,siPersistable,L"bounce",L"bounce",0,0,1000,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"plotted",CValue::siBool,siPersistable,L"plotted",L"plotted",0,0,1,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linThreshold",CValue::siFloat,siAnimatable | siPersistable,L"linThreshold",L"linThreshold",0.8f,0.0f,1000.0f,0.0f,0.01f);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angThreshold",CValue::siFloat,siAnimatable | siPersistable,L"angThreshold",L"angThreshold",1.0f,0.0f,1000.0f,0.0f,0.01f);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linDamping",CValue::siFloat,siAnimatable | siPersistable,L"linDamping",L"linDamping",0.3f,0.0f,1000.0f,0.0f,0.01f);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angDamping",CValue::siFloat,siAnimatable | siPersistable,L"angDamping",L"angDamping",0.3f,0.0f,1000.0f,0.0f,0.01f);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linVelX",CValue::siFloat,siAnimatable | siPersistable,L"linVelX",L"linVelX",0.0f,-100000.0f,100000.0f,-10.0f,10.00f);
   oCustomOperator.AddParameter(oPDef,oParam);
   oPDef = oFactory.CreateParamDef(L"linVelY",CValue::siFloat,siAnimatable | siPersistable,L"linVelY",L"linVelY",0.0f,-100000.0f,100000.0f,-10.0f,10.00f);
   oCustomOperator.AddParameter(oPDef,oParam);
   oPDef = oFactory.CreateParamDef(L"linVelZ",CValue::siFloat,siAnimatable | siPersistable,L"linVelZ",L"linVelZ",0.0f,-100000.0f,100000.0f,-10.0f,10.00f);
   oCustomOperator.AddParameter(oPDef,oParam);
   oPDef = oFactory.CreateParamDef(L"angVelX",CValue::siFloat,siAnimatable | siPersistable,L"angVelX",L"angVelX",0.0f,-100000.0f,100000.0f,-10.0f,10.00f);
   oCustomOperator.AddParameter(oPDef,oParam);
   oPDef = oFactory.CreateParamDef(L"angVelY",CValue::siFloat,siAnimatable | siPersistable,L"angVelY",L"angVelY",0.0f,-100000.0f,100000.0f,-10.0f,10.00f);
   oCustomOperator.AddParameter(oPDef,oParam);
   oPDef = oFactory.CreateParamDef(L"angVelZ",CValue::siFloat,siAnimatable | siPersistable,L"angVelZ",L"angVelZ",0.0f,-100000.0f,100000.0f,-10.0f,10.00f);
   oCustomOperator.AddParameter(oPDef,oParam);



   oCustomOperator.PutAlwaysEvaluate(false);

   oCustomOperator.PutDebug(0);

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumKinematics_DefineLayout( CRef& in_ctxt )

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
   oLayout.AddItem(L"mass",L"Mass");
   oLayout.AddItem(L"margin",L"Margin");
   oLayout.AddItem(L"friction",L"Friction");
   oLayout.AddItem(L"bounce",L"Bounce");
   oLayout.AddItem(L"linDamping",L"Linear Damping");
   oLayout.AddItem(L"angDamping",L"Angular Damping");
   oLayout.AddItem(L"plotted",L"Plotted");

   oLayout.AddGroup(L"Sleeping Mode Controls");
   oLayout.AddItem(L"linThreshold",L"Linear Threshold");
   oLayout.AddItem(L"angThreshold",L"Angular Threshold");
   oLayout.EndGroup();


   oLayout.AddGroup(L"Mesh Controls");
   CValueArray modeItems(6);
   modeItems[0] = L"Box Shape";
   modeItems[1] = (LONG)0l;
   modeItems[2] = L"Convex Hull Shape";
   modeItems[3] = (LONG)4l;
   modeItems[4] = L"GImpact Actual Shape";
   modeItems[5] = (LONG)25l;
   oLayout.AddEnumControl(L"mode",modeItems,L"Mode");
   oLayout.AddItem(L"recreate",L"Recreate Shape");
   oLayout.EndGroup();
   oLayout.AddGroup(L"Initial Linear Velocity");
   oLayout.AddRow();
   oLayout.AddItem(L"linVelX",L"x");
   oLayout.AddItem(L"linVelY",L"y");
   oLayout.AddItem(L"linVelZ",L"z");
   oLayout.EndRow();
   oLayout.EndGroup();
   oLayout.AddGroup(L"Initial Angular Velocity");
   oLayout.AddRow();
   oLayout.AddItem(L"angVelX",L"x");
   oLayout.AddItem(L"angVelY",L"y");
   oLayout.AddItem(L"angVelZ",L"z");
   oLayout.EndRow();
   oLayout.EndGroup();
   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumKinematics_Update( CRef& in_ctxt )
{
   // check if we have a simulation
   if(gSimulation == NULL)
      return CStatus::OK;

   OperatorContext ctxt( in_ctxt );

   // access the input kinematic state
   KinematicState kine = (CRef)ctxt.GetInputValue(0);
   CTransformation xf = kine.GetTransform();
   Primitive prim = (CRef)ctxt.GetInputValue(1);
   CString lType = prim.GetType();

   // get the frame
   int mode = (LONG)ctxt.GetParameterValue(L"mode");
   int state = (int)(LONG)ctxt.GetParameterValue(L"state");
   int frame = (LONG)ctxt.GetParameterValue(L"frame");
   frame--;

   // if we are plotted, let's not do anything please
   if((bool)ctxt.GetParameterValue(L"plotted"))
   {
      // let's retrieve the plotted data
      UserDataBlob udb(ctxt.GetInputValue(2));
      PLOT_UD plotData;
      udb.GetValue(plotData.buffer,plotData.size);
      if(plotData.size > 0)
      {
         // clamp frame
         if(frame < plotData.GetFrameIn())
            frame = plotData.GetFrameIn();
         else if(frame > plotData.GetFrameOut())
            frame = plotData.GetFrameOut();
         btVector3 rbdPos = plotData.GetPosition(frame,0);
         btQuaternion rbdRot = plotData.GetRotation(frame,0);

         // set them on the output
         xf.SetTranslationFromValues(rbdPos.getX(),rbdPos.getY(),rbdPos.getZ());
         xf.SetRotationFromQuaternion(CQuaternion(rbdRot.getW(),rbdRot.getX(),rbdRot.getY(),rbdRot.getZ()));

         KinematicState outKine = (CRef)ctxt.GetOutputTarget();
         outKine.PutTransform(xf);
      }
      return CStatus::OK;
   }

   // get the dbID and the rigid body for it
   CValue udVal = ctxt.GetUserData();
   kine_UD * p = (kine_UD*)(CValue::siPtrType)udVal;
   if(p==NULL)
      return CStatus::Unexpected;


   btCollisionShapeReference * shapeRef = gSimulation->GetCollisionShape(p->shape);
   // if we have one, check if the mode is correct!
   if(shapeRef != NULL)
   {
      if(shapeRef->shape != NULL && lType.IsEqualNoCase(L"polymsh"))
      {
         bool recreate = (bool)ctxt.GetParameterValue(L"recreate");
         if(shapeRef->shape->getShapeType() != mode || (recreate && frame <= 0))
         {
            gSimulation->DeleteCollisionShape(p->shape);
            gSimulation->DeleteRigidBody(p->rbd);
            shapeRef = NULL;
         }
      }
   }

   // if we don't have a shape, create one!
   btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(p->rbd);
   if(bodyRef == NULL)
   {
      // first check if we need to create the collision shape
      if(shapeRef == NULL)
      {
         CString lType = prim.GetType();

         shapeRef = new btCollisionShapeReference();
         shapeRef->op = ctxt.GetSource();
         shapeRef->prim = prim.GetRef();

         // we have to create the collision shape according
         // to the type of primitive
         if(lType.IsEqualNoCase(L"cube"))
         {
            // create a cube rigid body!
            // use the proper scaling of the box!
            float length = prim.GetParameterValue(L"length");
            shapeRef->shape = new btBoxShape(btVector3(length * .5f,length * .5f,length * .5f));
         }
         else if(lType.IsEqualNoCase(L"grid"))
         {
            // create a plane rigid body!
            shapeRef->shape = new btStaticPlaneShape(btVector3(0,1,0),0);
         }
         else if(lType.IsEqualNoCase(L"sphere"))
         {
            // create a sphere rigid body!
            float radius = prim.GetParameterValue(L"radius");
            shapeRef->shape = new btSphereShape(radius);
         }
         else if(lType.IsEqualNoCase(L"cylinder"))
         {
            // create a cylinder rigid body!
            float radius = prim.GetParameterValue(L"radius");
            float height = prim.GetParameterValue(L"height");
            shapeRef->shape = new btCylinderShape(btVector3(radius,radius,height * 0.5));
         }
         else if(lType.IsEqualNoCase(L"polymsh"))
         {
            PolygonMesh mesh = prim.GetGeometry();

            // get the mode that we need
            if(mode == 0) // btBoxShape
            {
               CVector3Array pos = mesh.GetPoints().GetPositionArray();

               btVector3 bbMin(100000,100000,100000);
               btVector3 bbMax(-100000,-100000,-100000);

               for(LONG i=0;i<pos.GetCount();i++)
               {
                  // merge the bbox
                  CVector3 p1(pos[i]);
                  if(p1.GetX() < bbMin.getX())bbMin.setX(p1.GetX());
                  if(p1.GetY() < bbMin.getY())bbMin.setY(p1.GetY());
                  if(p1.GetZ() < bbMin.getZ())bbMin.setZ(p1.GetZ());
                  if(p1.GetX() > bbMax.getX())bbMax.setX(p1.GetX());
                  if(p1.GetY() > bbMax.getY())bbMax.setY(p1.GetY());
                  if(p1.GetZ() > bbMax.getZ())bbMax.setZ(p1.GetZ());
               }

               btVector3 scale = (bbMax - bbMin) * .5;
               btVector3 center = (bbMin + bbMax) * .5;
               btTransform offset;
               offset.setIdentity();
               offset.setOrigin(center);

               btBoxShape * boxShape = new btBoxShape(scale);
               btCompoundShape * compShape = new btCompoundShape();
               compShape->addChildShape(offset,boxShape);

               shapeRef->shape = compShape;
            }
            else if(mode == 4) // btConvexHullShape
            {
               float margin = (float)ctxt.GetParameterValue(L"margin");

               CVector3Array pos = mesh.GetPoints().GetPositionArray();

               btConvexHullShape * complexShape = new btConvexHullShape();
               for(LONG i=0;i<pos.GetCount();i++)
                  complexShape->addPoint(btVector3(pos[i].GetX(),pos[i].GetY(),pos[i].GetZ()));

               //This optmization fails for low poly objects, so we'll remove it for now.
               // now create the convex hull shape
               /*
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
               gSimulation->AddCollisionShape(p->shape,shapeRef);
            }
            else if(mode == 25) // btGImpactShape
            {
               CGeometryAccessor acc = mesh.GetGeometryAccessor(siConstructionModeSecondaryShape);

               // get the mesh data
               CLongArray indices;
               acc.GetTriangleVertexIndices(indices);
               CDoubleArray positions;
               acc.GetVertexPositions(positions);

               shapeRef->triId = (int*)malloc(sizeof(int)*indices.GetCount());
               shapeRef->triPos = (btScalar*)malloc(sizeof(btScalar)*positions.GetCount());

                // copy the data
               for(int i=0;i<indices.GetCount();i++)
                  shapeRef->triId[i] = indices[i];
               for(int i = 0; i<positions.GetCount();i++)
                  shapeRef->triPos[i] = positions[i];

               shapeRef->tris = new btTriangleIndexVertexArray(
                  indices.GetCount() / 3,shapeRef->triId,3*sizeof(int),
                  positions.GetCount() / 3, shapeRef->triPos,3*sizeof(btScalar));

               btGImpactMeshShape * trimesh = new btGImpactMeshShape(shapeRef->tris);
               trimesh->updateBound();

               shapeRef->shape = trimesh;

               // update the local scaling
               shapeRef->shape->setLocalScaling(btVector3(xf.GetSclX(),xf.GetSclY(),xf.GetSclZ()));

               // let's insert the shape into the simulation
               gSimulation->AddCollisionShape(p->shape,shapeRef);
            }
            Application().LogMessage(L"[MOMENTUM] Created the collisionshape for dbID "+CValue((LONG)p->shape.primary).GetAsText(),siVerboseMsg);
         }
      }

      // if we have a shape, let's create the rigid body
      if(shapeRef != NULL)
      {
         if(shapeRef->shape == NULL)
            Application().LogMessage(L"oh brother, shape is NULL");

         // setup the transform on the object!
         btTransform transform;
         transform.setIdentity();
         transform.setOrigin(btVector3(xf.GetPosX(),xf.GetPosY(),xf.GetPosZ()));
         CQuaternion q = xf.GetRotationQuaternion();
         transform.setRotation(btQuaternion(q.GetX(),q.GetY(),q.GetZ(),q.GetW()));

         // create the default motion state
         btDefaultMotionState* motionState = new btDefaultMotionState(transform);

         bodyRef = new btRigidBodyReference();

         // compute the inertia of the obkcet
         bodyRef->mass = ctxt.GetParameterValue(L"mass");
         btVector3 inertia(0,0,0);
         if(bodyRef->mass > 0.0f)
            shapeRef->shape->calculateLocalInertia(bodyRef->mass,inertia);

         // create the rbd construction info
         btRigidBody::btRigidBodyConstructionInfo rbInfo(bodyRef->mass,motionState,shapeRef->shape,inertia);

         // set some more things
         float friction = ctxt.GetParameterValue(L"friction");
         float bounce = ctxt.GetParameterValue(L"bounce");
         rbInfo.m_friction = friction;
         rbInfo.m_restitution = bounce;

         bodyRef->op = ctxt.GetSource();
         bodyRef->kine = kine.GetRef();

         bodyRef->body = new btRigidBody(rbInfo);

         // now set the initial state
         bodyRef->body->setActivationState(state);

         //Set the sleeping thresholds
         float linTh = ctxt.GetParameterValue(L"linThreshold");
         float angTh = ctxt.GetParameterValue(L"angThreshold");
         bodyRef->body->setSleepingThresholds(linTh,angTh);

         //Set damping params
         float linDamping = ctxt.GetParameterValue(L"linDamping");
         float angDamping = ctxt.GetParameterValue(L"angDamping");
         bodyRef->body->setDamping(linDamping,angDamping);

         // setup the linear velocity
         btVector3 vel;
         vel.setX(ctxt.GetParameterValue(L"linVelX"));
         vel.setY(ctxt.GetParameterValue(L"linVelY"));
         vel.setZ(ctxt.GetParameterValue(L"linVelZ"));
         bodyRef->body->setLinearVelocity(vel);

         // setup the angular velocity
         vel.setX(ctxt.GetParameterValue(L"angVelX"));
         vel.setY(ctxt.GetParameterValue(L"angVelY"));
         vel.setZ(ctxt.GetParameterValue(L"angVelZ"));
         bodyRef->body->setAngularVelocity(vel);

         //add the body to the dynamics world
         gSimulation->AddRigidBody(p->rbd,bodyRef);

         Application().LogMessage(L"[MOMENTUM] Created the rigidbody for dbID "+CValue((LONG)p->rbd.primary).GetAsText(),siVerboseMsg);
      }
   }

   // if we finally have a body
   bool wasActive = false;
   if(bodyRef != NULL)
   {
      // determine if this is an active rbd or not
      wasActive = bodyRef->body->getInvMass() != 0.0f;

      // update a couple of things!
      if(bodyRef->body->getCollisionShape() != NULL)
      {
         float margin = ctxt.GetParameterValue(L"margin");
         bodyRef->body->getCollisionShape()->setMargin(margin);

         bodyRef->mass = ctxt.GetParameterValue(L"mass");
         btVector3 inertia(0,0,0);
         if(bodyRef->mass > 0.0f)
            bodyRef->body->getCollisionShape()->calculateLocalInertia(bodyRef->mass,inertia);
         bodyRef->body->setMassProps(bodyRef->mass,inertia);

         float linTh = ctxt.GetParameterValue(L"linThreshold");
         float angTh = ctxt.GetParameterValue(L"angThreshold");
         bodyRef->body->setSleepingThresholds(linTh,angTh);

         float linDamping = ctxt.GetParameterValue(L"linDamping");
         float angDamping = ctxt.GetParameterValue(L"angDamping");
         bodyRef->body->setDamping(linDamping,angDamping);

         if(wasActive && bodyRef->mass == 0.0)
         {
            // deactivate
            if(!bodyRef->body->isStaticObject())
               bodyRef->body->setCollisionFlags( bodyRef->body->getCollisionFlags() & btCollisionObject::CF_KINEMATIC_OBJECT);
            wasActive = false;
         }
         else if(!wasActive && bodyRef->mass != 0.0f)
         {
            if(bodyRef->body->isStaticObject())
               bodyRef->body->setCollisionFlags( bodyRef->body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
            if(state == DISABLE_DEACTIVATION || state == ACTIVE_TAG)
               bodyRef->body->forceActivationState(ACTIVE_TAG);
            else
               bodyRef->body->setActivationState(state);

            // check if the rigid body has this...
            if(bodyRef->cluster == NULL)
            {
               // force a refresh
               gSimulation->GetDynamicsWorld()->removeRigidBody(bodyRef->body);
               gSimulation->GetDynamicsWorld()->addRigidBody(bodyRef->body);
            }

            // setup the linear velocity
            btVector3 vel;
            vel.setX(ctxt.GetParameterValue(L"linVelX"));
            vel.setY(ctxt.GetParameterValue(L"linVelY"));
            vel.setZ(ctxt.GetParameterValue(L"linVelZ"));
            bodyRef->body->setLinearVelocity(vel);

            // setup the angular velocity
            vel.setX(ctxt.GetParameterValue(L"angVelX"));
            vel.setY(ctxt.GetParameterValue(L"angVelY"));
            vel.setZ(ctxt.GetParameterValue(L"angVelZ"));
            bodyRef->body->setAngularVelocity(vel);
            //wasActive = true;
         }
      }
   }

   if(frame != 0 && wasActive && bodyRef != NULL)
   {
      // query the transform of the rbd!
      btTransform rbdTransform = bodyRef->GetWorldTransform();
      btVector3 rbdPos = rbdTransform.getOrigin();
      btQuaternion rbdRot = rbdTransform.getRotation();

      // set them on the output
      xf.SetTranslationFromValues(rbdPos.getX(),rbdPos.getY(),rbdPos.getZ());
      xf.SetRotationFromQuaternion(CQuaternion(rbdRot.getW(),rbdRot.getX(),rbdRot.getY(),rbdRot.getZ()));

      KinematicState outKine = (CRef)ctxt.GetOutputTarget();
      outKine.PutTransform(xf);
   }

   return CStatus::OK;
}



