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
#include "bulletSimulation.h"
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

using namespace XSI;
using namespace XSI::MATH;


XSIPLUGINCALLBACK CStatus MomentumSurface_Define( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );

   CustomOperator oCustomOperator;

   Parameter oParam;

   CRef oPDef;

   Factory oFactory = Application().GetFactory();

   oCustomOperator = ctxt.GetSource();


   oPDef = oFactory.CreateParamDef(L"frame",CValue::siInt4,siAnimatable | siPersistable,L"frame",L"frame",1,-100000,100000,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"margin",CValue::siFloat,siAnimatable | siPersistable,L"margin",L"margin",-.025,-100,100,-.1,.1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"friction",CValue::siFloat,siPersistable,L"friction",L"friction",.5,0,1000,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"bounce",CValue::siFloat,siPersistable,L"bounce",L"bounce",.1,0,1000,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);



   oCustomOperator.PutAlwaysEvaluate(false);

   oCustomOperator.PutDebug(0);

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumSurface_DefineLayout( CRef& in_ctxt )

{
   Context ctxt( in_ctxt );

   PPGLayout oLayout;

   PPGItem oItem;

   oLayout = ctxt.GetSource();

   oLayout.Clear();
   oLayout.AddItem(L"margin",L"Margin");
   oLayout.AddItem(L"friction",L"Friction");
   oLayout.AddItem(L"bounce",L"Bounce");
   return CStatus::OK;

}


XSIPLUGINCALLBACK CStatus MomentumSurface_Update( CRef& in_ctxt )
{
   // check if we have a simulation
   if(gSimulation == NULL)
      return CStatus::OK;

   OperatorContext ctxt( in_ctxt );

   // get the frame
   int frame = (LONG)ctxt.GetParameterValue(L"frame");
   frame--;

   // get the geometry
   Primitive prim = (CRef)ctxt.GetInputValue(0);
   PolygonMesh mesh = prim.GetGeometry();
   CVertexRefArray points = mesh.GetVertices();
   CVector3Array pos = points.GetPositionArray();
   CEdgeRefArray edges = mesh.GetEdges();
   CTriangleRefArray triangles = mesh.GetTriangles();

   // get the transform
   KinematicState kine = (CRef)ctxt.GetInputValue(1);
   CTransformation xf = kine.GetTransform();

   // get the dbID and the rigid body for it
   CValue udVal = ctxt.GetUserData();
   mesh_UD * p = (mesh_UD*)(CValue::siPtrType)udVal;
   if(p==NULL)
      return CStatus::Unexpected;

   // ok, we have the user data
   // let's ensure that we need to build it
   if(p->nbPoints != (int)points.GetCount() || p->nbEdges != (int)edges.GetCount() || p->nbTriangles != (int)triangles.GetCount())
   {
      // now let's remove the collision shape and the rigidbody
      gSimulation->DeleteCollisionShape(p->shape);
      gSimulation->DeleteRigidBody(p->rbd);

      // create a  new shape reference
      btCollisionShapeReference * shapeRef = new btCollisionShapeReference();
      shapeRef->op = ctxt.GetSource();
      shapeRef->prim = prim.GetRef();

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

      shapeRef->shape = new btBvhTriangleMeshShape(shapeRef->tris,true);

      // update the local scaling
      shapeRef->shape->setLocalScaling(btVector3(xf.GetSclX(),xf.GetSclY(),xf.GetSclZ()));

      // let's insert the shape into the simulation
      gSimulation->AddCollisionShape(p->shape,shapeRef);

      Application().LogMessage(L"[MOMENTUM] Created the collisionshape for dbID "+CValue((LONG)p->shape.primary).GetAsText(),siVerboseMsg);
   }

   btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(p->rbd);
   if(bodyRef == NULL)
   {
      // if we have a shape, let's create the rigid body
      btCollisionShapeReference * shapeRef = gSimulation->GetCollisionShape(p->shape);
      if(shapeRef != NULL)
      {
         float margin = ctxt.GetParameterValue(L"margin");
         float friction = ctxt.GetParameterValue(L"friction");
         float bounce = ctxt.GetParameterValue(L"bounce");

         bodyRef = new btRigidBodyReference();

         // setup the transform on the object!
         btTransform transform;
         transform.setIdentity();
         transform.setOrigin(btVector3(xf.GetPosX(),xf.GetPosY(),xf.GetPosZ()));
         CQuaternion q = xf.GetRotationQuaternion();
         transform.setRotation(btQuaternion(q.GetX(),q.GetY(),q.GetZ(),q.GetW()));

         // create the default motion state
         btDefaultMotionState* motionState = new btDefaultMotionState(transform);

         // compute the inertia of the obkcet
         float mass = ctxt.GetParameterValue(L"mass");
         btVector3 inertia(0,0,0);
         if(mass > 0.0f)
            shapeRef->shape->calculateLocalInertia(mass,inertia);

         // create the rbd construction info
         btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,motionState,shapeRef->shape,inertia);

         // set some more things
         rbInfo.m_friction = friction;
         rbInfo.m_restitution = bounce;

         bodyRef = new btRigidBodyReference();
         bodyRef->op = ctxt.GetSource();
         bodyRef->kine = kine.GetRef();

         bodyRef->body = new btRigidBody(rbInfo);

         // now set the initial state
         int state = (int)(LONG)ctxt.GetParameterValue(L"state");
         bodyRef->body->setActivationState(state);

         //add the body to the dynamics world
         gSimulation->AddRigidBody(p->rbd,bodyRef);

         Application().LogMessage(L"[MOMENTUM] Created the rigidbody for dbID "+CValue((LONG)p->rbd.primary).GetAsText(),siVerboseMsg);
      }
   }

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumSurface_Init( CRef& in_ctxt )
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

XSIPLUGINCALLBACK CStatus MomentumSurface_Term( CRef& in_ctxt )
{
   //return MomentumDeform_Term(in_ctxt);
   gInstanceCount--;
   Context ctxt( in_ctxt );

   // get the dbid from the userdata
   CValue udVal = ctxt.GetUserData();
   mesh_UD * p = (mesh_UD*)(CValue::siPtrType)udVal;
   if(gSimulation->DeleteCollisionShape(p->shape))
      Application().LogMessage(L"[MOMENTUM] Destroyed the collision shape for dbID "+CValue((LONG)p->shape.primary).GetAsText(),siVerboseMsg);
   if(gSimulation->DeleteRigidBody(p->rbd))
      Application().LogMessage(L"[MOMENTUM] Destroyed the rigidbodies for dbID "+CValue((LONG)p->rbd.primary).GetAsText(),siVerboseMsg);
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


XSIPLUGINCALLBACK CStatus apply_MomentumSurface_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of MomentumSurface operator");
   oCmd.SetFlag(siNoLogging,false);

   return CStatus::OK;

}



XSIPLUGINCALLBACK CStatus apply_MomentumSurface_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Selection l_pSelection = Application().GetSelection();

   // search the selection for any compatible 3d object
   for(long i=0;i<l_pSelection.GetCount();i++)
   {
      X3DObject l_pSelObj(l_pSelection.GetItem(i));
      if(l_pSelObj.GetType().IsEqualNoCase(L"polymsh"))
      {
         // create the operator
         CustomOperator newOp = Application().GetFactory().CreateObject(L"MomentumSurface");
         newOp.AddOutputPort(l_pSelObj.GetActivePrimitive().GetRef());
         newOp.AddInputPort(l_pSelObj.GetActivePrimitive().GetRef());
         newOp.AddInputPort(l_pSelObj.GetKinematics().GetGlobal().GetRef());
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
         Application().LogMessage(L"[MOMENTUM] Object "+l_pSelObj.GetFullName()+L" is not valid for surfacing.",siWarningMsg);
      }

   }

   return CStatus::OK;

}
