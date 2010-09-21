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


XSIPLUGINCALLBACK CStatus apply_MomentumConsBallSocket_Init( CRef& in_ctxt )

{

   Context ctxt( in_ctxt );

   Command oCmd;

   oCmd = ctxt.GetSource();

   oCmd.PutDescription(L"Create an instance of MomentumConsBallSocket operator");

   oCmd.SetFlag(siNoLogging,false);

   return CStatus::OK;

}



XSIPLUGINCALLBACK CStatus apply_MomentumConsBallSocket_Execute( CRef& in_ctxt )

{

   Context ctxt( in_ctxt );

   Selection l_pSelection = Application().GetSelection();

   // you need to select TWO objects
   if(l_pSelection.GetCount() != 2)
   {
      Application().LogMessage(L"[MOMENTUM] You need to select TWO rigid bodies.",siErrorMsg);
      return CStatus::Abort;
   }

   X3DObject obj1(l_pSelection.GetItem(0));

   X3DObject obj2(l_pSelection.GetItem(1));
   CRef opRef1,opRef2;
   opRef1.Set(obj1.GetFullName()+L".kine.global.MomentumKinematics");
   if(!opRef1.IsValid())

      opRef1.Set(obj1.GetFullName()+L".polymsh.MomentumDeform");
   opRef2.Set(obj2.GetFullName()+L".kine.global.MomentumKinematics");
   if(!opRef2.IsValid())

      opRef2.Set(obj2.GetFullName()+L".polymsh.MomentumDeform");
   if(!opRef1.IsValid())
   {
      Application().LogMessage(L"[MOMENTUM] Object "+obj1.GetFullName()+L" is not a bullet rigid body.",siErrorMsg);
      return CStatus::Abort;

   }
   if(!opRef2.IsValid())
   {
      Application().LogMessage(L"[MOMENTUM] Object "+obj1.GetFullName()+L" is not a bullet rigid body.",siErrorMsg);
      return CStatus::Abort;

   }

   // get the transform for the cons
   CTransformation xfA = obj1.GetKinematics().GetGlobal().GetTransform();
   CTransformation xfB = obj2.GetKinematics().GetGlobal().GetTransform();
   CVector3 trans;
   trans.LinearlyInterpolate(xfA.GetTranslation(),xfB.GetTranslation(),0.5);
   xfA.SetTranslation(trans);

   // create the constraint object
   Null consNull1;
   obj1.AddNull(L"BallSocket_pivot",consNull1);
   consNull1.GetKinematics().GetGlobal().PutTransform(xfA);
   consNull1.GetActivePrimitive().PutParameterValue(L"primary_icon", (LONG)2);

   // create the operator

   CustomOperator newOp = Application().GetFactory().CreateObject(L"MomentumConsBallSocket");

   newOp.AddOutputPort(consNull1.GetKinematics().GetGlobal().GetRef());

   newOp.AddInputPort(consNull1.GetKinematics().GetGlobal().GetRef());

   newOp.AddInputPort(obj1.GetKinematics().GetGlobal().GetRef());

   newOp.AddInputPort(obj2.GetKinematics().GetGlobal().GetRef());

   newOp.AddInputPort(opRef1);

   newOp.AddInputPort(opRef2);

   newOp.Connect();

   // set the expression for the frame!
   CValueArray args(2);
   CValue returnVal;
   args[0] = newOp.GetFullName()+L".frame";
   args[1] = L"fc";
   Application().ExecuteCommand(L"SetExpr",args,returnVal);

   return CStatus::OK;

}




XSIPLUGINCALLBACK CStatus MomentumConsBallSocket_Define( CRef& in_ctxt )

{

   Context ctxt( in_ctxt );

   CustomOperator oCustomOperator;

   Parameter oParam;

   CRef oPDef;

   Factory oFactory = Application().GetFactory();

   oCustomOperator = ctxt.GetSource();


   oPDef = oFactory.CreateParamDef(L"frame",CValue::siInt4,siAnimatable | siPersistable,L"frame",L"frame",1,-100000,100000,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"secondaryA",CValue::siInt4,siPersistable,L"secondaryA",L"secondaryA",-1,-1,100000,-1,1000);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"secondaryB",CValue::siInt4,siPersistable,L"secondaryB",L"secondaryB",-1,-1,100000,-1,1000);

   oCustomOperator.AddParameter(oPDef,oParam);


   oCustomOperator.PutAlwaysEvaluate(false);

   oCustomOperator.PutDebug(0);

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumConsBallSocket_DefineLayout( CRef& in_ctxt )

{

   Context ctxt( in_ctxt );

   PPGLayout oLayout;

   PPGItem oItem;

   oLayout = ctxt.GetSource();

   oLayout.Clear();
   oLayout.AddGroup(L"Ports");
   oLayout.AddItem(L"secondaryA",L"Secondary ID A");
   oLayout.AddItem(L"secondaryB",L"Secondary ID B");
   oLayout.EndGroup();
   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumConsBallSocket_Update( CRef& in_ctxt )
{
   // check if we have a simulation
   if(gSimulation == NULL)
      return CStatus::OK;

   OperatorContext ctxt( in_ctxt );

   // get the dbID and the rigid body for it
   CValue udVal = ctxt.GetUserData();
   cons_UD * p = (cons_UD*)(CValue::siPtrType)udVal;
   if(p==NULL)
      return CStatus::Unexpected;

   btConstraintReference * consRef = gSimulation->GetConstraint(p->cons);
   if(consRef == NULL)
   {
      // we have to build the rigid body!
      KinematicState kineA = (CRef)ctxt.GetInputValue(0);
      CTransformation xfA = kineA.GetTransform();

      // just pull on the object transform to ensure eval of
      // the rbd ops
      ctxt.GetInputValue(1);
      ctxt.GetInputValue(2);

      // first we need to find the rigid bodies
      LONG primaryA = ProjectItem((CRef)ctxt.GetInputValue(3)).GetObjectID();
      LONG primaryB = ProjectItem((CRef)ctxt.GetInputValue(4)).GetObjectID();
      LONG secondaryA = ctxt.GetParameterValue(L"secondaryA");
      LONG secondaryB = ctxt.GetParameterValue(L"secondaryB");
      rbdID bodyIdA((int)primaryA,(int)secondaryA);
      rbdID bodyIdB((int)primaryB,(int)secondaryB);
      btRigidBodyReference * bodyRefA = gSimulation->GetRigidBody(bodyIdA);
      if(bodyRefA == NULL)
      {
         Application().LogMessage(L"[MOMENTUM] Rigid body "+CValue((LONG)bodyIdA.primary).GetAsText()+L","+CValue((LONG)bodyIdA.secondary).GetAsText()+L" not found.",siVerboseMsg);
         return CStatus::OK;
      }
      btRigidBodyReference * bodyRefB = gSimulation->GetRigidBody(bodyIdB);
      if(bodyRefB == NULL)
      {
         Application().LogMessage(L"[MOMENTUM] Rigid body "+CValue((LONG)bodyIdB.primary).GetAsText()+L","+CValue((LONG)bodyIdB.secondary).GetAsText()+L" not found.",siVerboseMsg);
         return CStatus::OK;
      }
      if(bodyRefA->body == NULL)
         return CStatus::OK;
      if(bodyRefB->body == NULL)
         return CStatus::OK;

      // get the transform from the bodies
      btTransform bodyTransformA = bodyRefA->GetWorldTransform();
      btTransform bodyTransformB = bodyRefB->GetWorldTransform();

      // map the pivot points
      btVector3 pivotPosGlobal(xfA.GetPosX(),xfA.GetPosY(),xfA.GetPosZ());

      // we need to compute both transforms
      btTransform pivotA;
      pivotA.setIdentity();
      pivotA.setOrigin(bodyTransformA.inverse() * pivotPosGlobal);
      btTransform pivotB;
      pivotB.setIdentity();
      pivotB.setOrigin(bodyTransformB.inverse() * pivotPosGlobal);

      // cool, we have both bodies
      // let's create the constraint!
      consRef = new btConstraintReference();
      consRef->bodyA = bodyRefA->body;
      consRef->bodyB = bodyRefB->body;
      consRef->bodyIdA = bodyIdA;
      consRef->bodyIdB = bodyIdB;
      consRef->pivotA = pivotA;
      consRef->pivotB = pivotB;

      btPoint2PointConstraint * BallSocket = new btPoint2PointConstraint(
         *consRef->bodyA,*consRef->bodyB,pivotA.getOrigin(),pivotB.getOrigin());

      consRef->constraint = BallSocket;

      // now create it
      gSimulation->AddConstraint(p->cons,consRef);

      Application().LogMessage(L"[MOMENTUM] Created the constraint for dbID "+CValue((LONG)p->cons.primary).GetAsText(),siVerboseMsg);
   }

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumConsBallSocket_Init( CRef& in_ctxt )
{
   gInstanceCount++;
   Context ctxt( in_ctxt );
   CustomOperator op(ctxt.GetSource());

   // inject the dbid of the operator into
   // the userdata of the operator
   cons_UD * p = new cons_UD;
   p->cons.primary = (int)op.GetObjectID();
   p->cons.secondary = -1;
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

XSIPLUGINCALLBACK CStatus MomentumConsBallSocket_Term( CRef& in_ctxt )
{
   gInstanceCount--;
   Context ctxt( in_ctxt );

   // get the dbid from the userdata
   CValue udVal = ctxt.GetUserData();
   cons_UD * p = (cons_UD*)(CValue::siPtrType)udVal;
   if(gSimulation->DeleteConstraint(p->cons))
      Application().LogMessage(L"[MOMENTUM] Destroyed the constraint for dbID "+CValue((LONG)p->cons.primary).GetAsText(),siVerboseMsg);
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

