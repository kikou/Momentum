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



XSIPLUGINCALLBACK CStatus MomentumConsSlider_Define( CRef& in_ctxt )
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

   oPDef = oFactory.CreateParamDef(L"linearMin",CValue::siFloat,siAnimatable | siPersistable,L"linearMin",L"linearMin",0,-100000,100000,-5,10);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linearMax",CValue::siFloat,siAnimatable | siPersistable,L"linearMax",L"linearMax",10,-100000,100000,-5,10);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linearSoftness",CValue::siFloat,siAnimatable | siPersistable,L"linearSoftness",L"linearSoftness",1,0,100,0,2);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linearDamping",CValue::siFloat,siAnimatable | siPersistable,L"linearDamping",L"linearDamping",1,0,100,0,2);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linearBounce",CValue::siFloat,siAnimatable | siPersistable,L"linearBounce",L"linearBounce",.7,0,100,0,2);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linearMotor",CValue::siBool,siAnimatable | siPersistable,L"linearMotor",L"linearMotor",0,0,1,0,1);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linearMotorForce",CValue::siFloat,siAnimatable | siPersistable,L"linearMotorForce",L"linearMotorForce",.1,-10000,10000,-.5,.5);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"linearMotorVelocity",CValue::siFloat,siAnimatable | siPersistable,L"linearMotorVelocity",L"linearMotorVelocity",5,0,10000,0,10);
   oCustomOperator.AddParameter(oPDef,oParam);


   oPDef = oFactory.CreateParamDef(L"angularMin",CValue::siFloat,siAnimatable | siPersistable,L"angularMin",L"angularMin",0,-100000,100000,-180,180);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angularMax",CValue::siFloat,siAnimatable | siPersistable,L"angularMax",L"angularMax",0,-100000,100000,-180,180);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angularSoftness",CValue::siFloat,siAnimatable | siPersistable,L"angularSoftness",L"angularSoftness",1,0,100,0,2);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angularDamping",CValue::siFloat,siAnimatable | siPersistable,L"angularDamping",L"angularDamping",1,0,100,0,2);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angularBounce",CValue::siFloat,siAnimatable | siPersistable,L"angularBounce",L"angularBounce",.7,0,100,0,2);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angularMotor",CValue::siBool,siAnimatable | siPersistable,L"angularMotor",L"angularMotor",0,0,1,0,1);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angularMotorForce",CValue::siFloat,siAnimatable | siPersistable,L"angularMotorForce",L"angularMotorForce",.1,-10000,10000,-.5,.5);
   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"angularMotorVelocity",CValue::siFloat,siAnimatable | siPersistable,L"angularMotorVelocity",L"angularMotorVelocity",5,0,10000,0,10);
   oCustomOperator.AddParameter(oPDef,oParam);

   oCustomOperator.PutAlwaysEvaluate(false);

   oCustomOperator.PutDebug(0);

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumConsSlider_DefineLayout( CRef& in_ctxt )
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
   oLayout.AddGroup(L"Linear");
   oLayout.AddItem(L"linearMin",L"Minimum");
   oLayout.AddItem(L"linearMax",L"Maximum");
   oLayout.AddItem(L"linearSoftness",L"Softness");
   oLayout.AddItem(L"linearDamping",L"Damping");
   oLayout.AddItem(L"linearBounce",L"Bounce");
   oLayout.AddItem(L"linearMotor",L"Motor");
   oLayout.AddItem(L"linearMotorForce",L"Force");
   oLayout.AddItem(L"linearMotorVelocity",L"Velocity");
   oLayout.EndGroup();
   oLayout.AddGroup(L"Angular");
   oLayout.AddItem(L"angularMin",L"Minimum");
   oLayout.AddItem(L"angularMax",L"Maximum");
   oLayout.AddItem(L"angularSoftness",L"Softness");
   oLayout.AddItem(L"angularDamping",L"Damping");
   oLayout.AddItem(L"angularBounce",L"Bounce");
   oLayout.AddItem(L"angularMotor",L"Motor");
   oLayout.AddItem(L"angularMotorForce",L"Force");
   oLayout.AddItem(L"angularMotorVelocity",L"Velocity");
   oLayout.EndGroup();

   return CStatus::OK;

}


XSIPLUGINCALLBACK CStatus MomentumConsSlider_Update( CRef& in_ctxt )
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

      // we need to compute both transforms
      btTransform pivotA;
      pivotA.setIdentity();
      pivotA.setOrigin(btVector3(xfA.GetPosX(),xfA.GetPosY(),xfA.GetPosZ()));
      CQuaternion q =xfA.GetRotationQuaternion();
      pivotA.setRotation(btQuaternion(q.GetX(),q.GetY(),q.GetZ(),q.GetW()));
      btTransform pivotB = pivotA;
      pivotA = bodyTransformA.inverse() * pivotA;
      pivotB = bodyTransformB.inverse() * pivotB;

      // cool, we have both bodies
      // let's create the constraint!
      consRef = new btConstraintReference();
      consRef->bodyA = bodyRefA->body;
      consRef->bodyB = bodyRefB->body;
      consRef->bodyIdA = bodyIdA;
      consRef->bodyIdB = bodyIdB;
      consRef->pivotA = pivotA;
      consRef->pivotB = pivotB;

      consRef->constraint = new btSliderConstraint(
            *consRef->bodyA,*consRef->bodyB,pivotA,pivotB,true);

      // now create it
      gSimulation->AddConstraint(p->cons,consRef);

      Application().LogMessage(L"[MOMENTUM] Created the constraint for dbID "+CValue((LONG)p->cons.primary).GetAsText(),siVerboseMsg);
   }

   // if we have a constraint
   if(consRef != NULL)
   {
      // update a couple of things on the constraint!
      btSliderConstraint * slider = (btSliderConstraint *)consRef->constraint;

      slider->setLowerLinLimit((float)ctxt.GetParameterValue(L"linearMin"));
      slider->setUpperLinLimit((float)ctxt.GetParameterValue(L"linearMax"));
      slider->setSoftnessLimLin((float)ctxt.GetParameterValue(L"linearSoftness"));
      slider->setDampingLimLin((float)ctxt.GetParameterValue(L"linearDamping"));
      slider->setRestitutionLimLin((float)ctxt.GetParameterValue(L"linearBounce"));
      slider->setPoweredLinMotor((bool)ctxt.GetParameterValue(L"linearMotor"));
      slider->setMaxLinMotorForce((float)ctxt.GetParameterValue(L"linearMotorForce"));
      slider->setTargetLinMotorVelocity((float)ctxt.GetParameterValue(L"linearMotorVelocity"));

      slider->setLowerAngLimit((float)ctxt.GetParameterValue(L"angularMin"));
      slider->setUpperAngLimit((float)ctxt.GetParameterValue(L"angularMax"));
      slider->setSoftnessLimAng((float)ctxt.GetParameterValue(L"angularSoftness"));
      slider->setDampingLimAng((float)ctxt.GetParameterValue(L"angularDamping"));
      slider->setRestitutionLimAng((float)ctxt.GetParameterValue(L"angularBounce"));
      slider->setPoweredAngMotor((bool)ctxt.GetParameterValue(L"angularMotor"));
      slider->setMaxAngMotorForce((float)ctxt.GetParameterValue(L"angularMotorForce"));
      slider->setTargetAngMotorVelocity((float)ctxt.GetParameterValue(L"angularMotorVelocity"));
   }

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus MomentumConsSlider_Init( CRef& in_ctxt )
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

XSIPLUGINCALLBACK CStatus MomentumConsSlider_Term( CRef& in_ctxt )
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



XSIPLUGINCALLBACK CStatus apply_MomentumConsSlider_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;

   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Create an instance of MomentumConsSlider operator");

   oCmd.SetFlag(siNoLogging,false);

   return CStatus::OK;

}



XSIPLUGINCALLBACK CStatus apply_MomentumConsSlider_Execute( CRef& in_ctxt )
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
   obj1.AddNull(L"Slider_pivot",consNull1);
   consNull1.GetKinematics().GetGlobal().PutTransform(xfA);
   consNull1.GetActivePrimitive().PutParameterValue(L"primary_icon", (LONG)0);
   consNull1.GetActivePrimitive().PutParameterValue(L"shadow_icon", (LONG)4);
   consNull1.GetActivePrimitive().PutParameterValue(L"shadow_scaleX", (LONG)3);

   // create the operator

   CustomOperator newOp = Application().GetFactory().CreateObject(L"MomentumConsSlider");

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


