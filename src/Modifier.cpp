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

using namespace XSI;
using namespace MATH;

XSIPLUGINCALLBACK CStatus apply_MomentumModifier_Init( CRef& in_ctxt )

{

   Context ctxt( in_ctxt );

   Command oCmd;

   oCmd = ctxt.GetSource();

   oCmd.PutDescription(L"Create an instance of MomentumModifier operator");

   oCmd.SetFlag(siNoLogging,false);

   return CStatus::OK;

}



XSIPLUGINCALLBACK CStatus apply_MomentumModifier_Execute( CRef& in_ctxt )
{

   Context ctxt( in_ctxt );

   Selection l_pSelection = Application().GetSelection();


   Null ModifierNull;
   Application().GetActiveSceneRoot().AddNull(L"Modifier",ModifierNull);
   ModifierNull.GetActivePrimitive().PutParameterValue(L"primary_icon",(LONG)2);
   ModifierNull.GetActivePrimitive().PutParameterValue(L"shadow_icon",(LONG)10);
   ModifierNull.GetActivePrimitive().PutParameterValue(L"shadow_offsetZ",1.0f);

   // create the operator

   CustomOperator newOp = Application().GetFactory().CreateObject(L"MomentumModifier");

   newOp.AddOutputPort(ModifierNull.GetKinematics().GetGlobal().GetRef());

   newOp.AddInputPort(ModifierNull.GetKinematics().GetGlobal().GetRef());

   newOp.AddInputPort(ModifierNull.GetActivePrimitive().GetRef());

   newOp.Connect();

   // set the expression for the frame!
   CValue returnVal;
   CValueArray args(2);
   args[0] = newOp.GetFullName()+L".frame";
   args[1] = L"fc";
   Application().ExecuteCommand(L"SetExpr",args,returnVal);

   // select the object
   l_pSelection.Clear();
   l_pSelection.Add(ModifierNull.GetRef());

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumModifier_Define( CRef& in_ctxt )

{

   Context ctxt( in_ctxt );

   CustomOperator oCustomOperator;

   Parameter oParam;

   CRef oPDef;

   Factory oFactory = Application().GetFactory();

   oCustomOperator = ctxt.GetSource();


   oPDef = oFactory.CreateParamDef(L"frame",CValue::siInt4,siAnimatable | siPersistable,L"frame",L"frame",1,-100000,100000,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"mode",CValue::siInt4,siAnimatable | siPersistable,L"mode",L"mode",1,0,1000,0,1000);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"force",CValue::siFloat,siAnimatable | siPersistable,L"force",L"force",1,-10000000,100000000,-10,10);

   oCustomOperator.AddParameter(oPDef,oParam);

   oPDef = oFactory.CreateParamDef(L"falloff",CValue::siBool,siPersistable,L"falloff",L"falloff",1,0,1,0,1);

   oCustomOperator.AddParameter(oPDef,oParam);



   oCustomOperator.PutAlwaysEvaluate(false);

   oCustomOperator.PutDebug(0);

   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumModifier_DefineLayout( CRef& in_ctxt )

{

   Context ctxt( in_ctxt );

   PPGLayout oLayout;

   PPGItem oItem;

   oLayout = ctxt.GetSource();

   oLayout.Clear();
   CValueArray modeItems(6);
   modeItems[0] = L"Center Push";
   modeItems[1] = 0;
   modeItems[2] = L"Corner Torque";
   modeItems[3] = 1;
   modeItems[4] = L"Activate";
   modeItems[5] = 2;
   modeItems[6] = L"Set to Sleep";
   modeItems[7] = 3;
   modeItems[8] = L"Deactivate";
   modeItems[9] = 4;
   oLayout.AddEnumControl(L"mode",modeItems,L"Mode");
   oLayout.AddItem(L"force",L"Force");
   oLayout.AddItem(L"falloff",L"Falloff");
   return CStatus::OK;

}

XSIPLUGINCALLBACK CStatus MomentumModifier_Update( CRef& in_ctxt )
{
   // check if we have a simulation
   if(gSimulation == NULL)
      return CStatus::OK;

   OperatorContext ctxt( in_ctxt );
   int mode = (LONG)ctxt.GetParameterValue(L"mode");
   float force = ctxt.GetParameterValue(L"force");
   if(force == 0.0f)
      return CStatus::OK;

   bool falloff = ctxt.GetParameterValue(L"falloff");

   KinematicState kine = (CRef)ctxt.GetInputValue(0);
   CTransformation xf = kine.GetTransform();
   Primitive prim = (CRef)ctxt.GetInputValue(1);

   btTransform transform;
   transform.setOrigin(btVector3(xf.GetPosX(),xf.GetPosY(),xf.GetPosZ()));
   CQuaternion q = xf.GetRotationQuaternion();
   transform.setRotation(btQuaternion(q.GetX(),q.GetY(),q.GetZ(),q.GetW()));
   btTransform invTransform = transform.inverse();
   btVector3 scale(xf.GetSclX(),xf.GetSclY(),xf.GetSclZ());
   btTransform bodyTransform;
   btVector3 direction = transform.getBasis() * btVector3(0,0,1);

   // loop over all rbds!
   btRigidBodyIt it = gSimulation->GetRigidBodyItBegin();
   for(;it != gSimulation->GetRigidBodyitEnd();it++)
   {
      if(it->second->body == NULL)
         continue;
      if(it->second->body->getMotionState() == NULL)
         continue;

      // compute the local position inside the torque transform
      bodyTransform = it->second->GetWorldTransform();
      btVector3 localPos = (invTransform * bodyTransform.getOrigin()) / scale;

      // check if the rbd is in the sphere
      float length = localPos.length();

      if(length > 1.0f)
         continue;

      // great, now let choose the mode
      if(mode == 0) // center push
      {
         // if the rbd is deactivated, activate it
         if(it->second->body->getActivationState() != DISABLE_DEACTIVATION)
            it->second->body->setActivationState(DISABLE_DEACTIVATION);

         float factor = (falloff ? 1.0f - length : 1.0f) * force * gSimulation->GetFps();
         factor /= it->second->body->getInvMass();
         it->second->body->applyCentralForce(direction * factor);
      }
      else if(mode == 1) // torque force
      {
         // if the rbd is deactivated, activate it
         if(it->second->body->getActivationState() != DISABLE_DEACTIVATION)
            it->second->body->setActivationState(DISABLE_DEACTIVATION);

         float factor = (falloff ? 1.0f - length : 1.0f) * force * gSimulation->GetFps();
         factor /= it->second->body->getInvMass();
         localPos = bodyTransform.inverse() * transform.getOrigin();
         it->second->body->applyForce(direction * factor,localPos);
      }
      else if(mode == 2) // activate
      {
         if(force != 0.0f)
         {
            if(it->second->body->getActivationState() != DISABLE_DEACTIVATION && it->second->body->getActivationState() != ACTIVE_TAG)
               it->second->body->forceActivationState(ACTIVE_TAG);
         }
      }
      else if(mode == 3) // sleep
      {
         if(force != 0.0f)
         {
            if(it->second->body->getActivationState() != ISLAND_SLEEPING)
               it->second->body->forceActivationState(ISLAND_SLEEPING);
         }
      }
      else if(mode == 4) // disable
      {
         if(force != 0.0f)
         {
            if(it->second->body->getActivationState() != DISABLE_SIMULATION)
               it->second->body->forceActivationState(DISABLE_SIMULATION);
         }
      }
   }

   return CStatus::OK;
}
