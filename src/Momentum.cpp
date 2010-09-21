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
#include "ICE_GetContactPoints.h"
#include "ICE_GetAllVelocities.h"
#include "ICE_CreateRigidBodies.h"
#include "ICE_GetRigidBodyXF.h"
#include "ICE_ApplyImpulse.h"
#include "ICE_GetAttributes.h"
#include "ICE_SetAttributes.h"
#include "ICE_AddToCluster.h"
#include "ICE_RemoveFromCluster.h"
#include "ICE_GetNbBodies.h"

using namespace XSI;
using namespace XSI::MATH;
using namespace XSI::MATH;
XSIPLUGINCALLBACK CStatus XSILoadPlugin( PluginRegistrar& in_reg )
{
   in_reg.PutAuthor(L"hmathee");
   in_reg.PutName(L"Momentum_Plugin");
   in_reg.PutEmail(L"");
   in_reg.PutURL(L"");
   in_reg.PutVersion(1,3);
   in_reg.RegisterOperator(L"MomentumKinematics");
   in_reg.RegisterOperator(L"MomentumDeform");
   in_reg.RegisterOperator(L"MomentumSurface");
   in_reg.RegisterOperator(L"MomentumConsBallSocket");
   in_reg.RegisterOperator(L"MomentumConsSlider");
   in_reg.RegisterOperator(L"MomentumConsHinge");
   in_reg.RegisterOperator(L"MomentumConsSpring");
   in_reg.RegisterOperator(L"MomentumModifier");
   in_reg.RegisterOperator(L"MomentumCopyID");
   in_reg.RegisterCommand(L"apply_MomentumKinematics",L"apply_MomentumKinematics");
   in_reg.RegisterCommand(L"apply_MomentumDeform",L"apply_MomentumDeform");
   in_reg.RegisterCommand(L"apply_MomentumDeform_CreateControls",L"apply_MomentumDeform_CreateControls");
   in_reg.RegisterCommand(L"apply_MomentumSurface",L"apply_MomentumSurface");
   in_reg.RegisterCommand(L"apply_MomentumConsBallSocket",L"apply_MomentumConsBallSocket");
   in_reg.RegisterCommand(L"apply_MomentumConsSlider",L"apply_MomentumConsSlider");
   in_reg.RegisterCommand(L"apply_MomentumConsHinge",L"apply_MomentumConsHinge");
   in_reg.RegisterCommand(L"apply_MomentumConsSpring",L"apply_MomentumConsSpring");
   in_reg.RegisterCommand(L"apply_MomentumModifier",L"apply_MomentumModifier");
   in_reg.RegisterCommand(L"inspect_Momentum",L"inspect_Momentum");
   in_reg.RegisterCommand(L"plot_Momentum",L"plot_Momentum");
   in_reg.RegisterEvent(L"MomentumTimeChange",siOnTimeChange);
   in_reg.RegisterEvent(L"MomentumOnEndSceneOpen",siOnEndSceneOpen);
   in_reg.RegisterProperty(L"MomentumSettings");

   // register the ICE nodes
   Register_MOM_GetContactPoints(in_reg);
   Register_MOM_GetAllVelocities(in_reg);
   Register_MOM_CreateRigidBodies(in_reg);
   Register_MOM_GetRigidBodyXF(in_reg);
   Register_MOM_ApplyImpulse(in_reg);
   Register_MOM_GetAttributes(in_reg);
   Register_MOM_SetAttributes(in_reg);
   Register_MOM_AddToCluster(in_reg);
   Register_MOM_RemoveFromCluster(in_reg);
   Register_MOM_GetNbBodies(in_reg);

   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus XSIUnloadPlugin( const PluginRegistrar& in_reg )
{
   CString strPluginName;
   strPluginName = in_reg.GetName();
   Application().LogMessage(strPluginName + L" has been unloaded.",siVerboseMsg);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus inspect_Momentum_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Inspect the selected Momentum operators.");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus inspect_Momentum_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Selection l_pSelection = Application().GetSelection();

   // search the selection for any compatible 3d object
   CRefArray ops;
   for(long i=0;i<l_pSelection.GetCount();i++)
   {
      X3DObject l_pSelObj(l_pSelection.GetItem(i));
      if(!l_pSelObj.IsValid())
         continue;
      CRef ref;
      ref.Set(l_pSelObj.GetFullName()+L".kine.global.MomentumKinematics");
      if(ref.IsValid())
      {
         ops.Add(ref);
         continue;
      }

      ref.Set(l_pSelObj.GetFullName()+L".polymsh.MomentumDeform");
      if(ref.IsValid())
      {
         ops.Add(ref);
         continue;
      }

      ref.Set(l_pSelObj.GetFullName()+L".polymsh.MomentumSurface");
      if(ref.IsValid())
      {
         ops.Add(ref);
         continue;
      }

      ref.Set(l_pSelObj.GetFullName()+L".kine.global.MomentumModifier");
      if(ref.IsValid())
      {
         ops.Add(ref);
         continue;
      }

      ref.Set(l_pSelObj.GetFullName()+L".kine.global.MomentumConsBallSocket");
      if(ref.IsValid())
      {
         ops.Add(ref);
         continue;
      }

      ref.Set(l_pSelObj.GetFullName()+L".kine.global.MomentumConsSlider");
      if(ref.IsValid())
      {
         ops.Add(ref);
         continue;
      }

      ref.Set(l_pSelObj.GetFullName()+L".kine.global.MomentumConsHinge");
      if(ref.IsValid())
      {
         ops.Add(ref);
         continue;
      }

      ref.Set(l_pSelObj.GetFullName()+L".kine.global.MomentumConsSpring");
      if(ref.IsValid())
      {
         ops.Add(ref);
         continue;
      }
   }

   if(ops.GetCount()==0)
   {
      Application().LogMessage(L"Please select any Momentum based object!",siErrorMsg);
      return CStatus::Unexpected;
   }

   CString choice;
   for(LONG i=0;i<ops.GetCount();i++)
   {
      if(i>0)
         choice += L",";
      choice += ops[i].GetAsText();
   }

   CValue returnVal;
   CValueArray args(1);
   args[0] = choice;
   Application().ExecuteCommand(L"InspectObj",args,returnVal);

   return CStatus::OK;
}

SICALLBACK MomentumTimeChange_OnEvent( CRef& in_ctxt )
{
   // only do this if we have a bullet simulation
   if(gSimulation == NULL)
      return CStatus::OK;

   // what we need to do here is step the simulation
	Context ctxt( in_ctxt );
	int frame = (int)ceil((float)ctxt.GetAttribute(L"Frame"));
	frame--;
   gSimulation->SetFrame(frame);

	return CStatus::OK;
}

SICALLBACK MomentumOnEndSceneOpen_OnEvent( CRef& in_ctxt )
{
   // only do this if we have a bullet simulation
   if(gSimulation == NULL)
      return CStatus::OK;

   // find all deform operators!
   CRefArray deformOps;
   deformOps.Set(L"*.polymsh.MomentumDeform,*.*.polymsh.MomentumDeform,*.polymsh.MomentumSurface,*.*.polymsh.MomentumSurface,");
   for(LONG i=0;i<deformOps.GetCount();i++)
   {
      CustomOperator op(deformOps[i]);
      if(!op.IsValid())
         continue;

      // we will change a flag on them
      // to enforce eval!
      op.PutMute(true);
      op.PutMute(false);
   }

   // reset the gravity etc
   gSimulation->ClearWorld();
   gSimulation->ResetSettings();

	return CStatus::OK;
}
