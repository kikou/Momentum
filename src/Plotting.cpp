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
#include <xsi_status.h>
#include <xsi_customoperator.h>
#include <xsi_selection.h>
#include <xsi_command.h>
#include <xsi_factory.h>
#include <xsi_primitive.h>
#include <xsi_kinematics.h>
#include <xsi_outputport.h>
#include <xsi_polygonmesh.h>
#include <xsi_x3dobject.h>
#include <xsi_math.h>
#include <xsi_uitoolkit.h>
#include <xsi_progressbar.h>
#include <xsi_boolarray.h>
#include <xsi_inputport.h>
#include <vector>

#include "bulletSimulation.h"
#include "Plotting.h"

using namespace XSI;
using namespace XSI::MATH;

XSIPLUGINCALLBACK CStatus plot_Momentum_Init( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Command oCmd;
   oCmd = ctxt.GetSource();
   oCmd.PutDescription(L"Plot the selected Momentum objects.");
   oCmd.SetFlag(siNoLogging,false);
   return CStatus::OK;
}

XSIPLUGINCALLBACK CStatus plot_Momentum_Execute( CRef& in_ctxt )
{
   Context ctxt( in_ctxt );
   Selection l_pSelection = Application().GetSelection();

   CValueArray cmdArgs(1);
   CValue returnVal;

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

      Application().LogMessage(L"The object "+l_pSelObj.GetFullName()+L" is not valid for Momentum Plotting!",siWarningMsg);
   }

   if(ops.GetCount()==0)
   {
      Application().LogMessage(L"Please select any Momentum based object!",siErrorMsg);
      return CStatus::Unexpected;
   }

   // check if we have the user data blobs and remember the kinematics
   CBoolArray isValid;
   int validCount = 0;
   CRefArray kineStates;
   CRefArray blobs;
   for(LONG i=0;i<ops.GetCount();i++)
   {
      CustomOperator op(ops[i]);

      // get the last connected target
      CRefArray ports = op.GetInputPorts();
      UserDataBlob blob(InputPort(ports[ports.GetCount()-1]).GetTarget());
      if(!blob.IsValid())
      {
         Application().LogMessage(L"The operator "+op.GetFullName()+L" doesn't have a plotting blob connected!",siWarningMsg);
         isValid.Add(false);
         continue;
      }

      // get the 3d object
      X3DObject obj(op.GetParent3DObject());

      // remember the blob and the kinestate
      blobs.Add(blob.GetRef());
      kineStates.Add(obj.GetKinematics().GetGlobal().GetRef());
      isValid.Add(true);
      validCount++;
   }

   if(validCount == 0)
   {
      Application().LogMessage(L"Que mierda! All selected Momentum operators are corrupted!",siErrorMsg);
      Application().LogMessage(L"The scene is probably older than Momentum 1.3!",siErrorMsg);
      return CStatus::OK;
   }

   Application().LogMessage(L"Va bene, plotting "+CValue((LONG)validCount).GetAsText()+L" Momentum operators!");

   // get the input and output frame
   CRef paramRef;
   paramRef.Set(L"PlayControl.In");
   LONG frameIn = Parameter(paramRef).GetValue();
   paramRef.Set(L"PlayControl.Out");
   LONG frameOut = Parameter(paramRef).GetValue();
   paramRef.Set(L"PlayControl.Current");
   Parameter currentFrameParam(paramRef);

   // disable plotting in case it is still on
   for(LONG i=0;i<ops.GetCount();i++)
   {
      if(!isValid[i])
         continue;

      CustomOperator op(ops[i]);
      op.PutParameterValue(L"plotted",false);
   }

   // let's prepare the plot data
   // for this we will have to move to the first frame, refresh,
   // and then see how many rigid bodies we have for the operator!
   currentFrameParam.PutValue(frameIn);
   cmdArgs[0] = frameIn;
   Application().ExecuteCommand(L"Refresh",cmdArgs,returnVal);
   std::vector<PLOT_UD*> plotData(ops.GetCount(),NULL);
   int totalNbTransforms = 0;
   for(LONG i=0;i<ops.GetCount();i++)
   {
      if(!isValid[i])
         continue;

      CustomOperator op(ops[i]);

      int nbTransforms = gSimulation->GetNbBodies((int)op.GetObjectID());
      totalNbTransforms += nbTransforms;
      plotData[i] = new PLOT_UD();
      plotData[i]->Allocate((int)frameIn-1,(int)frameOut-1,nbTransforms);
   }

   // great, now we have the data, let's plot!
   ProgressBar prog = Application().GetUIToolkit().GetProgressBar();
   prog.PutMinimum(frameIn);
   prog.PutMaximum(frameOut);
   prog.PutCancelEnabled(true);
   prog.PutCaption(L"Plotting "+CValue((LONG)totalNbTransforms).GetAsText()+L" Momentum Rigid bodies...");
   prog.PutVisible(true);
   bool canceled = false;

   // loop over all frames
   for(LONG frame=frameIn; frame <= frameOut; frame++)
   {
      currentFrameParam.PutValue(frame);
      cmdArgs[0] = frame;
      Application().ExecuteCommand(L"Refresh",cmdArgs,returnVal);

      // get the transform
      for(LONG i=0;i<ops.GetCount();i++)
      {
         if(!isValid[i])
            continue;

         CustomOperator op(ops[i]);
         int nbTransforms = plotData[i]->GetNbTransforms();
         if(op.GetType().IsEqualNoCase(L"momentumkinematics") && nbTransforms == 1)
         {
            // we only have one transform, let's do this!
            rbdID id(op.GetObjectID(),-1);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(id);
            btTransform xf;
            if(bodyRef != NULL)
               xf = bodyRef->GetWorldTransform();

            Application().LogMessage(L"Operator "+op.GetFullName()+L" frame "+CValue(frame).GetAsText());

            plotData[i]->SetPosition(frame-1,0,xf.getOrigin());
            plotData[i]->SetRotation(frame-1,0,xf.getRotation());
         }
         else
         {
            for(int j=0;j<nbTransforms;j++)
            {
               rbdID id(op.GetObjectID(),j);
               btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(id);
               btTransform xf;
               if(bodyRef != NULL)
                  xf = bodyRef->GetWorldTransform();

               plotData[i]->SetPosition(frame-1,j,xf.getOrigin());
               plotData[i]->SetRotation(frame-1,j,xf.getRotation());
            }
         }
      }
      prog.Increment();
      if(prog.IsCancelPressed())
      {
         canceled = true;
         break;
      }
   }

   prog.PutVisible(false);

   Application().LogMessage(L"Done plotting... freeing memory now.");

   // finally, let's output the data to the user data blobs and free memory
   for(LONG i=0;i<ops.GetCount();i++)
   {
      if(!isValid[i])
         continue;

      if(!canceled)
      {
         UserDataBlob udb(blobs[i]);
         udb.PutValue(plotData[i]->buffer,plotData[i]->size);

         // now set the plotted setting on the operator
         CustomOperator op(ops[i]);
         op.PutParameterValue(L"plotted",true);
      }

      plotData[i]->Deallocate();
      delete(plotData[i]);
   }


   return CStatus::OK;
}
