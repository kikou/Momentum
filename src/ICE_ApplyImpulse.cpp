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

// MOM_GetContactPoints Plugin
// Initial code generated by Softimage SDK Wizard
// Executed Sun Sep 12 11:22:35 UTC+0100 2010 by helge-m
//
//
// Tip: You need to compile the generated code before you can load the plug-in.
// After you compile the plug-in, you can load it by clicking Update All in the Plugin Manager.
#include <xsi_application.h>
#include <xsi_context.h>
#include <xsi_pluginregistrar.h>
#include <xsi_status.h>

#include <xsi_icenodecontext.h>
#include <xsi_icenodedef.h>
#include <xsi_command.h>
#include <xsi_factory.h>
#include <xsi_longarray.h>
#include <xsi_doublearray.h>
#include <xsi_math.h>
#include <xsi_vector2f.h>
#include <xsi_vector3f.h>
#include <xsi_vector4f.h>
#include <xsi_matrix3f.h>
#include <xsi_matrix4f.h>
#include <xsi_rotationf.h>
#include <xsi_quaternionf.h>
#include <xsi_color4f.h>
#include <xsi_shape.h>
#include <xsi_icegeometry.h>
#include <xsi_iceportstate.h>
#include <xsi_indexset.h>
#include <xsi_dataarray.h>
#include <xsi_dataarray2D.h>
#include "Momentum.h"

#include "bulletSimulation.h"
#include "ICE_ApplyImpulse.h"

// Defines port, group and map identifiers used for registering the ICENode
enum IDs
{
	ID_IN_base = 0,
	ID_IN_id = 1,
	ID_IN_mode = 2,
	ID_IN_origin = 3,
	ID_IN_impulse = 4,
	ID_G_100 = 100,
	ID_OUT_base = 200,
	ID_TYPE_CNS = 400,
	ID_STRUCT_CNS,
	ID_CTXT_CNS,
	ID_UNDEF = ULONG_MAX
};

using namespace XSI;
using namespace MATH;

CStatus Register_MOM_ApplyImpulse( PluginRegistrar& in_reg )
{
	ICENodeDef nodeDef;
	nodeDef = Application().GetFactory().CreateICENodeDef(L"MOM_ApplyImpulse",L"MOM_ApplyImpulse");

	CStatus st;
	st = nodeDef.PutColor(255,188,102);
	st.AssertSucceeded( ) ;

	st = nodeDef.PutThreadingModel(XSI::siICENodeSingleThreading);
	st.AssertSucceeded( ) ;

	// Add input ports and groups.
	st = nodeDef.AddPortGroup(ID_G_100);
	st.AssertSucceeded( ) ;

	st = nodeDef.AddInputPort(ID_IN_base,ID_G_100,siICENodeDataLong,siICENodeStructureSingle,siICENodeContextComponent0D,L"base",L"base",0,0,1000000,ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(ID_IN_id,ID_G_100,siICENodeDataLong,siICENodeStructureSingle,siICENodeContextComponent0D,L"id",L"id",0,0,1000000,ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(ID_IN_mode,ID_G_100,siICENodeDataLong,siICENodeStructureSingle,siICENodeContextComponent0D,L"mode",L"mode",1,0,1,ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(ID_IN_origin,ID_G_100,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextComponent0D,L"origin",L"origin",0,0,0,ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(ID_IN_impulse,ID_G_100,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextComponent0D,L"impulse",L"impulse",0,0,0,ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;

	// Add output ports.
	st = nodeDef.AddOutputPort(ID_OUT_base,siICENodeDataLong,siICENodeStructureSingle,siICENodeContextComponent0D,L"outbase",L"outbase",ID_UNDEF,ID_UNDEF,ID_UNDEF);
	st.AssertSucceeded( ) ;

	PluginItem nodeItem = in_reg.RegisterICENode(nodeDef);
	nodeItem.PutCategories(L"Custom ICENode");

	return CStatus::OK;
}


SICALLBACK MOM_ApplyImpulse_Evaluate( ICENodeContext& in_ctxt )
{
	// The current output port being evaluated...
	ULONG out_portID = in_ctxt.GetEvaluatedOutputPortID( );

	switch( out_portID )
	{
		case ID_OUT_base :
		{
			// Get the output port array ...
			CDataArrayLong outData( in_ctxt );

         // see if we have a simulation
         if(gSimulation == NULL)
            return CStatus::OK;

 			// Get the input data buffers for each port
			CDataArrayLong baseData( in_ctxt, ID_IN_base );
			CDataArrayLong idData( in_ctxt, ID_IN_id );
			CDataArrayLong modeData( in_ctxt, ID_IN_mode );
			CDataArrayVector3f originData( in_ctxt, ID_IN_origin );
			CDataArrayVector3f impulseData( in_ctxt, ID_IN_impulse );

         // get the index set iterator
         btTransform bodyTransform;
			rbdID rbd_ID;
			CIndexSet indexSet( in_ctxt );
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
            {
               CVector3f originf = originData.IsConstant() ? originData[0] : originData[it];
               CVector3f impulsef = impulseData.IsConstant() ? impulseData[0] : impulseData[it];
               btVector3 origin = btVector3(originf.GetX(),originf.GetY(),originf.GetZ());
               btVector3 impulse = btVector3(impulsef.GetX(),impulsef.GetY(),impulsef.GetZ());

               impulse = (impulse / bodyRef->body->getInvMass()) * gSimulation->GetFps();

               bodyTransform = bodyRef->GetWorldTransform();

               int mode = modeData.IsConstant() ? modeData[0] : modeData[it];
               if(mode == 0)
                  bodyRef->body->applyCentralForce(impulse);
               else if(mode == 1)
               {
                  origin = bodyTransform.inverse() * origin;
                  bodyRef->body->applyForce(impulse,origin);
               }
            }
            outData[it] = rbd_ID.primary;
			}
		}
		break;

		// Other output ports...
	};

	return CStatus::OK;
}
