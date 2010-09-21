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
#include "ICE_GetAttributes.h"

// Defines port, group and map identifiers used for registering the ICENode
enum IDs
{
	ID_IN_base = 0,
	ID_IN_id = 1,
	ID_IN_mode = 2,
	ID_IN_origin = 3,
	ID_IN_impulse = 4,
	ID_G_100 = 100,
	ID_OUT_position = 200,
	ID_OUT_orientation = 201,
	ID_OUT_linvelocity = 202,
	ID_OUT_angvelocity = 203,
	ID_OUT_state = 204,
	ID_OUT_mass = 205,
	ID_OUT_bounce = 206,
	ID_OUT_friction = 207,
	ID_OUT_lindamping = 208,
	ID_OUT_angdamping = 209,
	ID_OUT_lintreshold = 210,
	ID_OUT_angtreshold = 211,
	ID_TYPE_CNS = 400,
	ID_STRUCT_CNS,
	ID_CTXT_CNS,
	ID_UNDEF = ULONG_MAX
};

using namespace XSI;
using namespace MATH;

CStatus Register_MOM_GetAttributes( PluginRegistrar& in_reg )
{
	ICENodeDef nodeDef;
	nodeDef = Application().GetFactory().CreateICENodeDef(L"MOM_GetAttributes",L"MOM_GetAttributes");

	CStatus st;
	st = nodeDef.PutColor(255,188,102);
	st.AssertSucceeded( ) ;

	st = nodeDef.PutThreadingModel(XSI::siICENodeSingleThreading);
	st.AssertSucceeded( ) ;

	// Add input ports and groups.
	st = nodeDef.AddPortGroup(ID_G_100);
	st.AssertSucceeded( ) ;

	st = nodeDef.AddInputPort(ID_IN_base,ID_G_100,siICENodeDataLong,siICENodeStructureSingle,siICENodeContextComponent0D,L"base",L"base",0,0,1000000,ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
   st = nodeDef.AddInputPort(ID_IN_id,ID_G_100,siICENodeDataLong,siICENodeStructureSingle,siICENodeContextComponent0D,L"id",L"id",0,0,1000000,ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;

	// Add output ports.
	st = nodeDef.AddOutputPort(ID_OUT_position,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextComponent0D,L"position",L"position",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_orientation,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextComponent0D,L"orientation",L"orientation",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_linvelocity,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextComponent0D,L"linvelocity",L"linvelocity",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_angvelocity,siICENodeDataVector3,siICENodeStructureSingle,siICENodeContextComponent0D,L"angvelocity",L"angvelocity",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_state,siICENodeDataLong,siICENodeStructureSingle,siICENodeContextComponent0D,L"state",L"state",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_mass,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextComponent0D,L"mass",L"mass",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_bounce,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextComponent0D,L"bounce",L"bounce",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_friction,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextComponent0D,L"friction",L"friction",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_lindamping,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextComponent0D,L"lindamping",L"lindamping",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_angdamping,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextComponent0D,L"angdamping",L"angdamping",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_lintreshold,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextComponent0D,L"lintreshold",L"lintreshold",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;
	st = nodeDef.AddOutputPort(ID_OUT_angtreshold,siICENodeDataFloat,siICENodeStructureSingle,siICENodeContextComponent0D,L"angtreshold",L"angtreshold",ID_UNDEF,ID_UNDEF,ID_CTXT_CNS);
	st.AssertSucceeded( ) ;

	PluginItem nodeItem = in_reg.RegisterICENode(nodeDef);
	nodeItem.PutCategories(L"Custom ICENode");

	return CStatus::OK;
}


SICALLBACK MOM_GetAttributes_Evaluate( ICENodeContext& in_ctxt )
{
	// The current output port being evaluated...
   ULONG out_portID = in_ctxt.GetEvaluatedOutputPortID( );
   CDataArrayLong baseData( in_ctxt, ID_IN_base );
   CDataArrayLong idData( in_ctxt, ID_IN_id );
   rbdID rbd_ID;
   CIndexSet indexSet( in_ctxt );

   if(gSimulation == NULL)
      return CStatus::OK;

	switch( out_portID )
	{
		case ID_OUT_position :
		{
			// Get the output port array ...
			CDataArrayVector3f outData( in_ctxt );

         // get the index set iterator
         btTransform bodyTransform;
         btVector3 bodyPos;
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
            {
               bodyRef->body->getMotionState()->getWorldTransform(bodyTransform);
               bodyPos = bodyTransform.getOrigin();
               outData[it] = CVector3f(bodyPos.getX(),bodyPos.getY(),bodyPos.getZ());
            }
			}
         break;
		}
		case ID_OUT_orientation :
		{
			// Get the output port array ...
			CDataArrayVector3f outData( in_ctxt );

         // get the index set iterator
         btTransform bodyTransform;
         btQuaternion bodyRot;
			CRotation rot;
			CVector3 angles;
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
            {
               bodyTransform = bodyRef->GetWorldTransform();
               bodyRot = bodyTransform.getRotation();
               rot.SetFromQuaternion(CQuaternion(bodyRot.getW(),bodyRot.getX(),bodyRot.getY(),bodyRot.getZ()));
               angles = rot.GetXYZAngles();
               outData[it].Set(RadiansToDegrees(angles.GetX()),RadiansToDegrees(angles.GetY()),RadiansToDegrees(angles.GetZ()));
            }
			}
         break;
		}
		case ID_OUT_linvelocity:
		{
			// Get the output port array ...
			CDataArrayVector3f outData( in_ctxt );

         // get the index set iterator
         btVector3 linvel;
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
            {
               linvel = bodyRef->body->getLinearVelocity();
               outData[it].Set(linvel.getX(),linvel.getY(),linvel.getZ());
            }
			}
         break;
		}
		case ID_OUT_angvelocity:
		{
			// Get the output port array ...
			CDataArrayVector3f outData( in_ctxt );

         // get the index set iterator
         btVector3 angvel;
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
            {
               angvel = bodyRef->body->getAngularVelocity();
               outData[it].Set(angvel.getX(),angvel.getY(),angvel.getZ());
            }
			}
         break;
		}
		case ID_OUT_state:
		{
			// Get the output port array ...
			CDataArrayLong outData( in_ctxt );

         // get the index set iterator
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
            {
               outData[it] = 0;
               if(bodyRef->body->getActivationState() == WANTS_DEACTIVATION)
                  outData[it] = 1;
               else if(bodyRef->body->getActivationState() == DISABLE_SIMULATION)
                  outData[it] = 2;
            }
			}
         break;
		}
		case ID_OUT_mass:
		{
			// Get the output port array ...
			CDataArrayFloat outData( in_ctxt );

         // get the index set iterator
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
               outData[it] = 1.0f / bodyRef->body->getInvMass();
			}
         break;
		}
		case ID_OUT_bounce:
		{
			// Get the output port array ...
			CDataArrayFloat outData( in_ctxt );

         // get the index set iterator
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
               outData[it] = bodyRef->body->getRestitution();
			}
         break;
		}
		case ID_OUT_friction:
		{
			// Get the output port array ...
			CDataArrayFloat outData( in_ctxt );

         // get the index set iterator
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
               outData[it] = bodyRef->body->getFriction();
			}
         break;
		}
		case ID_OUT_lindamping:
		{
			// Get the output port array ...
			CDataArrayFloat outData( in_ctxt );

         // get the index set iterator
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
               outData[it] = bodyRef->body->getLinearDamping();
			}
         break;
		}
		case ID_OUT_angdamping:
		{
			// Get the output port array ...
			CDataArrayFloat outData( in_ctxt );

         // get the index set iterator
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
               outData[it] = bodyRef->body->getAngularDamping();
			}
         break;
		}
		case ID_OUT_lintreshold:
		{
			// Get the output port array ...
			CDataArrayFloat outData( in_ctxt );

         // get the index set iterator
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
               outData[it] = bodyRef->body->getLinearSleepingThreshold();
			}
         break;
		}
		case ID_OUT_angtreshold:
		{
			// Get the output port array ...
			CDataArrayFloat outData( in_ctxt );

         // get the index set iterator
			for(CIndexSet::Iterator it = indexSet.Begin(); it.HasNext(); it.Next())
			{
            rbd_ID.primary = (int)(baseData.IsConstant() ? baseData[0] : baseData[it]);
            rbd_ID.secondary = (int)(idData.IsConstant() ? idData[0] : idData[it]);
            btRigidBodyReference * bodyRef = gSimulation->GetRigidBody(rbd_ID);
            if(bodyRef != NULL)
               outData[it] = bodyRef->body->getAngularSleepingThreshold();
			}
         break;
		}
	};

	return CStatus::OK;
}

