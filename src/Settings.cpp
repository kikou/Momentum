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

// MomentumSettingsPlugin
// Initial code generated by Softimage SDK Wizard
// Executed Fri Sep 10 12:02:56 UTC+0100 2010 by tony-a
//
//
// Tip: The wizard only exposes a small subset of the possible controls
// and layout that can be achieved on a Property Page.  To find out more
// please refer to the Object Model reference documentation for PPGLayout, PPG
// and CustomProperty
//
// Tip: Don't be concerned about achieving the exact ordering of the parameters
// because they can easily be reordered in the second phase.
//
// Tip: Use the "Refresh" option on the Property Page context menu to
// reload your script changes and re-execute the DefineLayout callback.
// Tip: You need to compile the generated code before you can load the plug-in.
// After you compile the plug-in, you can load it by clicking Update All in the Plugin Manager.
#include <xsi_application.h>
#include <xsi_context.h>
#include <xsi_pluginregistrar.h>
#include <xsi_status.h>
#include <xsi_customproperty.h>
#include <xsi_ppglayout.h>
#include <xsi_ppgeventcontext.h>
#include <xsi_selection.h>
using namespace XSI;

SICALLBACK MomentumSettings_Define( CRef& in_ctxt )
{
	Context ctxt( in_ctxt );
	CustomProperty oCustomProperty;
	Parameter oParam;
	oCustomProperty = ctxt.GetSource();
	oCustomProperty.AddParameter(L"substeps",CValue::siInt4,siPersistable,L"",L"",1,1,100,1,20,oParam);
	oCustomProperty.AddParameter(L"gravity_x",CValue::siFloat,siPersistable,L"",L"",0,-10000,10000,-10,10,oParam);
	oCustomProperty.AddParameter(L"gravity_y",CValue::siFloat,siPersistable,L"",L"",-10,-10000,10000,-10,10,oParam);
	oCustomProperty.AddParameter(L"gravity_z",CValue::siFloat,siPersistable,L"",L"",0,-10000,10000,-10,10,oParam);
	return CStatus::OK;
}

SICALLBACK MomentumSettings_DefineLayout( CRef& in_ctxt )
{
	Context ctxt( in_ctxt );
	PPGLayout oLayout;
	PPGItem oItem;
	oLayout = ctxt.GetSource();
	oLayout.Clear();
	oLayout.AddItem(L"substeps",L"Simulation Substeps");
	oLayout.AddGroup(L"Gravity");
	oLayout.AddRow();
	oLayout.AddItem(L"gravity_x",L"x");
	oLayout.AddItem(L"gravity_y",L"y");
	oLayout.AddItem(L"gravity_z",L"z");
	oLayout.EndRow();
	oLayout.EndGroup();
	return CStatus::OK;
}
