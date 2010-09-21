#ifndef __PLOTTING_H__
#define __PLOTTING_H__

#include <btBulletDynamicsCommon.h>
#include <xsi_userdatablob.h>

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

class PLOT_UD
{
   public:
      const unsigned char * buffer;
      unsigned int size;

   PLOT_UD()
   {
      size = 0;
      buffer = NULL;
   }

   void Allocate(int in_inFrame, int in_outFrame, int in_nbTransforms)
   {
      if(size != 0 || buffer != NULL)
         Deallocate();

      // 3 for frameIn, frameOut and nbTransforms
      // 7 is posXYZ and quatWXYZ * nbFrames * nbTransforms
      size = (3 + (in_outFrame - in_inFrame + 2) * in_nbTransforms * 7) * sizeof(float);
      buffer = (unsigned char*)malloc(size);

      // now store at least the three counters
      SetFrameIn(in_inFrame);
      SetFrameOut(in_outFrame);
      SetNbTransforms(in_nbTransforms);
   }

   void Deallocate()
   {
      size = 0;
      if(buffer != NULL)
      {
         free((unsigned char*)buffer);
         buffer = NULL;
      }
   }

   int GetFrameIn()
   {
      if(size == 0 || buffer == NULL)
         return 0;
      return (int)((float*)buffer)[0];
   }
   int GetFrameOut()
   {
      if(size == 0 || buffer == NULL)
         return 0;
      return (int)((float*)buffer)[1];
   }
   int GetNbTransforms()
   {
      if(size == 0 || buffer == NULL)
         return 0;
      return (int)((float*)buffer)[2];
   }
   void SetFrameIn(int in_inFrame)
   {
      if(size == 0 || buffer == NULL)
         return;
      ((float*)buffer)[0] = (float)in_inFrame;
   }
   void SetFrameOut(int in_outFrame)
   {
      if(size == 0 || buffer == NULL)
         return;
      ((float*)buffer)[1] = (float)in_outFrame;
   }
   void SetNbTransforms(int in_nbTransforms)
   {
      if(size == 0 || buffer == NULL)
         return;
      ((float*)buffer)[2] = (float)in_nbTransforms;
   }
   btVector3 GetPosition(int in_Frame, int in_Transform)
   {
      btVector3 result;
      if(size != 0 && buffer != NULL)
      {
         int offset = 3 + in_Frame * GetNbTransforms() * 7 + in_Transform * 7;
         result.setX(((float*)buffer)[offset++]);
         result.setY(((float*)buffer)[offset++]);
         result.setZ(((float*)buffer)[offset++]);
      }
      return result;
   }
   void SetPosition(int in_Frame, int in_Transform, btVector3 in_Position)
   {
      if(size == 0 || buffer == NULL)
         return;
      if(in_Frame < GetFrameIn() || in_Frame > GetFrameOut() || in_Transform < 0 || in_Transform >= GetNbTransforms())
         return;

      int offset = 3 + in_Frame * GetNbTransforms() * 7 + in_Transform * 7;
      ((float*)buffer)[offset++] = in_Position.getX();
      ((float*)buffer)[offset++] = in_Position.getY();
      ((float*)buffer)[offset++] = in_Position.getZ();
   }
   btQuaternion GetRotation(int in_Frame, int in_Transform)
   {
      btQuaternion result;
      if(size != 0 && buffer != NULL)
      {
         int offset = 3 + in_Frame * GetNbTransforms() * 7 + in_Transform * 7 + 3;
         result.setW(((float*)buffer)[offset++]);
         result.setX(((float*)buffer)[offset++]);
         result.setY(((float*)buffer)[offset++]);
         result.setZ(((float*)buffer)[offset++]);
      }
      return result;
   }
   void SetRotation(int in_Frame, int in_Transform, btQuaternion in_Rotation)
   {
      if(size == 0 || buffer == NULL)
         return;
      if(in_Frame < GetFrameIn() || in_Frame > GetFrameOut() || in_Transform < 0 || in_Transform >= GetNbTransforms())
         return;

      int offset = 3 + in_Frame * GetNbTransforms() * 7 + in_Transform * 7 + 3;
      ((float*)buffer)[offset++] = in_Rotation.getW();
      ((float*)buffer)[offset++] = in_Rotation.getX();
      ((float*)buffer)[offset++] = in_Rotation.getY();
      ((float*)buffer)[offset++] = in_Rotation.getZ();
   }
};

#endif
