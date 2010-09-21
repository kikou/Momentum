#ifndef __BULLETSIMULATION_H__
#define __BULLETSIMULATION_H__

#include <btBulletDynamicsCommon.h>
#include <map>
#include <xsi_ref.h>
#include <vector>

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

struct rbdID
{
   int primary;
   int secondary;

   rbdID()
   {
      primary = secondary = -1;
   }

   rbdID(int in_PRIMARY, int in_SECONDARY)
   {
      primary = in_PRIMARY;
      secondary = in_SECONDARY;
   }

   bool operator < (const rbdID & in_t) const
   {
      if(primary < in_t.primary)
         return true;
      else if(primary > in_t.primary)
         return false;
      return secondary < in_t.secondary;
   }
};

class btRigidBodyReference
{
public:
      btRigidBody * body;
      XSI::CRef op;
      XSI::CRef kine;

      // cluster management
      btRigidBodyReference * cluster;
      btTransform clusterOffset;
      bool isACluster;
      float mass;
      btAlignedObjectArray<btRigidBodyReference*> children;

      btRigidBodyReference()
      {
         body = NULL;
         cluster = NULL;
         isACluster = false;
         mass = 1.0f;
      }

      void AddToCluster(btRigidBodyReference * in_pCluster);
      void RemoveFromCluster();
      btTransform GetWorldTransform(bool bRecurseToCluster = true);
      void SetWorldTransform(const btTransform & in_XF, bool bRecreateMotionState = false);
};

class btCollisionShapeReference
{
public:
      btCollisionShape * shape;
      XSI::CRef op;
      XSI::CRef prim;
      btTriangleIndexVertexArray * tris;
      int * triId;
      btScalar * triPos;

      btCollisionShapeReference()
      {
         shape = NULL;
         tris = NULL;
         triId = NULL;
         triPos = NULL;
      }
};

struct btConstraintReference
{
public:
      btTypedConstraint * constraint;
      btRigidBody * bodyA;
      btRigidBody * bodyB;
      btTransform pivotA;
      btTransform pivotB;
      rbdID bodyIdA;
      rbdID bodyIdB;

      btConstraintReference()
      {
         constraint = NULL;
         bodyA = NULL;
         bodyA = NULL;
      }
};

typedef std::map<rbdID,btRigidBodyReference*> btRigidBodyMap;
typedef std::pair<rbdID,btRigidBodyReference*> btRigiBodyPair;
typedef btRigidBodyMap::iterator btRigidBodyIt;

typedef std::map<rbdID,btCollisionShapeReference*> btCollisionShapeMap;
typedef std::pair<rbdID,btCollisionShapeReference*> btCollisionShapePair;
typedef btCollisionShapeMap::iterator btCollisionShapeIt;

typedef std::map<rbdID,btConstraintReference*> btConstraintMap;
typedef std::pair<rbdID,btConstraintReference*> btConstraintPair;
typedef btConstraintMap::iterator btConstraintIt;

class bulletSimulation
{
	//keep the bodies and collision shapes, for deletion/cleanup
	btRigidBodyMap mRigidBodies;
	btCollisionShapeMap mCollisionShapes;
	btConstraintMap mConstraints;

	btDynamicsWorld * mDynamicsWorld;
	btBroadphaseInterface*	mBroadphase;
	btCollisionDispatcher*	mDispatcher;
	btConstraintSolver*	mSolver;
	btDefaultCollisionConfiguration* mCollisionConfiguration;
	int mLastFrame;
	float mFps;
	bool mChangingFrame;
	float mLinearDamping;
	float mAngularDamping;
	int mSubSteps;

	public:

   // constructor
	bulletSimulation();

   // destructor
	virtual ~bulletSimulation();

	// setters
	void ClearWorld();
	void ResetWorld();
	void ResetSettings();
	bool SetFrame(int in_Frame);
	void SetDamping(float in_Linear, float in_Angular) { mLinearDamping = in_Linear; mAngularDamping = in_Angular; }
	bool IsChangingFrame() const { return mChangingFrame; }
	btDynamicsWorld * GetDynamicsWorld() { return mDynamicsWorld; }
	float GetFps() { return mFps; }

   // rigid bodies
   int GetNbBodies(int in_Base = -1);
   bool AddRigidBody(const rbdID & in_ID, btRigidBodyReference * in_BodyRef);
   bool DeleteRigidBody(const rbdID & in_ID);
   bool DeleteAllRigidBodies(const rbdID & in_ID = rbdID());
   btRigidBodyReference * GetRigidBody(const rbdID & in_ID);
   btRigidBodyIt GetRigidBodyItBegin() { return mRigidBodies.begin(); }
   btRigidBodyIt GetRigidBodyitEnd() { return mRigidBodies.end(); }

   // collision shapes
   bool AddCollisionShape(const rbdID & in_ID, btCollisionShapeReference * in_ShapeRef);
   bool DeleteCollisionShape(const rbdID & in_ID);
   bool DeleteAllCollisionShapes(const rbdID & in_ID = rbdID());
   btCollisionShapeReference * GetCollisionShape(const rbdID & in_ID);

   // constraints
   bool AddConstraint(const rbdID & in_ID, btConstraintReference * in_ConstraintRef);
   bool DeleteConstraint(const rbdID & in_ID);
   bool DeleteAllConstraints(const rbdID & in_ID = rbdID());
   btConstraintReference * GetConstraint(const rbdID & in_ID);

   // getters for the ICE nodes
   void GetContactPoints(btAlignedObjectArray<btVector3> * pos, btAlignedObjectArray<btVector3> * vel, int mode = 0, float minImpulse = 1.0f);
   void GetBodyVelocities(btAlignedObjectArray<btVector3> * pos, btAlignedObjectArray<btVector3> * vel, int mode = 0);
};

extern bulletSimulation * gSimulation;
extern int gInstanceCount;

#endif //BASIC_DEMO_H

