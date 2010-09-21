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

///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "bulletSimulation.h"
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.

// XSI includes
#include <xsi_kinematicstate.h>
#include <xsi_transformation.h>
#include <xsi_quaternion.h>
#include <xsi_customoperator.h>
#include <xsi_application.h>
#include <xsi_customproperty.h>
#include <xsi_model.h>

int gInstanceCount=0;
bulletSimulation * gSimulation=NULL;

bulletSimulation::bulletSimulation()
{
	///collision configuration contains default setup for memory, collision setup
	mCollisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);

	mBroadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	mSolver = sol;

	mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher,mBroadphase,mSolver,mCollisionConfiguration);

	// create the GIMpact collision algorithm
	btGImpactCollisionAlgorithm::registerAlgorithm(mDispatcher);
	mChangingFrame = false;
	mLastFrame = 0;
	mFps = 25.0f;

	ResetWorld();
}

bulletSimulation::~bulletSimulation()
{
   // remove all constraints
   DeleteAllConstraints();
   mConstraints.clear();

   // remove all collision shapes
   DeleteAllCollisionShapes();
   mCollisionShapes.clear();

   // remove all rbds
   DeleteAllRigidBodies();
   mRigidBodies.clear();

	delete mDynamicsWorld;
	delete mSolver;
	delete mBroadphase;
	delete mDispatcher;
	delete mCollisionConfiguration;
}

void bulletSimulation::ClearWorld()
{
   // remove all constraints
   DeleteAllConstraints();
   mConstraints.clear();

   // remove all rbds
   DeleteAllRigidBodies();
   mRigidBodies.clear();

   XSI::Application().LogMessage(L"[MOMENTUM] All rigid bodies and constraints deleted.",XSI::siVerboseMsg);

   ///reset some internal cached data in the broadphase
   mDynamicsWorld->getBroadphase()->resetPool(mDynamicsWorld->getDispatcher());
   mDynamicsWorld->getConstraintSolver()->reset();
   mDynamicsWorld->synchronizeMotionStates();
}

void bulletSimulation::ResetWorld()
{
   mChangingFrame = true;
   mLastFrame = 0;

   XSI::Application().LogMessage(L"[MOMENTUM] Resetting world.",XSI::siVerboseMsg);

   // remove all bodies and shapes
   ClearWorld();

   // reset the gravity etc
   ResetSettings();

   mChangingFrame = false;
}

void bulletSimulation::ResetSettings()
{
   // test if the momentum settings exist
   XSI::CRef settingsRef;
   settingsRef.Set(L"Scene_Root.MomentumSettings");
   XSI::CustomProperty settingsProperty;
   if(!settingsRef.IsValid())
   {
      if(XSI::Application().GetActiveSceneRoot().IsValid())
         settingsProperty = XSI::Application().GetActiveSceneRoot().AddProperty(L"MomentumSettings",false,L"MomentumSettings");
   }
   else
      settingsProperty = XSI::CustomProperty(settingsRef);

   if(settingsRef.IsValid())
   {
      //retrieve the substeps and gravity settings
      mSubSteps = (LONG)settingsProperty.GetParameterValue(L"substeps");
      btVector3 gravity;
      gravity.setX((float)settingsProperty.GetParameterValue(L"gravity_x"));
      gravity.setY((float)settingsProperty.GetParameterValue(L"gravity_y"));
      gravity.setZ((float)settingsProperty.GetParameterValue(L"gravity_z"));
      mDynamicsWorld->setGravity(gravity);
   }

   // get the fps and update the frame
   XSI::CRef fpsParamRef;
   fpsParamRef.Set("PlayControl.Rate");
   if(fpsParamRef.IsValid())
      mFps = (float)XSI::Parameter(fpsParamRef).GetValue();
}

bool bulletSimulation::SetFrame(int in_Frame)
{
   // see if we have to reset the world
   if(in_Frame <= 0)
   {
      ResetWorld();
      return true;
   }

   // now let's skip if we already have that frame or the interval>5 - we don't want bullet to compute the whole sequence if you move to the last frame.
   if(mLastFrame == in_Frame || (in_Frame-mLastFrame)>5)
      return false;

   mChangingFrame = true;

   // if this is not the first frame, let's update all rbd!
   for(btRigidBodyIt it = mRigidBodies.begin();it!=mRigidBodies.end();it++)
   {
      // skip corrupt bodies
      if(it->second->body==NULL)
         continue;
      if(it->first.secondary != -1)
         continue;

      btCollisionShape * shape = (btBoxShape*)it->second->body->getCollisionShape();
      if(shape != NULL)
      {
         // refresh the kine ptr
         it->second->kine.Set(it->second->kine.GetAsText());

         XSI::MATH::CTransformation xf = XSI::KinematicState(it->second->kine).GetTransform();//(float)in_Frame);
         shape->setLocalScaling(btVector3(xf.GetSclX(),xf.GetSclY(),xf.GetSclZ()));

         // skip non valid ops
         if(!it->second->op.IsValid())
            continue;

         // update the mass an inertia
         it->second->mass = XSI::CustomOperator(it->second->op).GetParameterValue(L"mass");
         btVector3 inertia(0,0,0);
         if(it->second->mass > 0.0f)
            shape->calculateLocalInertia(it->second->mass,inertia);
         it->second->body->setMassProps(it->second->mass,inertia);

         // if we are passive rigid body, let's update the transform
         if(it->second->mass == 0.0f)
         {
            btDefaultMotionState * motionState = (btDefaultMotionState*)it->second->body->getMotionState();
            if(motionState != NULL)
            {
               btTransform transform;
               transform.setOrigin(btVector3(xf.GetPosX(),xf.GetPosY(),xf.GetPosZ()));

               XSI::MATH::CQuaternion q = xf.GetRotationQuaternion();
               transform.setRotation(btQuaternion(q.GetX(),q.GetY(),q.GetZ(),q.GetW()));
               it->second->body->proceedToTransform( transform );
            }
         }
      }
   }

   // synchronize if we have any passive ones
   mDynamicsWorld->synchronizeMotionStates();

   // step to the given frame
   if(in_Frame > 0)
   {
      while(in_Frame > mLastFrame)
      {
         //mDynamicsWorld->stepSimulation(1.0f/float(in_Fps),0);

         float dt=(1.0f/mFps)/(float)mSubSteps;
         //Step every subframe
         for (int i=0;i<mSubSteps;i++)
             mDynamicsWorld->stepSimulation(dt,0,1.0f/mFps);

         mLastFrame++;
      }
   }
   mLastFrame = in_Frame;

   mChangingFrame = false;
   return true;
}

int bulletSimulation::GetNbBodies(int in_Base)
{
   if(in_Base == -1)
      return (int)mRigidBodies.size();

   // if we want a specific base, count them!
   int result = 0;
   for(btRigidBodyIt it=mRigidBodies.begin();it!=mRigidBodies.end();it++)
   {
      if(it->first.primary == in_Base)
         result++;
   }
   return result;
}

bool bulletSimulation::AddRigidBody(const rbdID & in_ID, btRigidBodyReference * in_BodyRef)
{
   // don't allow for NULL
   if(in_BodyRef==NULL)
      return false;

   if(in_BodyRef->body==NULL)
      return false;

   // first check if we already have it
   btRigidBodyIt it = mRigidBodies.find(in_ID);
   if(it != mRigidBodies.end())
      return false;

   // update the rbd's damping
   in_BodyRef->body->setDamping(mLinearDamping,mAngularDamping);
   // add it to the simulation
   mDynamicsWorld->addRigidBody(in_BodyRef->body);
   mRigidBodies.insert(btRigiBodyPair(in_ID,in_BodyRef));
   return true;
}

bool bulletSimulation::DeleteRigidBody(const rbdID & in_ID)
{
   // first check if we already have it
   btRigidBodyIt it = mRigidBodies.find(in_ID);
   if(it == mRigidBodies.end())
      return false;

   // we don't have to delete also the collision shape,
   // because that's done separately anyway!

   btRigidBodyReference * bodyRef = it->second;
   if(bodyRef != NULL)
   {
      if(bodyRef->body!=NULL)
      {
         if(bodyRef->cluster == NULL)
            mDynamicsWorld->removeRigidBody(bodyRef->body);
         if(bodyRef->body->getMotionState()!=NULL)
            delete bodyRef->body->getMotionState();
         if(bodyRef->isACluster && bodyRef->body->getCollisionShape()!= NULL)
         {
            btCompoundShape * compound = (btCompoundShape *)bodyRef->body->getCollisionShape();
            delete compound;
         }
         delete(bodyRef->body);
      }
      delete(bodyRef);
      bodyRef = NULL;
   }

   // remove it from the world
   mRigidBodies.erase(it);
   return true;
}

bool bulletSimulation::DeleteAllRigidBodies(const rbdID & in_ID)
{
   // loop through all collision shapes!
   bool deleteSomething = false;
   for(btRigidBodyIt it=mRigidBodies.begin();it!=mRigidBodies.end();)
   {
      // check if the primary id matches!
      if(it->first.primary == in_ID.primary || in_ID.primary == -1)
      {
         btRigidBodyReference * bodyRef = it->second;
         if(bodyRef != NULL)
         {
            if(bodyRef->body!=NULL)
            {
               if(bodyRef->cluster == NULL)
                  mDynamicsWorld->removeRigidBody(bodyRef->body);
               if(bodyRef->body->getMotionState()!=NULL)
                  delete bodyRef->body->getMotionState();
               if(bodyRef->isACluster && bodyRef->body->getCollisionShape()!= NULL)
               {
                  btCompoundShape * compound = (btCompoundShape *)bodyRef->body->getCollisionShape();
                  delete compound;
               }
               delete(bodyRef->body);
            }
            delete(bodyRef);
            bodyRef = NULL;
         }
         mRigidBodies.erase(it++);
         deleteSomething = true;
      }
      else
         ++it;
   }
   return deleteSomething;
}

btRigidBodyReference * bulletSimulation::GetRigidBody(const rbdID & in_ID)
{
   // first check if we already have it
   btRigidBodyIt it = mRigidBodies.find(in_ID);
   if(it == mRigidBodies.end())
      return NULL;
   return it->second;
}

bool bulletSimulation::AddCollisionShape(const rbdID & in_ID, btCollisionShapeReference * in_ShapeRef)
{
   // don't allow for NULL
   if(in_ShapeRef == NULL)
      return false;
   if(in_ShapeRef->shape==NULL)
      return false;

   // first check if we already have it
   btCollisionShapeIt it = mCollisionShapes.find(in_ID);
   if(it != mCollisionShapes.end())
      return false;

   mCollisionShapes.insert(btCollisionShapePair(in_ID,in_ShapeRef));
   return true;
}

bool bulletSimulation::DeleteCollisionShape(const rbdID & in_ID)
{
   // first check if we already have it
   btCollisionShapeIt it = mCollisionShapes.find(in_ID);
   if(it == mCollisionShapes.end())
      return false;

   btCollisionShapeReference * shapeRef = it->second;
   if(shapeRef != NULL)
   {
      if(shapeRef->triPos != NULL)
         free(shapeRef->triPos);
      if(shapeRef->triId != NULL)
         free(shapeRef->triId);
      if(shapeRef->tris != NULL)
         delete shapeRef->tris;
      if(shapeRef->shape != NULL)
         delete shapeRef->shape;
      delete(shapeRef);
      shapeRef = NULL;
   }
   // remove it from the world
   mCollisionShapes.erase(it);
   return true;
}

bool bulletSimulation::DeleteAllCollisionShapes(const rbdID & in_ID)
{
   // loop through all collision shapes!
   bool deleteSomething = false;
   for(btCollisionShapeIt it=mCollisionShapes.begin();it!=mCollisionShapes.end();)
   {
      // check if the primary id matches!
      if(it->first.primary == in_ID.primary || in_ID.primary == -1)
      {
         btCollisionShapeReference * shapeRef = it->second;
         if(shapeRef != NULL)
         {
            if(shapeRef->triPos != NULL)
               free(shapeRef->triPos);
            if(shapeRef->triId != NULL)
               free(shapeRef->triId);
            if(shapeRef->tris != NULL)
               delete shapeRef->tris;
            if(shapeRef->shape != NULL)
               delete shapeRef->shape;;
            delete(shapeRef);
            shapeRef = NULL;
         }
         mCollisionShapes.erase(it++);
         deleteSomething = true;
      }
      else
         ++it;
   }
   return deleteSomething;
}

btCollisionShapeReference * bulletSimulation::GetCollisionShape(const rbdID & in_ID)
{
   // first check if we already have it
   btCollisionShapeIt it = mCollisionShapes.find(in_ID);
   if(it == mCollisionShapes.end())
      return NULL;
   return it->second;
}

bool bulletSimulation::AddConstraint(const rbdID & in_ID, btConstraintReference * in_ConstraintRef)
{
   // don't allow for NULL
   if(in_ConstraintRef == NULL)
      return false;
   if(in_ConstraintRef->constraint==NULL)
      return false;
   if(in_ConstraintRef->bodyA==NULL)
      return false;
   if(in_ConstraintRef->bodyB==NULL)
      return false;

   // first check if we already have it
   btConstraintIt it = mConstraints.find(in_ID);
   if(it != mConstraints.end())
      return false;

   // add it to the dynamics world
   mDynamicsWorld->addConstraint(in_ConstraintRef->constraint);
   mConstraints.insert(btConstraintPair(in_ID,in_ConstraintRef));
   return true;
}

bool bulletSimulation::DeleteConstraint(const rbdID & in_ID)
{
   // first check if we already have it
   btConstraintIt it = mConstraints.find(in_ID);
   if(it == mConstraints.end())
      return false;

   btConstraintReference * constraintRef= it->second;
   if(constraintRef != NULL)
   {
      if(constraintRef->constraint != NULL)
      {
         mDynamicsWorld->removeConstraint(constraintRef->constraint);
         delete constraintRef->constraint;
      }
      delete(constraintRef);
      constraintRef = NULL;
   }

   // remove it from the world
   mConstraints.erase(it);
   return true;
}

bool bulletSimulation::DeleteAllConstraints(const rbdID & in_ID)
{
   // loop through all constraints!
   bool deleteSomething = false;
   for(btConstraintIt it=mConstraints.begin();it!=mConstraints.end();)
   {
      // check if the primary id matches!
      if(it->first.primary == in_ID.primary || in_ID.primary == -1)
      {
         btConstraintReference * constraintRef= it->second;
         if(constraintRef != NULL)
         {
            if(constraintRef->constraint != NULL)
            {
               mDynamicsWorld->removeConstraint(constraintRef->constraint);
               delete constraintRef->constraint;
            }
            delete(constraintRef);
            constraintRef = NULL;
         }
         mConstraints.erase(it++);
         deleteSomething = true;
      }
      else
         ++it;
   }
   return deleteSomething;
}

btConstraintReference * bulletSimulation::GetConstraint(const rbdID & in_ID)
{
   // first check if we already have it
   btConstraintIt it = mConstraints.find(in_ID);
   if(it == mConstraints.end())
      return NULL;
   return it->second;
}

void bulletSimulation::GetContactPoints(btAlignedObjectArray<btVector3> * pos, btAlignedObjectArray<btVector3> * vel, int mode, float minImpulse)
{
   int numManifolds = mDispatcher->getNumManifolds();
   for (int i=0;i<numManifolds;i++)
   {
      btPersistentManifold* contactManifold =  mDispatcher->getManifoldByIndexInternal(i);
      btRigidBody * obA = static_cast<btRigidBody*>(contactManifold->getBody0());
      btRigidBody * obB = static_cast<btRigidBody*>(contactManifold->getBody1());

      int numContacts = contactManifold->getNumContacts();
      for (int j=0;j<numContacts;j++)
      {
         btManifoldPoint& pt = contactManifold->getContactPoint(j);
         if (pt.getDistance()<0.f)
         {
            if(pt.getAppliedImpulse() < minImpulse)
               continue;
            if(mode != 0)
            {
               pos->push_back(pt.getPositionWorldOnB());
               vel->push_back(obB->getLinearVelocity());
            }
            if(mode != 1)
            {
               pos->push_back(pt.getPositionWorldOnA());
               vel->push_back(obA->getLinearVelocity());
            }
         }
      }
   }
}

void bulletSimulation::GetBodyVelocities(btAlignedObjectArray<btVector3> * pos, btAlignedObjectArray<btVector3> * vel, int mode)
{
   btTransform xf;
   for(btRigidBodyIt it = mRigidBodies.begin(); it != mRigidBodies.end(); it++)
   {
      xf = it->second->GetWorldTransform();
      pos->push_back(xf.getOrigin());
      vel->push_back(it->second->body->getLinearVelocity());
   }
}

void btRigidBodyReference::AddToCluster(btRigidBodyReference * in_pCluster)
{
   // skip null pointers
   if(in_pCluster==NULL)
      return;

   // check if I am already in a cluster
   if(cluster != NULL)
      return;

   // check if the cluster is me!?
   if(in_pCluster == this)
      return;

   // if this is a cluster itself
   if(isACluster)
      return;

   // we need to ensure that the cluster ref is actually a cluster!
   btCompoundShape * compound = NULL;
   if(!in_pCluster->isACluster)
   {
      // this will hopefully prevent the crash...!!
      gSimulation->GetDynamicsWorld()->removeRigidBody(in_pCluster->body);
      gSimulation->GetDynamicsWorld()->addRigidBody(in_pCluster->body);

      in_pCluster->clusterOffset.setIdentity();
      compound = new btCompoundShape();
      compound->addChildShape(in_pCluster->clusterOffset,in_pCluster->body->getCollisionShape());
      compound->setMargin(0);
      in_pCluster->body->setCollisionShape(compound);
      in_pCluster->isACluster = true;
      in_pCluster->children.push_back(in_pCluster);
   }
   else
   {
      compound = (btCompoundShape *)in_pCluster->body->getCollisionShape();
   }

   // add ourselves to the children map of the cluster
   in_pCluster->children.push_back(this);

   // now calculate our offset
   btTransform clusterTransform = in_pCluster->GetWorldTransform(false);
   clusterOffset = clusterTransform.inverse() * GetWorldTransform();

   // add the shape and remember the cluster!
   compound->addChildShape(clusterOffset,body->getCollisionShape());
   cluster = in_pCluster;

   // get the local bounding box
   btVector3 minBox,maxBox,center;
   btTransform identity;
   identity.setIdentity();
   compound->getAabb(identity,minBox,maxBox);
   center = (minBox + maxBox) * 0.5f;

   // now since we have the center, substract the center from all
   // child transforms!
   for(int i=0;i<compound->getNumChildShapes();i++)
   {
      btTransform childTransform = compound->getChildTransform(i);
      childTransform.setOrigin(childTransform.getOrigin() - center);
      compound->updateChildTransform(i,childTransform);
      cluster->children[i]->clusterOffset = childTransform;
   }

   center = clusterTransform(center);
   clusterTransform.setOrigin(center);
   cluster->SetWorldTransform(clusterTransform,true);

   // now update the mass!
   cluster->mass = cluster->mass + mass;
   btVector3 inertia(0,0,0);
   if(mass > 0.0f)
      compound->calculateLocalInertia(cluster->mass,inertia);
   cluster->body->setMassProps(cluster->mass,inertia);
   cluster->body->updateInertiaTensor();

   // remove my rigid body from the world!
   gSimulation->GetDynamicsWorld()->removeRigidBody(body);
}

void btRigidBodyReference::RemoveFromCluster()
{
   // check if I am in a cluster at all
   if(cluster == NULL)
      return;

   XSI::Application().LogMessage(L"Removing 1");

   // let calculate my global offset
   btTransform clusterTransform = cluster->GetWorldTransform(false);
   btTransform worldTransform = clusterTransform * clusterOffset;

   XSI::Application().LogMessage(L"Removing 2");

   // copy the velocities from the cluster
   body->setLinearVelocity(cluster->body->getLinearVelocity());
   body->setAngularVelocity(cluster->body->getAngularVelocity());

   // remove the shape from the compound
   btCompoundShape * compound = (btCompoundShape *)cluster->body->getCollisionShape();
   compound->removeChildShape(body->getCollisionShape());

   XSI::Application().LogMessage(L"Removing 3");

   // remove ourselves from the children vector
   cluster->children.remove(this);

   XSI::Application().LogMessage(L"Removing 4");

   // get the local bounding box
   btVector3 minBox,maxBox,center;
   btTransform identity;
   identity.setIdentity();
   compound->getAabb(identity,minBox,maxBox);
   center = (minBox + maxBox) * 0.5f;

   XSI::Application().LogMessage(L"Removing 5");

   // now since we have the center, substract the center from all
   // child transforms!
   for(int i=0;i<compound->getNumChildShapes();i++)
   {
      btTransform childTransform = compound->getChildTransform(i);
      childTransform.setOrigin(childTransform.getOrigin() - center);
      compound->updateChildTransform(i,childTransform);
      cluster->children[i]->clusterOffset = childTransform;
   }

   XSI::Application().LogMessage(L"Removing 6");

   center = clusterTransform(center);
   clusterTransform.setOrigin(center);
   cluster->SetWorldTransform(clusterTransform,true);

   XSI::Application().LogMessage(L"Removing 7");

   // if there are no children left!
   if(compound->getNumChildShapes()==1)
   {
      // this will hopefully prevent the crash...!!
      gSimulation->GetDynamicsWorld()->removeRigidBody(cluster->body);
      gSimulation->GetDynamicsWorld()->addRigidBody(cluster->body);

      clusterTransform = cluster->GetWorldTransform();
      cluster->clusterOffset.setIdentity();
      cluster->isACluster = false;
      cluster->children.clear();
      cluster->body->setCollisionShape(compound->getChildShape(0));
      compound->removeChildShapeByIndex(0);
      delete compound;
      cluster->SetWorldTransform(clusterTransform,true);
   }

   XSI::Application().LogMessage(L"Removing 8");

   // now update the mass!
   cluster->mass = cluster->mass - mass;
   btVector3 inertia(0,0,0);
   if(cluster->mass > 0.0f)
      cluster->body->getCollisionShape()->calculateLocalInertia(cluster->mass,inertia);
   cluster->body->setMassProps(cluster->mass,inertia);
   cluster->body->updateInertiaTensor();

   XSI::Application().LogMessage(L"Removing 9");

   // remove the cluster from my reference
   cluster = NULL;

   // finally update the transform!
   SetWorldTransform(worldTransform,true);

   // add my rigid body to the world!
   gSimulation->GetDynamicsWorld()->addRigidBody(body);

   // now update the mass!
   inertia = btVector3(0,0,0);
   if(mass > 0.0f)
      body->getCollisionShape()->calculateLocalInertia(mass,inertia);
   body->setMassProps(mass,inertia);
   body->updateInertiaTensor();

   XSI::Application().LogMessage(L"Removing End");
}

btTransform btRigidBodyReference::GetWorldTransform(bool bRecurseToCluster)
{
   btTransform result;

   if(cluster == NULL)
   {
      // if I am a normal rigid body, just ask the motionstate!
      body->getMotionState()->getWorldTransform(result);

      // if I am a cluster, oh brother
      if(isACluster && bRecurseToCluster)
         result = result * clusterOffset;
   }
   else
   {
      // in this case we need to get the transform of the cluster
      // and multiply it with my local offset!
      result = cluster->GetWorldTransform(false) * clusterOffset;
   }

   return result;
}

void btRigidBodyReference::SetWorldTransform(const btTransform & in_XF, bool bRecreateMotionState)
{
   if(bRecreateMotionState)
   {
      delete(body->getMotionState());
      body->setMotionState(new btDefaultMotionState(in_XF));
   }
   else
   {
      if(cluster == NULL)
      {
         // if I am a normal rigid body, just update the motionstate!
         body->getMotionState()->setWorldTransform(in_XF);
      }
      else
      {
         // in this case we need to get the transform of the cluster
         // and multiply it with my local offset!
         clusterOffset = cluster->GetWorldTransform().inverse() * in_XF;
      }
   }
}
