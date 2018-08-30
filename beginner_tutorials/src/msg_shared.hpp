/*
 * jnt_ref_mutex.hpp
 *
 *  Created on: May 12, 2016
 *      Author: l_vis
 */

#ifndef BEGINNER_TUTORIALS_SRC_MSG_SHARED_HPP_
#define BEGINNER_TUTORIALS_SRC_MSG_SHARED_HPP_

#include <boost/interprocess/sync/interprocess_mutex.hpp>

struct jnt_ref_shared
{
   //Mutex to protect access to the queue
   boost::interprocess::interprocess_mutex mutex;

   float q;
   float dq;
   float pwmComp;

   bool newRef;
};

struct jnt_state_shared
{
   //Mutex to protect access to the queue
   boost::interprocess::interprocess_mutex mutex;

   float q;
   float dq;
   float ua;

   bool newState;
};

struct force_shared
{
   //Mutex to protect access to the queue
   boost::interprocess::interprocess_mutex mutex;

   float fX;
   float fY;
   float fZ;

   bool newSample;
};

#endif /* BEGINNER_TUTORIALS_SRC_MSG_SHARED_HPP_ */
