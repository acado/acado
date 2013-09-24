/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2013 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *    \author Joris Gillis, Boris Houska, Joachim Ferreau
 *
 */


#include <acado/modeling_tools/frame.hpp>
#include <acado/modeling_tools/frame_node.hpp>

//#include <acado/symbolic_core/expression_tools.hpp>

#ifndef __MATLAB__
using namespace std;
#endif

USING_NAMESPACE_ACADO


Frame::Frame(){
    node = 0;
    // Why not:
    //node = new FrameNode(name);
    //node->count++;
}


Frame::Frame( const String     &name,
              const Expression &q   ,
              const Expression &dq  ,
              const Expression &ddq){

    node = new FrameNode( name, q, dq, ddq);
    node->count++;
}


Frame::Frame( const String     & name,
              const Frame      & ref ,
              const Expression & T     ){

    node = new FrameNode(name,ref,T);
    node->count++;
}


Frame::Frame(Frame const& frame){

  node = frame.node;
  node->count++;
}


Frame::Frame(FrameNode* ptr){

   node = ptr;
   node->count++;
}

Frame::~Frame(){
    if(node != 0){
      node->count--;
      if(node->count==0) delete node;
    }
}

Frame& Frame::operator=(const Frame &frame){
  // quick return if the old and new pointers point to the same object
  if(node == frame.node) return *this;

if(node){
  // decrease the object count for the old pointer
  node->count--;

  // delete if this was the last pointer	
  if(node->count == 0) delete node;
}

  // save the new pointer
  node = frame.node;
  if(node) node->count++;
  return *this;
}

Stream operator<<( Stream &stream, const Frame &frame){
   return frame.print(stream);
}


Stream Frame::print(Stream &stream) const{
    if(node) return node->print(stream);
    return "Empty Frame";
}


const String& Frame::getName() const{
  return node->name;
}

const Expression& Frame::getQ() const{
  return node->q;
}

const Expression& Frame::getDQ() const{
  return node->dq;
}

const Expression& Frame::getDDQ() const{
  return node->ddq;
}

// ------ Functions that deal with chaining

std::set<FrameNode*> Frame::getReferences() const {
  std::set<FrameNode*> refs;
  for(FrameNode* ptr = node; ptr != 0; ptr = ptr->ref.node){
      refs.insert(ptr);
  }
  return refs;
}

FrameNode* Frame::getCommonFramePtr(Frame other) const {
  std::set<FrameNode*> refs=getReferences();
  for(FrameNode* ptr = other.node; ptr != 0; ptr = ptr->ref.node){
    if(refs.count(ptr) > 0)
      return ptr;
  }
  throw "Could not find common frame";
}

Frame Frame::getCommonFrame(Frame other) const {
  return Frame(getCommonFramePtr(other));
}


std::vector<FrameNode*> Frame::getFrameChain(FrameNode* endptr) const {
  std::vector<FrameNode*> myChain;
  for(FrameNode* ptr = node; ptr != endptr; ptr = ptr->ref.node)
    myChain.push_back(ptr);
  return myChain;
}

// ---------

// The actual kinematics algorithms go below:

// Consider the tree  
//
//        ___0___
//       1       2
//     3   4       5

// (Frame 3).chain(e,(Frame 5))
//
// will return  inv(T_52)*inv(T_20)*T_10*T_31*e
// the third argument discriminates between position and velocity vector
// e can be a matrix as well

Expression Frame::chain(Expression e_,const Frame &ei,bool type) const {
    if (e_.getDim()==1 || e_.getDim()==0) return e_; // empty expression
    //printf("Chain Expression %d x %d",e_.getNumRows(),e_.getNumCols());
    FrameNode* common=getCommonFramePtr(ei);

    std::vector<FrameNode*> thisChain=     getFrameChain(common);
    std::vector<FrameNode*> eiChain  =  ei.getFrameChain(common);

    Expression e(e_);
    for (std::vector<FrameNode*>::iterator it=thisChain.begin() ; it != thisChain.end(); ++it ){

        e=(*it)->R * e; // Going up
        if (type) e=e+(*it)->p;
    }

    for (std::vector<FrameNode*>::reverse_iterator it=eiChain.rbegin() ; it != eiChain.rend(); ++it ){

        e=((*it)->R).transpose()*e; // Going down
        if (type) e=e-((((*it)->R).transpose() * (*it)->p));
    }
    return e;
}

// -----


