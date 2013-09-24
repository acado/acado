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


#ifndef ACADO_TOOLKIT_MODELING_TOOLS_KINVEC_HPP
#define ACADO_TOOLKIT_MODELING_TOOLS_KINVEC_HPP


#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/symbolic_expression/expression.hpp>
#include <acado/function/function.hpp>
#include <string>
#include <acado/modeling_tools/frame.hpp>
#include <acado/modeling_tools/kinetics_tools.hpp>


BEGIN_NAMESPACE_ACADO

#ifndef __MATLAB__
using namespace std;
#endif


/**
 *  \brief Represent kinematical vectors
 *
 * What is a kinematic vector?
 *
 * A kinematic vector is a - possibly time varying - object in vectorspace that can exist irrespective of what
 * reference frames happen to be defined in this space. In other words, a vector has an identity of its own
 * and a given column matrix r=[r_x;r_y;r_z] (or vector as it is often confusingly called) is merely a image
 * of this vector from one point of view (expressed or resolved or represented in one particular frame). 
 *
 * In fact two types of vectors can be distinguished: position or 1-vectors and displacement or 0-vectors.
 * - For a 0-vector, only orientation and length matter.
 * - For a 1-vector, the anchor point of the vector is important part of the vector's definition.
 *
 * This class represents a kinematic vector by having a column matrix representation and a reference frame as members.
 *
 * Operators can work on KinVec objects with different reference frames. The result is automatically computed in a common frame.
 *
 * The really useful functions are found in the header documentation (KinVec.hpp)
 *
 *
 * \code
 *   KinVec a=rotVel(f1,f0,f0)+rotVel(f2,f1,f0);
 *   KinVec b=rotVel(f2,f0,f0); // a-c is zero (relation from mechanics)
 *   KinVec c=rotVel(f2,f0,f1); // b-c is zero
 * \endcode
 *
 *  \author Joris Gillis, Boris Houska, Hans Joachim Ferreau
 *
 */


class KinVec {


    public:

        /** Default constructor*/
        KinVec();

        /** Copy constructor*/
        KinVec( const KinVec &v );


        KinVec( const Expression& x  ,
                const Expression& y  ,
                const Expression& z  ,
                bool  type           ,
                const Frame     & ref  );


        KinVec( const Expression& xyz,
                bool  type           ,
                const Frame     & ref  );


        KinVec( const Expression& xyz,
                bool  type           ,
                const Frame&      ref,
                const Expression&   J,
                const Expression&   q,
                const Expression&  dq,
                const Expression& ddq,
                int   order            );


        KinVec( const Expression& xyz,
                bool  type           ,
                const Frame     & ref,
                const Expression&   J,
                const Expression&   c,
                const Expression&   q,
                const Expression & dq,
                const Expression &ddq,
                int order              );


        KinVec( const Expression& xyz,
                bool  type           ,
                const Frame     & ref,
                const Expression&   q,
                const Expression&  dq,
                const Expression& ddq,
                int   order            );


    /** \brief Return the components of the KinVec as 3x1 expression
    *
    * \return 3x1 Expression
    */
    Expression getCoords() const;

    /** \brief Return the components of the KinVec as 3x1 expression, expressed in a particular frame
    *
    * \return 3x1 Expression
    */
    Expression getCoords(const Frame& ref_) const;


	/** \brief Get the time dependant symbols
        * They default to the time dependant symbols of the world frame
	 */
	const Expression& getQ() const;

	/** \brief Get the derivatives of the time dependant symbols
        * They default to the time dependant symbols of the world frame
	 */
	const Expression& getDQ() const;

	/** \brief Get the second derivatives of the time dependant symbols
        * They default to the time dependant symbols of the world frame
	 */
	const Expression& getDDQ() const;
	
	returnValue setDDQ(const Expression& ddq_);

    /**
    \return 0 for velocity vectors, one for position vectors
    */
    bool getType();

    /**
    * Returns the reference frame \a ref in which the vector components are expressed
    */
    Frame getRef();

    /**
    * This operator preserves jacobian information
    */
    KinVec operator+(const KinVec &b);

    /**
    * This operator preserves jacobian information
    */
    KinVec operator-(const KinVec &b);






    /**
    * This operator preserves jacobian information
    */
    KinVec operator-() const;

    
    /**
    * This operator preserves jacobian information
    */
    KinVec& operator+=(const KinVec &b);
    
    /**
    * This operator preserves jacobian information
    */
    KinVec& operator-=(const KinVec &b);

    Stream print(Stream &stream) const;
    friend  Stream operator<<(Stream &stream, const KinVec &vec);


    /** \brief Make an new KinVec, expressed in another Frame */
    KinVec expressedIn(const Frame& f) const;

    /** \brief express the KinVec in another Frame */
    void expressIn(const Frame& f);



    /** \brief component-wise time derivative */
    KinVec der();


    /**
    \param ddq symbols for the second derivatives, to which is explicitized
    */
    Expression explicitize(const Expression & ddq) const;
    
    /**
    \param ddq symbols for the second derivatives, to which is explicitized
    */
    Expression explicitize(std::map<uint,uint> &di) const;
    
    /**
    * \param di map which contains indices of ddq to which is explicitized
    * \param ri map which contains indices of the rows of the KinVec which are taken into account
    *
    * Documentation not finished
    * 
    * Will construct a system A*x+b from KinVec
    * eg:
    * vec:
    * \code
    * | 1 0 0 0 | | ddr     |   | x |
    * | 0 1 0 0 | | ddphi   | + | y |
    * | 1 1 1 1 | | ddtheta |   | z |
    *             | dddelta |
    * \endcode
    *
    * ddqn=[0 0 0 0];
    */
    Expression explicitize(std::map<uint,uint> &di,std::map<uint,uint> &ri) const;
    
    
    /**
    \param ddq symbols for the second derivatives, to which is explicitized
    */
    Expression explicitizeJc(std::map<uint,uint> &di) const;
    
    Expression explicitizeJc(std::map<uint,uint> &di,std::map<uint,uint> &ri) const;

    /** \brief vector style access (allocation if element does not exist)
    * This operator destroys jacobian information
    */
    Operator& operator[](int i);                    

    /** \brief vector style access (never allocate) 
    */
    Expression operator[](int i) const;
    
    /** \brief vector style access (allocation if element does not exist)
    * This operator destroys jacobian information
    */
    Operator& operator()(int i);                    

    /** \brief vector style access (never allocate) 
    */
    Expression operator()(int i) const;

    /** \brief vector components expressed in Frame \a ref
    */
    Expression v    ;
    Expression q    ;
    Expression dq   ;
    Expression ddq  ;
    /** The jacobian of the vector components with respect to the state derivative that applies
    *  - for velocity, this would be with respect to q
    *  - for acceleration, this would be with respect to dq
    */
    Expression J    ;
    /** 
    A constant vector such that:
     - v=J*qq+c (for velocities)
     - v=J*ddq+c (for accelerations)
    */
    Expression c    ;
    /** \brief order of derivative.
    * 0 = position, 1 = velocity, 2 = acceleration
    * minus sign indicates rotation in stead of linear motion */
    int        order;
    /** 0 for velocity vectors, one for position vectors**/
    bool       type ;   
    /** \brief Reference frame in which the vector's components are expressed */  
    Frame      ref  ;


  protected:

    
    returnValue Initialize(const Expression &xyz, bool type_,const Frame& ref_,const Expression &J_,const Expression &c_,const Expression & q_,const Expression & dq_,const Expression & ddq_,int order_);
    returnValue copy(const KinVec &f);
    

};

CLOSE_NAMESPACE_ACADO


/** \brief Take two vectors, express them in a common frame and return this common frame.
*
* Take two vectors, express them in a common frame and return this common frame.
*
* Will change the KinVecs passed to it
*/

REFER_NAMESPACE_ACADO Frame expressCommon(REFER_NAMESPACE_ACADO KinVec &a,REFER_NAMESPACE_ACADO KinVec &b);
    
/**
* \brief  Get the 1-vector that defines the position a frame \a f expressed it in another frame \a ei
*
* Get the 1-vector that defines the position a frame \a f expressed it in another frame \a ei
*
* Example usage:
*
* \code
*   Frame f1("f1",f0,tr(x,y,z));
*   KinVec p=pos(f1,f1);        // Would be [0,0,0,1]
*   KinVec q=p.expressedIn(f0); // Would be [x,y,z,1]
*   KinVec r=pos(f1,f0);	// same as above
* \endcode
*/
REFER_NAMESPACE_ACADO KinVec pos(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& ei);
/**
* \brief  Get the linear velocity of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
*
* Get the velocity of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
* \param q		nx1 expression containg the n time-dependant symbols
* \param dq		nx1 expression containg the derivatives of the n time-dependant symbols
* \param ddq		nx1 expression containg the second derivatives of the n time-dependant variables
*
*/
REFER_NAMESPACE_ACADO KinVec vel(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& wt, const REFER_NAMESPACE_ACADO Frame& ei,const REFER_NAMESPACE_ACADO Expression & q,const REFER_NAMESPACE_ACADO Expression & dq,const REFER_NAMESPACE_ACADO Expression & ddq);     // linear velocity
/**
* \brief  Get the linear velocity of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei using default time-dependence
*
* Get the velocity of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
*
* The default time-dependant symbols are taking from the world frame
*
* Example usage:
*
* \code
*   Frame f1("f1",f0,TRz(a)*tr(0,0,H));
*   Frame f2("f2",f1,tr(L,0,0));
*   KinVec	p=vel(f2,f1,f1);      	// Would be [dL,0,0,0]
*   p=pos(f2,f1,f1).der(t); 		// Same
*
*   p=vel(f2,f0,f1);      		// Would be [dL,da*L,0,0]
*   p=vel(f2,f0,f0).ExpressedIn(f1); 	// Same
*
*   p=vel(f2,f1,f0);    		// Would be [cos(a)*dL,sin(a)*dL,0,0]
*   p=vel(f2,f1,f1).ExpressedIn(f0); 	// Same
*
*   p=vel(f2,f0,f0);     		// Would be [cos(a)*dL-sin(a)*da*L,sin(a)*dL+cos(a)*da*L,0,0]
*   p=pos(f2,f0,f0).der(t); 		// Same
* \endcode
*
* Note that - in general - velocity is not just the component-wise time derivative of positition.
* This only holds if \a ei equals \a wt
*
*/
REFER_NAMESPACE_ACADO KinVec vel(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& wt, const REFER_NAMESPACE_ACADO  Frame& ei);     						
/**
* \brief  Get the linear acceleration of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
*
* Get the acceleration of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
* \param q		nx1 expression containg the n time-dependant symbols
* \param dq		nx1 expression containg the derivatives of the n time-dependant symbols
* \param ddq		nx1 expression containg the second derivatives of the n time-dependant variables
*/
REFER_NAMESPACE_ACADO KinVec acc(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& wt, const REFER_NAMESPACE_ACADO Frame& ei,const REFER_NAMESPACE_ACADO Expression & q,const REFER_NAMESPACE_ACADO Expression & dq,const REFER_NAMESPACE_ACADO Expression & ddq);     // linear acceleration
/**
* \brief  Get the linear acceleration of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei using default time-dependence
*
* Get the linear acceleration of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
*
* The default time-dependant symbols are taking from the world frame
*
* \code
*   KinVec	p=acc(f2,f1,f1);      	
*   p=vel(f2,f1,f1).der(t); 		// Same as previous line
*
*   p=acc(f2,f0,f1);      		
*   p=acc(f2,f0,f0).ExpressedIn(f1); 	// Same as previous line
*
*   p=acc(f2,f1,f0);    		 
*   p=acc(f2,f1,f1).ExpressedIn(f0); 	// Same as previous line
*
*   p=acc(f2,f0,f0);     		
*   p=vel(f2,f0,f0).der(t); 		// Same as previous line
* \endcode
*
* Note that - in general - acceleration is not just the component-wise time derivative of velocity.
* This only holds if \a ei equals \a wt
*/
REFER_NAMESPACE_ACADO KinVec acc(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& wt, const REFER_NAMESPACE_ACADO Frame& ei);
/**
* \brief  Get the rotational velocity of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
*
* Get the rotational velocity of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
* \param q		nx1 expression containg the n time-dependant symbols
* \param dq		nx1 expression containg the derivatives of the n time-dependant symbols
* \param ddq		nx1 expression containg the second derivatives of the n time-dependant variables
*/
REFER_NAMESPACE_ACADO KinVec rotVel(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& wt,const  REFER_NAMESPACE_ACADO Frame& ei,const REFER_NAMESPACE_ACADO Expression & q,const REFER_NAMESPACE_ACADO Expression & dq,const REFER_NAMESPACE_ACADO Expression & ddq);   // rotational velocity
/**
* \brief  Get the rotational velocity of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei using default time-dependence
*
* Get the rotational of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
*
* The default time-dependant symbols are taking from the world frame.
*
* The rotational velocity is defined by its skew-symmetric matrix form:
*    W_10 = dot(R_10)*R_01
* 
*    with R_10 the 3x3 rotation matrix that transforms from frame 0 to 1.
*/
REFER_NAMESPACE_ACADO KinVec rotVel(const REFER_NAMESPACE_ACADO  Frame& f,const REFER_NAMESPACE_ACADO Frame& wt,const  REFER_NAMESPACE_ACADO Frame& ei);  
/**
* \brief  Get the rotational acceleration of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
*
* Get the rotational acceleration of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
* \param q		nx1 expression containg the n time-dependant symbols
* \param dq		nx1 expression containg the derivatives of the n time-dependant symbols
* \param ddq		nx1 expression containg the second derivatives of the n time-dependant variables
*/
REFER_NAMESPACE_ACADO KinVec rotAcc(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& wt,const  REFER_NAMESPACE_ACADO Frame& ei,const REFER_NAMESPACE_ACADO Expression & q,const REFER_NAMESPACE_ACADO Expression & dq,const REFER_NAMESPACE_ACADO Expression & ddq);	// rotational acceleration
/**
* \brief  Get the rotational acceleration of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei using default time-dependence
*
* Get the rotational acceleration of the origin of frame \a f with respect to frame \a wt, expressed in frame \a ei
*
* The default time-dependant symbols are taking from the world frame
*
* Note that - contrary to the case with linear acceleration - rotational acceleration IS just the component-wise time derivative of rotational velocity.
*/
REFER_NAMESPACE_ACADO KinVec rotAcc(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO  Frame& wt,const  REFER_NAMESPACE_ACADO Frame& ei);


/**
* \brief  Get the 0-vector that defines the x-axis of a frame \a f
*
* Get the 0-vector that defines the x-axis of a frame \a f
*/
REFER_NAMESPACE_ACADO KinVec ex(const REFER_NAMESPACE_ACADO Frame& f);
/**
* \brief  Get the 0-vector that defines the y-axis of a frame \a f
*
* Get the 0-vector that defines the x-axis of a frame \a f
*/
REFER_NAMESPACE_ACADO KinVec ey(const REFER_NAMESPACE_ACADO Frame& f);
/**
* \brief  Get the 0-vector that defines the z-axis of a frame \a f
*
* Get the 0-vector that defines the x-axis of a frame \a f
*/
REFER_NAMESPACE_ACADO KinVec ez(const REFER_NAMESPACE_ACADO Frame& f);
/**
* \brief  Get the 0-vector that defines the x-axis of a frame \a f, but express it in another frame \a ei
*
* Get the 0-vector that defines the x-axis of a frame \a f, but express it in another frame \a ei
* \code
* KinVec e0=ex(f1).expressedIn(ei);
* KinVec e1=ex(f1,ei); // same as e0
* \endcode
*/
REFER_NAMESPACE_ACADO KinVec ex(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& ei);
/**
* \brief  Get the 0-vector that defines the y-axis of a frame \a f, but express it in another frame \a ei
*
* Get the 0-vector that defines the y-axis of a frame \a f, but express it in another frame \a ei
* \code
* KinVec e0=ey(f1).expressedIn(ei);
* KinVec e1=ey(f1,ei); // same as e0
* \endcode
*/
REFER_NAMESPACE_ACADO KinVec ey(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& ei);
/**
* \brief  Get the 0-vector that defines the z-axis of a frame \a f, but express it in another frame \a ei
*
* Get the 0-vector that defines the z-axis of a frame \a f, but express it in another frame \a ei
* \code
* KinVec e0=ez(f1).expressedIn(ei);
* KinVec e1=ez(f1,ei); // same as e0
* \endcode
*/
REFER_NAMESPACE_ACADO KinVec ez(const REFER_NAMESPACE_ACADO Frame& f,const REFER_NAMESPACE_ACADO Frame& ei);


/** \brief Take the cross product of two vectors
*
* This operator destroys jacobian information
 */

REFER_NAMESPACE_ACADO KinVec cross(const REFER_NAMESPACE_ACADO KinVec& a,const REFER_NAMESPACE_ACADO KinVec& b);

/** \brief right multiply all vector components with a scalar
*
* This operator preserves jacobian information
 */
REFER_NAMESPACE_ACADO KinVec operator*(const REFER_NAMESPACE_ACADO KinVec &a,const REFER_NAMESPACE_ACADO Expression &b);
/** \brief left multiply all vector components with a scalar 
*
* This operator preserves jacobian information
*/
REFER_NAMESPACE_ACADO  KinVec operator*(const REFER_NAMESPACE_ACADO  Expression &a,const REFER_NAMESPACE_ACADO KinVec &b);

REFER_NAMESPACE_ACADO  Expression operator*(const REFER_NAMESPACE_ACADO  KinVec &a,const REFER_NAMESPACE_ACADO  KinVec &b);
    
    
/** \brief divide all vector components by a scalar
*
* This operator preserves jacobian information
 */
REFER_NAMESPACE_ACADO KinVec operator/(const REFER_NAMESPACE_ACADO KinVec &a,const REFER_NAMESPACE_ACADO Expression &b);

/** \brief take 2-norm of a KinVec
*/
REFER_NAMESPACE_ACADO Expression norm(const REFER_NAMESPACE_ACADO KinVec &a);

#endif //ACADO_TOOLKIT_MODELING_TOOLS_KINVEC_HPP

