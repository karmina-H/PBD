#ifndef POSITION_BASED_DYNAMICS_H
#define POSITION_BASED_DYNAMICS_H

#include "Common/Common.h"

// ------------------------------------------------------------------------------------
namespace PBD
{
	class PositionBasedDynamics
	{
	public:

		// -------------- standard PBD -----------------------------------------------------
		//두 입자 사이 거리제약 해결하고 위치보정값 계산하는 함수
		static bool solve_DistanceConstraint(
			const Vector3r &p0, Real invMass0, //첫번째 입자
			const Vector3r &p1, Real invMass1, //두번째 입자
			const Real restLength, //거리제약길이
			const Real stiffness, //강성계수로 1에 가까울수록 제약조건을 더 강하게 만족시키게끔하여 딱딱한 물체로표현
			Vector3r &corr0, Vector3r &corr1); //각각 보정된 위치값
		
		// 이웃한 두 삼각형의 접힘을 제어하는 이각형 각도 제약을 해결하는 함수.
		// 즉 한변이 인접하는 두 삼각형 사이의 각도를 제어하는 함수
		// p2,p3가 인접한 한변임 그리고 각각의 삼각형은 (p0, p2, p3) and (p1, p3, p2)
		static bool solve_DihedralConstraint(
			const Vector3r &p0, Real invMass0,		
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Vector3r &p3, Real invMass3,
			const Real restAngle, //각도제약값
			const Real stiffness, //강성계수로 1에 가까울수록 제약조건을 더 강하게 만족시키게끔하여 덜?접히게(종이와 다르게 잘 안접히게)
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2, Vector3r &corr3);

		//물체의 부피보존제약을 수행하는 함수 즉 하나의 사면체(삼각뿔)의 부피가 원래의 안정적인부피를 최대한 유지하게 하는제약
		static bool solve_VolumeConstraint(
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Vector3r &p3, Real invMass3,
			const Real restVolume, //부피제약값
			const Real stiffness, //강성계수로 1에 가까울수록 제약조건을 더 강하게 만족시키게끔하여 압축이 안되게끔 하는계수
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2, Vector3r &corr3);
		
		// 점과 엣지 사이의 거리를 유지하는 제약조건을 해결하는 함수
		static bool solve_EdgePointDistanceConstraint(
			const Vector3r &p, Real invMass, //점 
			const Vector3r &p0, Real invMass0, //엣지1
			const Vector3r &p1, Real invMass1, //엣지2
			const Real restDist, // 되돌아 가고자 하는 길이
			const Real compressionStiffness, // 압축 강성계수
			const Real stretchStiffness, //늘어남 강성계수
			Vector3r &corr, Vector3r &corr0, Vector3r &corr1);

		// 점과 삼각형 사이의 거리를 유지하는 제약조건을 해결하는 함수
		static bool solve_TrianglePointDistanceConstraint(
			const Vector3r &p, Real invMass,
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Real restDist,
			const Real compressionStiffness,
			const Real stretchStiffness,
			Vector3r &corr, Vector3r &corr0, Vector3r &corr1, Vector3r &corr2);
		
		// 두 선분(엣지) 사이의 거리를 유지하는 제약조건을 해결하는 함수
		static bool solve_EdgeEdgeDistanceConstraint(
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Vector3r &p3, Real invMass3,
			const Real restDist,
			const Real compressionStiffness,
			const Real stretchStiffness,
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2, Vector3r &corr3);

		// -------------- Isometric bending -----------------------------------------------------

		/** Initialize the local stiffness matrix Q. The matrix is 
		 * required by the solver step. It must only be recomputed 
		 * if the rest shape changes. \n\n
		 * Bending is simulated for the angle on (p2, p3) between 
		 * the triangles (p0, p2, p3) and (p1, p3, p2).
		 *
		 * @param  p0 point 0 of stencil
		 * @param  p1 point 1 of stencil
		 * @param  p2 point 2 of stencil
		 * @param  p3 point 3 of stencil 
		 * @param  Q returns the local stiffness matrix which is required by the solver
		 */	

		// 등방성 굽힘 제약 조건(Isometric Bending Constraint)에 필요한 로컬강성행렬Q를 초기화하는 함수
		// (p2, p3)이 인접한 변, (p0, p2, p3) and (p1, p3, p2)이 두 삼각형
		// 4점을 사용해서 local 강성행렬(local stiffness matrix)를 계산해서 반환함
		static bool init_IsometricBendingConstraint(		
			const Vector3r &p0,
			const Vector3r &p1,
			const Vector3r &p2,
			const Vector3r &p3,
			Matrix4r &Q
			);
		
		// 등방성 굽힘 제약조건을 해결하는 함수 즉 거의 늘어나지않는 표면 모델의 물체의 굽힘동작 시뮬레이션함
		// (p2, p3)이 인접한 변, (p0, p2, p3) and (p1, p3, p2)이 두 삼각형
		static bool solve_IsometricBendingConstraint(
			const Vector3r &p0, Real invMass0,		
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Vector3r &p3, Real invMass3,
			const Matrix4r &Q, // LOCAL 강성계수로, 각 입자에 대한 굽힘 저항력을 나타냄 = 굽힘 특성을 결정하는 요소
			const Real stiffness, // 굽힘 강성계수
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2, Vector3r &corr3);

		// -------------- Shape Matching  -----------------------------------------------------

		/** Initialize rest configuration infos for one shape matching cluster 
		 * which are required by the solver step. It must only be reinitialized
		 * if the rest shape changes. 
		 *
		 * @param  x0 rest configuration of all particles in the cluster
		 * @param  invMasses inverse masses of all particles in the cluster
		 * @param  numPoints number of particles in the cluster
		 * @param  restCm returns the center of mass of the rest configuration
		 */	
		static bool init_ShapeMatchingConstraint(
			const Vector3r x0[], const Real invMasses[], const int numPoints,
			Vector3r &restCm);

		/** Determine the position corrections for a shape matching constraint.\n\n
		 * More information can be found in: \cite BMM2015, \cite BMOT2013, \cite BMOTM2014, 
		 * \cite Muller2005, \cite Bender2013, \cite Diziol2011
		 *
		 * @param  x0			rest configuration of all particles in the cluster
		 * @param  x			current configuration of all particles in the cluster
		 * @param  invMasses	invMasses inverse masses of all particles in the cluster
		 * @param  numPoints	number of particles in the cluster
		 * @param  restCm		center of mass of the rest configuration
		 * @param  stiffness	stiffness coefficient 
		 * @param  allowStretch allow stretching
		 * @param  corr			position corrections for all particles in the cluster
		 * @param  rot			returns determined rotation matrix 
		 */	
		static bool solve_ShapeMatchingConstraint(
			const Vector3r x0[], const Vector3r x[], const Real invMasses[], const int numPoints,
			const Vector3r &restCm, 
			const Real stiffness,
			const bool allowStretch,		// default false
			Vector3r corr[], Matrix3r *rot = NULL);


		// -------------- Strain Based Dynamics  -----------------------------------------------------

		/** Initialize rest configuration infos which are required by the solver step.
		 * Recomputation is only necessary when rest shape changes.\n\n
		 * The triangle is defined in the xy plane.
		 *
		 * @param  p0 point 0 of triangle
		 * @param  p1 point 1 of triangle
		 * @param  p2 point 2 of triangle
		 * @param  invRestMat returns a matrix required by the solver
		 */		
		static bool init_StrainTriangleConstraint(		
			const Vector3r &p0,
			const Vector3r &p1,
			const Vector3r &p2,
			Matrix2r &invRestMat
			);

		/** Solve triangle constraint with strain based dynamics and return position corrections.\n\n
		 * More information can be found in: \cite BMM2015, \cite Mueller2014
		 *
		 * @param p0 position of first particle
		 * @param invMass0 inverse mass of first particle
		 * @param p1 position of second particle
		 * @param invMass1 inverse mass of second particle
		 * @param p2 position of third particle
		 * @param invMass2 inverse mass of third particle
		 * @param  invRestMat precomputed matrix determined by init_StrainTriangleConstraint()
		 * @param  xxStiffness stiffness coefficient for xx stretching
		 * @param  yyStiffness stiffness coefficient for yy stretching
		 * @param  xyStiffness stiffness coefficient for xy shearing
		 * @param  normalizeStretch	should stretching be normalized
		 * @param  normalizeShear should shearing be normalized
		 * @param  corr0 position correction for point 0
		 * @param  corr1 position correction for point 1
		 * @param  corr2 position correction for point 2
		 */		
		static bool solve_StrainTriangleConstraint(
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Matrix2r &invRestMat,
			const Real xxStiffness, 
			const Real yyStiffness, 
			const Real xyStiffness,
			const bool normalizeStretch,	// use false as default
			const bool normalizeShear,		// use false as default
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2);

		/** Initialize rest configuration infos which are required by the solver step.
		 * Recomputation is only necessary when rest shape changes.
		 *
		 * @param  p0 point 0 of tet
		 * @param  p1 point 1 of tet
		 * @param  p2 point 2 of tet
		 * @param  p3 point 3 of tet
		 * @param  invRestMat returns a matrix required by the solver
		 */
		static bool init_StrainTetraConstraint(	
			const Vector3r &p0,
			const Vector3r &p1,
			const Vector3r &p2,
			const Vector3r &p3,
			Matrix3r &invRestMat
			);


		// has no inversion handling. Possible simple solution: if the volume is negative, 
		// scale corrections down and use the volume constraint to fix the volume sign
		static bool solve_StrainTetraConstraint(
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Vector3r &p3, Real invMass3,
			const Matrix3r &invRestMat,
			const Vector3r &stretchStiffness,			// xx, yy, zz
			const Vector3r &shearStiffness,			// xy, xz, yz
			const bool normalizeStretch,		// use false as default
			const bool normalizeShear,		// use false as default
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2, Vector3r &corr3);


		// -------------- FEM Based PBD  -----------------------------------------------------
		static void computeGradCGreen(
			Real restVolume, 
			const Matrix3r &invRestMat, 
			const Matrix3r &sigma, 
			Vector3r *J);

		static void computeGreenStrainAndPiolaStress(
			const Vector3r &x1, const Vector3r &x2, const Vector3r &x3, const Vector3r &x4,
			const Matrix3r &invRestMat,
			const Real restVolume,
			const Real mu, const Real lambda,
			Matrix3r &epsilon, Matrix3r &sigma, Real &energy);

		static void computeGreenStrainAndPiolaStressInversion(
			const Vector3r &x1, const Vector3r &x2, const Vector3r &x3, const Vector3r &x4,
			const Matrix3r &invRestMat,
			const Real restVolume,
			const Real mu, const Real lambda,
			Matrix3r &epsilon, Matrix3r &sigma, Real &energy);


	public:
		/** Implementation of the finite element method described in \n\n
		* Jan Bender, Dan Koschier, Patrick Charrier and Daniel Weber, \n
		* "Position-Based Simulation of Continuous Materials", \n
		* Computers & Graphics 44, 2014\n\n
		* Initialize rest configuration infos which are required by the solver step.
		* Recomputation is only necessary when rest shape changes.
		*/
		static bool init_FEMTriangleConstraint(		
			const Vector3r &p0,
			const Vector3r &p1,
			const Vector3r &p2,
			Real &area,
			Matrix2r &invRestMat
			);

		/** Implementation of the finite element method described in \n\n
		* Jan Bender, Dan Koschier, Patrick Charrier and Daniel Weber, \n
		* "Position-Based Simulation of Continuous Materials", \n
		* Computers & Graphics 44, 2014\n\n
		* Solve the continuum mechanical constraint defined for a triangle.
		*/
		static bool solve_FEMTriangleConstraint(
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Real &area,
			const Matrix2r &invRestMat,
			const Real youngsModulusX,
			const Real youngsModulusY,
			const Real youngsModulusShear,
			const Real poissonRatioXY,
			const Real poissonRatioYX,
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2);

		/** Implementation of the finite element method described in \n\n
		* Jan Bender, Dan Koschier, Patrick Charrier and Daniel Weber, \n
		* "Position-Based Simulation of Continuous Materials", \n
		* Computers & Graphics 44, 2014\n\n
		* Initialize rest configuration infos which are required by the solver step.
		* Recomputation is only necessary when rest shape changes.
		*/
		static bool init_FEMTetraConstraint(			
			const Vector3r &p0,
			const Vector3r &p1,
			const Vector3r &p2,
			const Vector3r &p3,
			Real &volume,
			Matrix3r &invRestMat
			);

		/** Implementation of the finite element method described in \n\n
		* Jan Bender, Dan Koschier, Patrick Charrier and Daniel Weber, \n
		* "Position-Based Simulation of Continuous Materials", \n
		* Computers & Graphics 44, 2014\n\n
		* Solve the continuum mechanical constraint defined for a tetrahedron.
		*/
		static bool solve_FEMTetraConstraint(
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Vector3r &p2, Real invMass2,
			const Vector3r &p3, Real invMass3,
			const Real restVolume,
			const Matrix3r &invRestMat,
			const Real youngsModulus,
			const Real poissonRatio,
			const bool  handleInversion,
			Vector3r &corr0, Vector3r &corr1, Vector3r &corr2, Vector3r &corr3);


		/** Initialize contact between a particle and a tetrahedron and return
		* info which is required by the solver step.
		*
		* @param invMass0 inverse mass of particle which collides with tet
		* @param x0 particle position
		* @param v0 particle velocity 
		* @param invMass inverse masses of tet particles
		* @param x positions of tet particles
		* @param v velocities of tet particles
		* @param bary barycentric coordinates of contact point in tet
		* @param normal contact normal in body 1
		* @param constraintInfo Stores the local and global position of the contact points and
		* the contact normal. \n
		* The joint info contains the following columns:\n
		* 0:	contact normal in body 1 (global)\n
		* 1:	contact tangent (global)\n
		* 0,2:   1.0 / normal^T * K * normal\n
		* 1,2:  maximal impulse in tangent direction\n
		*/
		static bool init_ParticleTetContactConstraint(
			const Real invMass0,							// inverse mass is zero if particle is static
			const Vector3r &x0,								// particle which collides with tet
			const Vector3r &v0,								// velocity of particle
			const Real invMass[],							// inverse masses of tet particles
			const Vector3r x[],								// positions of tet particles
			const Vector3r v[],								// velocities of tet particles
			const Vector3r &bary,							// barycentric coordinates of contact point in tet
			const Vector3r &normal,							// contact normal in body 1
			Eigen::Matrix<Real, 3, 3, Eigen::DontAlign> &constraintInfo);


		/** Perform a solver step for a contact constraint between a particle and a tetrahedron.
		* A contact constraint handles collisions and resting contacts between the bodies.
		* The contact info must be generated in each time step.
		*
		* @param invMass0 inverse mass of particle which collides with tet
		* @param x0 particle position
		* @param invMass inverse masses of tet particles
		* @param x positions of tet particles
		* @param bary barycentric coordinates of contact point in tet
		* @param constraintInfo information which is required by the solver. This
		* information must be generated in the beginning by calling init_RigidBodyContactConstraint().
		* @param corr0 position correction of particle
		* @param corr position corrections of tet particles
		*/
		static bool solve_ParticleTetContactConstraint(
			const Real invMass0,							// inverse mass is zero if particle is static
			const Vector3r &x0,								// particle which collides with tet
			const Real invMass[],							// inverse masses of tet particles
			const Vector3r x[],								// positions of tet particles
			const Vector3r &bary,							// barycentric coordinates of contact point in tet
			Eigen::Matrix<Real, 3, 3, Eigen::DontAlign> &constraintInfo,		// precomputed contact info
			Real &lambda,
			Vector3r &corr0,
			Vector3r corr[]);

		/** Perform a solver step for a contact constraint between a particle and a tetrahedron.
		* A contact constraint handles collisions and resting contacts between the bodies.
		* The contact info must be generated in each time step.
		*
		* @param invMass0 inverse mass of particle which collides with tet
		* @param x0 particle position
		* @param v0 particle velocity
		* @param invMass inverse masses of tet particles
		* @param x positions of tet particles
		* @param v velocities of tet particles
		* @param bary barycentric coordinates of contact point in tet
		* @param frictionCoeff friction coefficient
		* @param constraintInfo information which is required by the solver. This
		* information must be generated in the beginning by calling init_RigidBodyContactConstraint().
		* @param corr_v0 velocity correction of particle
		* @param corr_v velocity corrections of tet particles
		*/
		static bool velocitySolve_ParticleTetContactConstraint(
			const Real invMass0,							// inverse mass is zero if particle is static
			const Vector3r &x0,								// particle which collides with tet
			const Vector3r &v0,								// velocity of particle
			const Real invMass[],							// inverse masses of tet particles
			const Vector3r x[],								// positions of tet particles
			const Vector3r v[],								// velocities of tet particles
			const Vector3r &bary,							// barycentric coordinates of contact point in tet
			const Real lambda,
			const Real frictionCoeff,						// friction coefficient
			Eigen::Matrix<Real, 3, 3, Eigen::DontAlign> &constraintInfo,		// precomputed contact info
			Vector3r &corr_v0,
			Vector3r corr_v[]);
	};
}

#endif
