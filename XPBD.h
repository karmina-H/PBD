#pragma once

#include "Common/Common.h"

// ------------------------------------------------------------------------------------
namespace PBD
{
	class XPBD
	{
	public:
		// -------------- Distance constraint -----------------------------------------------------
		// 이웃한 두 삼각형의 접힘을 제어하는 이각형 각도 제약을 해결하는 함수.
		// 즉 한변이 인접하는 두 삼각형 사이의 각도를 제어하는 함수
		// p2,p3가 인접한 한변임 그리고 각각의 삼각형은 (p0, p2, p3) and (p1, p3, p2)
		static bool solve_DistanceConstraint(
			const Vector3r &p0, Real invMass0,
			const Vector3r &p1, Real invMass1,
			const Real restLength,
			const Real stiffness,
			const Real dt, // 타입스텝크기
			Real& lambda, // 라그랑주 승수
			Vector3r &corr0, Vector3r &corr1); 

		//물체의 부피보존제약을 수행하는 함수 즉 하나의 사면체(삼각뿔)의 부피가 원래의 안정적인부피를 최대한 유지하게 하는제약
		static bool solve_VolumeConstraint(
			const Vector3r& p0, Real invMass0,
			const Vector3r& p1, Real invMass1,
			const Vector3r& p2, Real invMass2,
			const Vector3r& p3, Real invMass3,
			const Real restVolume, //부피제약값
			const Real stiffness, //강성계수
			const Real dt, //타입스템
			Real& lambda, //라그랑주 승수
			Vector3r& corr0, Vector3r& corr1, Vector3r& corr2, Vector3r& corr3); //보정된 위치값


		// -------------- Isometric bending -----------------------------------------------------
		 // 등방성 굽힘 제약 조건(Isometric Bending Constraint)에 필요한 로컬강성행렬Q를 초기화하는 함수
		// (p2, p3)이 인접한 변, (p0, p2, p3) and (p1, p3, p2)이 두 삼각형
		// 4점을 사용해서 local 강성행렬(local stiffness matrix)를 계산해서 반환함
		static bool init_IsometricBendingConstraint(
			const Vector3r& p0,
			const Vector3r& p1,
			const Vector3r& p2,
			const Vector3r& p3,
			Matrix4r& Q
		);

		// 등방성 굽힘 제약조건을 해결하는 함수 즉 거의 늘어나지않는 표면 모델의 물체의 굽힘동작 시뮬레이션함
		// (p2, p3)이 인접한 변, (p0, p2, p3) and (p1, p3, p2)이 두 삼각형
		static bool solve_IsometricBendingConstraint(
			const Vector3r& p0, Real invMass0,		// angle on (p2, p3) between triangles (p0, p2, p3) and (p1, p3, p2)
			const Vector3r& p1, Real invMass1,
			const Vector3r& p2, Real invMass2,
			const Vector3r& p3, Real invMass3,
			const Matrix4r& Q, // LOCAL 강성계수로, 각 입자에 대한 굽힘 저항력을 나타냄 = 굽힘 특성을 결정하는 요소
			const Real stiffness, // 굽힘 강성계수
			const Real dt,
			Real& lambda,
			Vector3r& corr0, Vector3r& corr1, Vector3r& corr2, Vector3r& corr3);

		//사면체 형태의 요소에 적용되는 제약조건을 처리하는 함수
		static bool solve_FEMTetraConstraint(
			const Vector3r& p0, Real invMass0,
			const Vector3r& p1, Real invMass1,
			const Vector3r& p2, Real invMass2,
			const Vector3r& p3, Real invMass3,
			const Real restVolume, //사면체의 변형되기 전 정지 상태의 부피
			const Matrix3r& invRestMat, // 사면체의 정지 상태의 변환행렬의 역행렬
			const Real youngsModulus, // 재료물성치(재료의 강성을 나타내는 물리량으로 값이 높을수록 단단함)
			const Real poissonRatio, // 푸아송 비율(재료가 한 방향으로 늘어날 때 다른 방향으로 얼마나 수축하는지 나타내는 물리량)
			const bool  handleInversion, //사면체의 반전(뒤집히거나 안으로꺽이는)상태 처리 여부
			const Real dt,
			Real& lambda,
			Vector3r& corr0, Vector3r& corr1, Vector3r& corr2, Vector3r& corr3);
	};
}

