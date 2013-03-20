#pragma once

namespace osf
{
	class Joint;
	class Fitting;

	/************************************************************************
	  Skeleton
	*************************************************************************/
	class Skeleton
	{
	public:
		virtual ~Skeleton();

		Joint* getRoot();
		void init();
		virtual Joint* iInit() = 0;
		
		void updateFK();
		void updateFK(Joint* joint);

	protected:
		Skeleton(Fitting*);

		Fitting* m_skeletonFitting;

	private:
		Joint*	m_rootJoint;
	};
}