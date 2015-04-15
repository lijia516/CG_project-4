#ifndef INCLUDED_CATMULL_CURVE_EVALUATOR_H
#define INCLUDED_CATMULL_CURVE_EVALUATOR_H

#pragma warning(disable : 4786)  

#include "curveevaluator.h"

//using namespace std;

class CatmullCurveEvaluator : public CurveEvaluator
{
public:
	void evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
		std::vector<Point>& ptvEvaluatedCurvePts, 
		const float& fAniLength, 
		const bool& bWrap) const;
    static Mat4<float> m_basisMatrix;
  void evaluate(const int p1, const int p2, const int p3, const int p4, const float& fAniLength,
          const std::vector<Point>& ptvCtrlPts, std::vector<Point>& ptvEvaluatedCurvePts) const;
};

#endif
