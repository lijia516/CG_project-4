#include "catmullcurveevaluator.h"
#include <assert.h>
#include "math.h"

Mat4<float> CatmullCurveEvaluator::m_basisMatrix = Mat4<float>(-1, 3, -3, 1, 2, -5, 4, -1, -1, 0, 1, 0, 0, 2, 0, 0);

void CatmullCurveEvaluator::evaluate(const int p1, const int p2, const int p3, const int p4, const float& fAniLength,
    const std::vector<Point>& ptvCtrlPts, std::vector<Point>& ptvEvaluatedCurvePts)const {
  float t, y, x;
  Vec4<float> vec = m_basisMatrix * Vec4<float>( 
      ptvCtrlPts[p1].y, 
      ptvCtrlPts[p2].y, 
      ptvCtrlPts[p3].y, 
      ptvCtrlPts[p4].y
      );
  for (int j = 0; j < 100; j++) {
    t = j / 100.0f;
    y = Vec4<float>(t * t * t, t * t, t, 1) * vec;
    y *= 0.5f;
    float len = ptvCtrlPts[p3].x - ptvCtrlPts[p2].x;
    // quick fix
    if (len < 0) len += fAniLength;
    x = ptvCtrlPts[p2].x + t * len;
    if (x > fAniLength) {
      x = float_module(x, fAniLength);
    }
    ptvEvaluatedCurvePts.push_back(Point(x, y));
  }
}

void CatmullCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts, 
										 std::vector<Point>& ptvEvaluatedCurvePts, 
										 const float& fAniLength, 
										 const bool& bWrap) const
{
  ptvEvaluatedCurvePts.clear();
	int iCtrlPtCount = ptvCtrlPts.size(); 
  // return if not enough points
  if (iCtrlPtCount < 4) return;

	ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());

  for (int i = 1; i < iCtrlPtCount - 3; i++) {
    // compute for interval (i, i + 1)
    this->evaluate(i - 1, i, i + 1, i + 2, fAniLength, ptvCtrlPts, ptvEvaluatedCurvePts);
  }

	if (bWrap) {
    for (int i = iCtrlPtCount - 2; i < iCtrlPtCount + 1; i++) {
      evaluate((i - 1) % iCtrlPtCount, 
                i % iCtrlPtCount,
                (i + 1) % iCtrlPtCount,
                (i + 2) % iCtrlPtCount,
                fAniLength, ptvCtrlPts, ptvEvaluatedCurvePts);
    }
  } else {
    ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
    ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount-1].y));
  }
}
