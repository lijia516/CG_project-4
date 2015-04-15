#include "c2interpolatingcurveevaluator.h"
#include <cassert>
#include "mat.h"

inline bool mycmp(const Point a, const Point b) {
    return b.x > a.x;
}

Mat4<float> C2interpolatingCurveEvaluator::m_basisMatrix = Mat4<float>(1.0f, .0f, .0f, .0f,
                                                          .0f, .0f, 1.0f, .0f,
                                                          -3.0f, 3.0f, -2.0f, -1.0f,
                                                          2.0f, -2.0f, 1.0f, 1.0f);

void C2interpolatingCurveEvaluator::evaluate(const int p1, const int p2, const float& fAniLength,
                                const std::vector<Point>& ptvCtrlPts, std::vector<Point>& ptvEvaluatedCurvePts,
                                const std::vector<float>& deri_pts
                                )const {
    float t, y, x;
    Vec4 vec = m_basisMatrix * Vec4 (ptvCtrlPts[p1].y,
                                    ptvCtrlPts[p2].y,
                                    deri_pts[p1],
                                    deri_pts[p2]
                                    );
    for (int j = 0; j < 100; j++) {
        t = j / 100.0f;
        y = Vec4<float>(1, t, t*t, t*t*t) * vec;
        float len = ptvCtrlPts[p2].x - ptvCtrlPts[p1].x;
        // still quick fix ?
        if (len < 0) len += fAniLength;
        x = ptvCtrlPts[p1].x + t * len;
        if (x > fAniLength)
            x = x - fAniLength;
        ptvEvaluatedCurvePts.push_back(Point(x, y));
    }
}

void C2interpolatingCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
										 std::vector<Point>& ptvEvaluatedCurvePts, 
										 const float& fAniLength, 
										 const bool& bWrap) const
{
    int iCtrlPtCount = ptvCtrlPts.size();
    ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
    ptvEvaluatedCurvePts.clear();
    
    
    
    if (bWrap) {
        
        
        
        
    } else {
        
        
        ptvEvaluatedCurvePts.clear();
        
        int ctpn = ptvCtrlPts.size();
        if (ctpn < 2) return;
        
        std::vector<Point> copy_ctrlpts = std::vector<Point>(ptvCtrlPts);
        std::sort(copy_ctrlpts.begin(), copy_ctrlpts.end(), mycmp);
        std::vector<float> deri_pts;
        
        // compute the deri_pts
        std::vector<float> gammer;
        std::vector<float> delta;
        
        deri_pts.resize(ctpn + 1, 0.0f);
        gammer.resize(ctpn + 1, 0.0f);
        delta.resize(ctpn + 1, 0.0f);
        
        // automatically consider wrapping
        copy_ctrlpts.push_back(Point(copy_ctrlpts[0].x + fAniLength, copy_ctrlpts[0].y));
        // so there should be n + 1 points
        
        int control_n;
        if (bWrap) {
            control_n = ctpn;
        } else {
            control_n = ctpn - 1;
        }
        
        int i;
        gammer[0] = 0.5f;
        for (i = 1; i < control_n; i++) {
            gammer[i] = 1.0f / (4.0f - gammer[i - 1]);
        }
        gammer[control_n] = 1.0 / (2.0 - gammer[control_n - 1]);
        
        delta[0] = 1.5 * (copy_ctrlpts[1].y - copy_ctrlpts[0].y);
        for (i = 1; i < control_n; i++) {
            delta[i] = gammer[i] * (3 * (copy_ctrlpts[i+1].y - copy_ctrlpts[i-1].y) - delta[i-1]);
        }
        delta[control_n] = gammer[control_n] *
        (3 * (copy_ctrlpts[control_n].y - copy_ctrlpts[control_n - 1].y) - delta[control_n]);
        
        deri_pts[control_n] = delta[control_n];
        for (i = control_n - 1; i >= 0; i--) {
            deri_pts[i] = delta[i] - gammer[i] * deri_pts[i+1];
        }
        
        // put them into the evaluator
        for (int i = 0; i < control_n; i++) {
            this->evaluate(i, i + 1, fAniLength, copy_ctrlpts, ptvEvaluatedCurvePts, deri_pts);
        }
        
    }

}
