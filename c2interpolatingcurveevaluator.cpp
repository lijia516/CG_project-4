#include "c2interpolatingcurveevaluator.h"
#include <cassert>
#include "math.h"
#include "vec.h"


#define M11	 0.0
#define M12	 1.0
#define M13	 0.0
#define M14	 0.0
#define M21	-0.5
#define M22	 0.0
#define M23	 0.5
#define M24	 0.0
#define M31	 1.0
#define M32	-2.5
#define M33	 2.0
#define M34	-0.5
#define M41	-0.5
#define M42	 1.5
#define M43	-1.5
#define M44	 0.5

inline bool mycmp(const Point a, const Point b) {
    return b.x > a.x;
}


Mat4<float> C2interpolatingCurveEvaluator::m_basisMatrix = Mat4<float>(1.0f, .0f, .0f, .0f,
                                                          .0f, .0f, 1.0f, .0f,
                                                          -3.0f, 3.0f, -2.0f, -1.0f,
                                                          2.0f, -2.0f, 1.0f, 1.0f);


void C2interpolatingCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
                                             std::vector<Point>& ptvEvaluatedCurvePts,
                                             const float& fAniLength,
                                             const bool& bWrap) const
{
    if (s_AddNewPt) return; 
    
    int iCtrlPtCount = ptvCtrlPts.size();
    ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
    ptvEvaluatedCurvePts.clear();
    
    
    
    if (bWrap) {
        
        
        
        std::vector<float> gammer;
        std::vector<float> delta;
        std::vector<float> newPtvCtrlPts;
        
        newPtvCtrlPts.resize(iCtrlPtCount + 1, 0.0f);
        gammer.resize(iCtrlPtCount + 1, 0.0f);
        delta.resize(iCtrlPtCount + 1, 0.0f);
        
        gammer[0] = 0.5f;
        for (int i = 1; i < iCtrlPtCount; i++) {
            gammer[i] = 1.0f / (4.0f - gammer[i - 1]);
        }
        gammer[iCtrlPtCount] = 1.0 / (2.0 - gammer[iCtrlPtCount - 1]);
        
        delta[0] = 1.5 * (ptvCtrlPts[1].y - ptvCtrlPts[0].y);
        for (int i = 1; i < iCtrlPtCount; i++) {
            delta[i] = gammer[i] * (3 * (ptvCtrlPts[i+1].y - ptvCtrlPts[i-1].y) - delta[i-1]);
        }
        delta[iCtrlPtCount] = gammer[iCtrlPtCount] *
        (3 * (ptvCtrlPts[iCtrlPtCount].y - ptvCtrlPts[iCtrlPtCount - 1].y) - delta[iCtrlPtCount]);
        
        newPtvCtrlPts[iCtrlPtCount] = delta[iCtrlPtCount];
        for (int i = iCtrlPtCount - 1; i >= 0; i--) {
            newPtvCtrlPts[i] = delta[i] - gammer[i] * newPtvCtrlPts[i+1];
        }
        
        int i = 0;
        for(; i < iCtrlPtCount; i++){
            
            int p1 = i % iCtrlPtCount;
            int p2 = (i + 1) % iCtrlPtCount;
            
            Vec4f vec = m_basisMatrix * Vec4f(
                                              ptvCtrlPts[p1].y,
                                              ptvCtrlPts[p2].y,
                                              newPtvCtrlPts[p1],
                                              newPtvCtrlPts[p2]
                                              );
            for (int n = 0; n < s_iSegCount; n++) {
                
                float u = (float)n / (float) (s_iSegCount - 1);
                float y = Vec4f(1, u, u*u, u*u*u) * vec;
                
                float len = ptvCtrlPts[p2].x - ptvCtrlPts[p1].x;
                len = len < 0 ? len + fAniLength : len;
                float x = ptvCtrlPts[p1].x + u * len;
                x = x > fAniLength ? x - fAniLength : x;
                ptvEvaluatedCurvePts.push_back(Point(x, y));
            }
        }

        
        
    } else {
        
        std::vector<float> gammer;
        std::vector<float> delta;
        std::vector<float> newPtvCtrlPts;
        
        newPtvCtrlPts.resize(iCtrlPtCount, 0.0f);
        gammer.resize(iCtrlPtCount, 0.0f);
        delta.resize(iCtrlPtCount, 0.0f);
   
        gammer[0] = 0.5f;
        for (int i = 1; i < iCtrlPtCount - 1; i++) {
            gammer[i] = 1.0f / (4.0f - gammer[i - 1]);
        }
        gammer[iCtrlPtCount - 1] = 1.0 / (2.0 - gammer[iCtrlPtCount - 2]);
        
        delta[0] = 1.5 * (ptvCtrlPts[1].y - ptvCtrlPts[0].y);
        for (int i = 1; i < iCtrlPtCount - 1; i++) {
            delta[i] = gammer[i] * (3 * (ptvCtrlPts[i+1].y - ptvCtrlPts[i-1].y) - delta[i-1]);
        }
        delta[iCtrlPtCount - 1] = gammer[iCtrlPtCount - 1] *
        (3 * (ptvCtrlPts[iCtrlPtCount - 1].y - ptvCtrlPts[iCtrlPtCount - 2].y) - delta[iCtrlPtCount - 1]);
        
        newPtvCtrlPts[iCtrlPtCount - 1] = delta[iCtrlPtCount - 1];
        for (int i = iCtrlPtCount - 2; i >= 0; i--) {
            newPtvCtrlPts[i] = delta[i] - gammer[i] * newPtvCtrlPts[i+1];
        }
        
        int i = 0;
        for(; i < iCtrlPtCount - 1; i++){
            
            int p1 = i;
            int p2 = i + 1;
            
            Vec4f vec = m_basisMatrix * Vec4f(
                                              ptvCtrlPts[p1].y,
                                              ptvCtrlPts[p2].y,
                                              newPtvCtrlPts[p1],
                                              newPtvCtrlPts[p2]
                                              );
            for (int n = 0; n < s_iSegCount; n++) {
               
                float u = (float)n / (float) (s_iSegCount - 1);
                float y = Vec4f(1, u, u*u, u*u*u) * vec;
                
                float len = ptvCtrlPts[p2].x - ptvCtrlPts[p1].x;
                float x = ptvCtrlPts[p1].x + u * len;
                
                ptvEvaluatedCurvePts.push_back(Point(x, y));
            }
        }
        
        // start
        ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
        // end
        ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount-1].y));
    
        
    }
         
         
}
