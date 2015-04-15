#include "beziercurveevaluator.h"
#include <cassert>
#include "math.h"

int BezierCurveEvaluator::parameter( int n,  int k) const
{
    int r = 1;
    if(k > n)
        return 0;
    for( int d = 1; d <= k; d++)
    {
        r *= n--;
        r /= d;
    }
    return r;
}
void BezierCurveEvaluator::evaluateCurve(const std::vector<Point>& ptvCtrlPts,
                                         std::vector<Point>& ptvEvaluatedCurvePts,
                                         const float& fAniLength,
                                         const bool& bWrap) const
{
    int iCtrlPtCount = ptvCtrlPts.size();
    ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
    ptvEvaluatedCurvePts.clear();
    
    float x = 0.0;
    float y1;
    
    if (iCtrlPtCount < 4) {
        ptvEvaluatedCurvePts.assign(ptvCtrlPts.begin(), ptvCtrlPts.end());
        
        if (bWrap) {
            // if wrapping is on, interpolate the y value at xmin and
            // xmax so that the slopes of the lines adjacent to the
            // wraparound are equal.
            
            if ((ptvCtrlPts[0].x + fAniLength) - ptvCtrlPts[iCtrlPtCount - 1].x > 0.0f) {
                y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x) +
                      ptvCtrlPts[iCtrlPtCount - 1].y * ptvCtrlPts[0].x) /
                (ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x);
            }
            else
                y1 = ptvCtrlPts[0].y;
        }
        else {
            // if wrapping is off, make the first and last segments of
            // the curve horizontal.
            
            y1 = ptvCtrlPts[0].y;
        }
        
        ptvEvaluatedCurvePts.push_back(Point(x, y1));
        
        /// set the endpoint based on the wrap flag.
        float y2;
        x = fAniLength;
        if (bWrap)
            y2 = y1;
        else
            y2 = ptvCtrlPts[iCtrlPtCount - 1].y;
        
        ptvEvaluatedCurvePts.push_back(Point(x, y2));
        
        return;
    }
    
    
    // Bezier
    
    int i = 0;
    for (; i + 3 < iCtrlPtCount; i += 3){
        
            for(float n=0; n < s_iSegCount; n++){
        
                float u = ((float)n)/((float)s_iSegCount-1);
                float x=0;
                float y=0;
                
                for(int j=0; j < 4;j++){
                    float factor =  parameter(3,j) * pow(u,j) * pow((1-u),3-j);
                    x += factor*(ptvCtrlPts[j+i].x);
                    y += factor*(ptvCtrlPts[j+i].y);
                }
                ptvEvaluatedCurvePts.push_back(Point(x,y));
            }
        }
    
    
    if (bWrap) {
        
        std::cout << "i" << i << std::endl;
        
        if (iCtrlPtCount - i == 3) {
        
            for(float n=0; n < s_iSegCount; n++){
                
                float u = ((float)n)/((float)s_iSegCount-1);
                float x=0;
                float y=0;
                
                for(int j=0; j < 4;j++){
                    
                    float factor =  parameter(3,j) * pow(u,j) * pow((1-u),3-j);
                    
                    int index = i + j >= iCtrlPtCount ? 0 : i + j;
                    
                    float x_temp = ptvCtrlPts[index].x;
                    float y_temp = ptvCtrlPts[index].y;
                
                    if (index == 0)
                        x_temp += fAniLength;
                    
                    x += factor*(x_temp);
                    y += factor*(y_temp);
                    
                    std::cout << "index" << index << std::endl;
                    
                }
                std::cout << "x, y: " << x << "," << y << std::endl;
                
                x = x > fAniLength ? x - fAniLength : x;
                
                ptvEvaluatedCurvePts.push_back(Point(x,y));
            }
            
            
        } else {
            
             for (;i < iCtrlPtCount; i++)
                ptvEvaluatedCurvePts.push_back(ptvCtrlPts[i]);
            
            float y1;
            
            if ((ptvCtrlPts[0].x + fAniLength) - ptvCtrlPts[iCtrlPtCount - 1].x > 0.0f) {
                y1 = (ptvCtrlPts[0].y * (fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x) +
                      ptvCtrlPts[iCtrlPtCount - 1].y * ptvCtrlPts[0].x) /
                (ptvCtrlPts[0].x + fAniLength - ptvCtrlPts[iCtrlPtCount - 1].x);
            }
            else
                y1 = ptvCtrlPts[0].y;
            
            ptvEvaluatedCurvePts.push_back(Point(0, y1));
            ptvEvaluatedCurvePts.push_back(Point(fAniLength, y1));
        
            
        }
        
    } else {
        
    
        for (;i < iCtrlPtCount; i++)
            ptvEvaluatedCurvePts.push_back(ptvCtrlPts[i]);
        
        // start
        ptvEvaluatedCurvePts.push_back(Point(0, ptvCtrlPts[0].y));
        // end
        ptvEvaluatedCurvePts.push_back(Point(fAniLength, ptvCtrlPts[iCtrlPtCount-1].y));
    
    }
}
