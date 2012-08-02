#ifndef _MIX_GAUSSIAN_H_
#define _MIX_GAUSSIAN_H_

#ifndef OPENCV
#define OPENCV
	#include <opencv/cv.h>
	#include <opencv/highgui.h>
#endif

#define MAX_NUM_GAUSSIAN    100

class GMM
{
public:
    GMM();
    ~GMM();

    bool LoadFile( char *filename );
    bool AddGaussian( CvMat * MeanMat, CvMat * CovMat, float Weight );

    float GetProbability( CvMat * Sample );

    void MakeLookUpTable();
    bool SaveLookUpTable( char *filename );
    bool LoadLookUpTable( char *filename );
    float GetProbabilityByLookup( int R, int G, int B );

private:
    int      _nMixture;
    int      _nDim;
    CvMat   *_MeanMat[MAX_NUM_GAUSSIAN];
    CvMat   *_CovMat [MAX_NUM_GAUSSIAN];
    CvMat   *_CovMatI[MAX_NUM_GAUSSIAN];
    float    _Weight [MAX_NUM_GAUSSIAN];

    float    _Probability[256][256][256];
};

#endif // _MIX_GAUSSIAN_H_
