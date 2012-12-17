#include "HandRegion.h"
#include <stdio.h>
#include <vector>

using namespace std;

const int		CVCLOSE_ITR	 = 1;
const CvScalar	CV_CVX_WHITE = cvScalar(255);
const CvScalar	CV_CVX_BLACK = cvScalar(0);
const int		CVCONTOUR_APPROX_LEVEL = 1;

namespace{

// Remove certain contour with area less than threshold
void removeNoise( IplImage* src, int size )
{    
	IplImage* tmp = cvCreateImage(cvSize(src->width, src->height),IPL_DEPTH_8U,1);
	cvCopyImage(src, tmp);
	CvMemStorage* storage   = cvCreateMemStorage( 0 );     
	CvSeq* contours         = NULL;    
	CvScalar black          = CV_RGB( 0, 0, 0 ); 
	CvScalar white          = CV_RGB( 255, 255, 255 ); 
    double area;   
	cvFindContours( tmp, storage, &contours, sizeof( CvContour ), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );  

	while( contours )   {  
		area = cvContourArea( contours, CV_WHOLE_SEQ );       
		if( fabs( area ) <= size )  {// if less than threshold then remove by paint over white by black           
			cvDrawContours( src, contours, black, black, -1, CV_FILLED, 8 );      
		}     
		contours = contours->h_next;   
	}   
	cvReleaseMemStorage( &storage );    
	cvReleaseImage(&tmp);
}

	//Connected component(Labeling) from opencv book example 
void getConnectedComponents(IplImage *mask, int poly1_hull0, float perimScale, int *cont_num, std::vector <CvRect> & cont_boundbox, std::vector <CvBox2D> & cont_boundbox2D, std::vector <CvPoint> & cont_center) {
		static CvMemStorage*	mem_storage	= NULL;
		static CvSeq*			contours	= NULL;

		//CLEAN UP RAW MASK
//		cvMorphologyEx( mask, mask, NULL, NULL, CV_MOP_OPEN, CVCLOSE_ITR );
//		cvMorphologyEx( mask, mask, NULL, NULL, CV_MOP_CLOSE, CVCLOSE_ITR );

//		removeNoise(mask,20);

		//FIND CONTOURS AROUND ONLY BIGGER REGIONS
		if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
		else cvClearMemStorage(mem_storage);

		CvContourScanner scanner = cvStartFindContours(mask,mem_storage,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
		CvSeq* c;
		int numCont = 0;
		while( (c = cvFindNextContour( scanner )) != NULL )
		{
			double len = cvContourPerimeter( c );
			double q = (mask->height + mask->width) /perimScale;   //calculate perimeter len threshold

			if( len < q ) //Get rid of blob if it's perimeter is too small
			{
				cvSubstituteContour( scanner, NULL );
			}
			else //Smooth it's edges if it's large enough
			{
				CvSeq* c_new;
				if(poly1_hull0) //Polygonal approximation of the segmentation
					c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, CVCONTOUR_APPROX_LEVEL,0);
				else //Convex Hull of the segmentation
					c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
				cvSubstituteContour( scanner, c_new );
				numCont++;
			}
		}
		contours = cvEndFindContours( &scanner );

		// PAINT THE FOUND REGIONS BACK INTO THE IMAGE
		cvZero( mask );
		IplImage *maskTemp;
		//CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
		if(cont_num != NULL)
		{
			int N = *cont_num, numFilled = 0, i=0;
			CvMoments moments;
			double M00, M01, M10;
			maskTemp = cvCloneImage(mask);
			for(i=0, c=contours; c != NULL; c = c->h_next,i++ )
			{
				if(i < N) //Only process up to *num of them
				{
					cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
					//Find the center of each contour
					cvMoments(maskTemp,&moments,1);
					M00 = cvGetSpatialMoment(&moments,0,0);
					M10 = cvGetSpatialMoment(&moments,1,0);
					M01 = cvGetSpatialMoment(&moments,0,1);
					cont_center.push_back(cvPoint((int)(M10/M00),(int)(M01/M00)));
					//centers[i].x = (int)(M10/M00);
					//centers[i].y = (int)(M01/M00);
					//Bounding rectangles around blobs
					cont_boundbox.push_back(cvBoundingRect(c));
					cont_boundbox2D.push_back(cvMinAreaRect2(c));
					cvZero(maskTemp);
					numFilled++;
				}
				//Draw filled contours into mask
				cvDrawContours(mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
			} //end looping over contours
			*cont_num = numFilled;
			cvReleaseImage( &maskTemp);

		}
		//ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
		else
		{
			for( c=contours; c != NULL; c = c->h_next )
			{
				cvDrawContours(mask,c,CV_CVX_WHITE, CV_CVX_BLACK,-1,CV_FILLED,8);
			}
		}
		cvReleaseMemStorage( &mem_storage );   
	}
}

HandRegion::HandRegion(void)
{
	_pImage = 0;
}

HandRegion::~HandRegion(void)
{
    if ( _pImage )
    {
        cvReleaseImage( &_pImage );
    }
}

bool HandRegion::LoadSkinColorProbTable()
{
    if ( _SkinColor.LoadLookUpTable( "skin.dis" ) == false )
    {
        if ( _SkinColor.LoadFile( "skin.mgm" ) == false )
        {
            fprintf( stderr, "skin color distribution load error.\n" );
            return false;
        }
        printf("making a lookup table for skin color distribution ");
        _SkinColor.MakeLookUpTable();
        printf("done\n");
        if ( _SkinColor.SaveLookUpTable( "skin.dis" ) == false )
        {
            fprintf( stderr, "skin color distribution look up table save error.\n" );
            return false;
        }
    }
    if ( _NonSkinColor.LoadLookUpTable( "nonskin.dis" ) == false )
    {
        if ( _NonSkinColor.LoadFile( "nonskin.mgm" ) == false )
        {
            fprintf( stderr, "non-skin color distribution load error.\n" );
            return false;
        }
        printf("making a lookup table for non-skin color distribution ");
        _NonSkinColor.MakeLookUpTable();
        printf("done\n");
        if ( _NonSkinColor.SaveLookUpTable( "nonskin.dis" ) == false )
        {
            fprintf( stderr, "non-skin color distribution look up table save error.\n" );
            return false;
        }
    }

    return true;
}


IplImage * HandRegion::GetHandRegion( IplImage * srcImage, int *cont_num, std::vector <CvRect> & cont_boundbox,  std::vector <CvBox2D> & cont_boundbox2D, std::vector <CvPoint> & cont_center)
{
    //
    // Initialize Memory for Image
    //
    if ( !_pImage )
    {
        _pImage = cvCreateImage( cvGetSize( srcImage ), 8, 1 );
        _pImage->origin = srcImage->origin;
    }
	
    //
    // Segmentation by Color Distribution
    //
    for ( int i = 0 ; i < srcImage->height ; i ++ )
    {
        for ( int j = 0 ; j < srcImage->width ; j ++ )
        {
            unsigned char R = srcImage->imageData[ i * srcImage->widthStep + j*3 + 2];
            unsigned char G = srcImage->imageData[ i * srcImage->widthStep + j*3 + 1];
            unsigned char B = srcImage->imageData[ i * srcImage->widthStep + j*3 + 0];

            float P_Skin = _SkinColor.GetProbabilityByLookup( R, G, B );
            float P_NonSkin = _NonSkinColor.GetProbabilityByLookup( R, G, B );

            float P = P_Skin/P_NonSkin;
            if ( P < 0.4 ) {
                _pImage->imageData[ i * _pImage->widthStep + j ] = 0;
            }
            else
            {
                _pImage->imageData[ i * _pImage->widthStep + j ] = 255;
						}
        }
    }
	// find connected components
	getConnectedComponents(_pImage,1,4,cont_num,cont_boundbox, cont_boundbox2D,cont_center);//int *num, CvRect *bbs, CvPoint *centers

	//IplImage *temp = (IplImage*) cvClone(_pImage);
	//cv::Mat handMask = temp;

	//vector< vector<cv::Point> > fingerTips;
	//vector< vector<cv::Point> > contours;

	//fingerTips.clear();
	//contours.clear();
	//cv::findContours(handMask, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//for (unsigned int i=0; i<contours.size(); i++){
	//	vector<cv::Point> contour = contours[i];
	//	cv::Mat contourMat = cv::Mat(contour);
	//	double area = cv::contourArea(contourMat);

	//	if (area > 200)  { // possible hand
	//		vector<cv::Point> tmp_fingertips;
	//		tmp_fingertips.clear();

	//		cv::Scalar center = mean(contourMat);
	//		cv::Point centerPoint = cv::Point(static_cast<int>(center.val[0]), static_cast<int>(center.val[1]) );
	//		vector<cv::Point> approxCurve;
	//		cv::approxPolyDP(contourMat, approxCurve, 15, true);

	//		vector<int> hull;
	//		cv::convexHull(cv::Mat(approxCurve), hull);

	//		// find upper and lower bounds of the hand and define cutoff threshold (don't consider lower vertices as fingers)
	//		int upper = 640, lower = 0;
	//		for (unsigned int j=0; j<hull.size(); j++) {
	//			int idx = hull[j]; // corner index
	//			if (approxCurve[idx].y < upper) upper = approxCurve[idx].y;
	//			if (approxCurve[idx].y > lower) lower = approxCurve[idx].y;
	//		}

	//		float cutoff = lower - (lower - upper) * 0.1f;
	//		// find interior angles of hull corners
	//		for (unsigned int j=0; j<hull.size(); j++) {
	//			int idx = hull[j]; // corner index
	//			int pdx = idx == 0 ? approxCurve.size() - 1 : idx - 1; //  predecessor of idx
	//			int sdx = idx == approxCurve.size() - 1 ? 0 : idx + 1; // successor of idx
	//			cv::Point v1 = approxCurve[sdx] - approxCurve[idx];
	//			cv::Point v2 = approxCurve[pdx] - approxCurve[idx];
	//			double angle = acos( (v1.x*v2.x + v1.y*v2.y) / (cv::norm(v1) * cv::norm(v2)) );
	//			// low interior angle + within upper 90% of region -> we got a finger
	//			if ( angle < 1 && approxCurve[idx].y < cutoff) {
	//				tmp_fingertips.push_back(cv::Point( approxCurve[idx].x , approxCurve[idx].y) );
	//			}
	//		}
	//		fingerTips.push_back(tmp_fingertips);
	//	}
	//}
  return _pImage;
}