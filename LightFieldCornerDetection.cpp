// To Do: Set JPG quality setting to 100% in MATLAB capture script!!!
// Somehow I'm referencing the same cluster more than once...likely an issue with indexing through the 48 corners.

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <lapackpp/laslv.h>
#include <lapackpp/blas1pp.h>
#include <lapackpp/blas3pp.h>
// Test alglib
#include "alglib_src/ap.h"
#include "alglib_src/interpolation.h"
using namespace alglib_impl;


using namespace cv;
using namespace std;

// Function to draw optical flow
void drawOptFlowMap(const Mat& flow, Mat& cflowmap, int step, double, const Scalar& color){
	for(int y=0; y<cflowmap.rows; y+=step){
		for(int x=0; x<cflowmap.cols; x+=step){
			const Point2f& fxy = flow.at<Point2f>(y, x);
			line(cflowmap, Point(x,y), Point(cvRound(x+fxy.x), cvRound(y+fxy.y)), color);
			circle(cflowmap, Point(x,y), 1, color, -1);}}}

// Function to calculate 3D position of a point
/*void interpolatePosition(

	// Set calibration grid spacing (mm)
	float gridSpacing = 5.29;//mm
	float tempDist, minSqrDist;
	int minDistCornerIndex[2];
	bool isInteriorX, isInteriorY;

	// Find closest board vertex to point
	minSqrDist=100000000; minDistCorner=-1;
	for(kk=1; kk<=boardSize.width; kk++){
		for(ll=1; ll<=boardSize.height; ll++){
			tempDist=pow(ii-corners[ii-1][jj-1][kk-1][ll-1][0],2)+pow(jj-corners[ii-1][jj-1][kk-1][ll-1][1],2);
			if(tempDist < minSqrDist){
				minSqrDist=tempDist;  minDistCornerIndex[0]=kk+1; minDistCornerIndex[1]=ll+1;

	// Is this point interior to the grid?
	if((minDistCornerIndex[0]>1 && minDistCornerIndex[0]<boardSize.width)){




		if(minDistCornerIndex[1]>1 && minDistCornerIndex[1]<boardSize.height){*/

int main( int argc, char** argv ){
	// Define some variables
	Size boardSize, imageSize, lightFieldSize;
	float squareSize = 1.f, aspectRatio = 1.f;
	const char * outputFilename = "out_camera_data.yml";
	char *   pathToCalibrationLightField;//Watch for seg faults?
	char *   pathToscanLightField;//Watch for seg faults?
	char pathToCurrentImage[200];//Watch for seg faults?
	int ii, jj, kk, ll, mm;
	bool writePoints = false;
	int flags = 0;
	string inputFilename;

/*
	PARSE COMMAND LINE ARGUMENTS -----
*/

	// Check for correct number of arguments and print usage if incorrect
	if( argc < 2 ){
		printf( "This is a camera calibration sample.\n"
			"Usage: calibration\n"
			"     -lw <light_field_width>         # Light field width\n"
			"     -lh <light_field_height>        # Light field height\n"
			"     -bw <board_width>         # the number of inner corners per one of board dimension\n"
			"     -bh <board_height>        # the number of inner corners per another board dimension\n"
			"     -p <path_to_light_field> # the number of inner corners per another board dimension\n"
			"     [-s <squareSize>]        # square size in some user-defined units (1 by default)\n"
			"     [-o <out_camera_params>] # Output filename for intrinsic [and extrinsic] parameters\n"
			"     [-op]                    # write detected feature points\n"
			"     [input_data]             # input image name\n" );
		return 0;}
	// Parse command-line arguments
	for( ii = 1; ii < argc; ii++ ){
		const char* s = argv[ii];
		if( strcmp( s, "-lw" ) == 0 ){
			if( sscanf( argv[++ii], "%u", &lightFieldSize.width ) != 1 || lightFieldSize.width <= 0 )
				return fprintf( stderr, "Invalid light field width\n" ), -1;}
		else if( strcmp( s, "-lh" ) == 0 ){
			if( sscanf( argv[++ii], "%u", &lightFieldSize.height ) != 1 || lightFieldSize.height <= 0 )
				return fprintf( stderr, "Invalid light field height\n" ), -1;}
		else if( strcmp( s, "-bw" ) == 0 ){
			if( sscanf( argv[++ii], "%u", &boardSize.width ) != 1 || boardSize.width <= 0 )
				return fprintf( stderr, "Invalid board width\n" ), -1;}
		else if( strcmp( s, "-bh" ) == 0 ){
			if( sscanf( argv[++ii], "%u", &boardSize.height ) != 1 || boardSize.height <= 0 )
				return fprintf( stderr, "Invalid board height\n" ), -1;}
		else if( strcmp( s, "-s" ) == 0 ){
			if( sscanf( argv[++ii], "%f", &squareSize ) != 1 || squareSize <= 0 )
				return fprintf( stderr, "Invalid board square width\n" ), -1;}
		else if( strcmp( s, "-op" ) == 0 ){
			writePoints = 1;}
		else if( strcmp( s, "-o" ) == 0 ){
			outputFilename = argv[++ii];}
		else if( strcmp( s, "-calibrationLF" ) == 0 ){
			  pathToCalibrationLightField = argv[++ii];}
		else if( strcmp( s, "-scanLF" ) == 0 ){
			  pathToscanLightField = argv[++ii];}
		else if( s[0] != '-' ){
			inputFilename = string(s);}
		else{return fprintf( stderr, "Unknown option %s", s ), -1;}}

	// Check output
	printf("Light field path: %s\n",  pathToCalibrationLightField);

/*
	RUN CORNER DETECTION -----
*/

	// Create image objects and matrix of coordinates to hold corner information
	Mat view, viewGray;
	float ***** corners = new float **** [lightFieldSize.width];
	for(ii=1; ii<=lightFieldSize.width; ii++){
		corners[ii-1] = new float *** [lightFieldSize.height];
		for(jj=1; jj<=lightFieldSize.height; jj++){
			corners[ii-1][jj-1] = new float ** [boardSize.width];
			for(kk=1; kk<=boardSize.width; kk++){
				corners[ii-1][jj-1][kk-1] = new float * [boardSize.height];
				for(ll=1; ll<=boardSize.height; ll++){
					corners[ii-1][jj-1][kk-1][ll-1] = new float[2];}}}}

	// Loop through each image in light field to detect, print and display corners
	int numCorners=0;
	int expectedNumCorners = lightFieldSize.width*lightFieldSize.height*boardSize.width*boardSize.height;
	for(ii=1; ii<=lightFieldSize.width; ii++){
		for(jj=1; jj<=lightFieldSize.height; jj++){

			// Load image and get size
			sprintf(pathToCurrentImage, "%s/LightField%i_%i.jpg",  pathToCalibrationLightField, ii, jj);
			view = imread(pathToCurrentImage,1);
			imageSize = view.size();

			// Determine chessboard corner positions
			vector<Point2f> pointbuf;
			bool found = findChessboardCorners(view, boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH);

			// improve the found corners' coordinate accuracy
			cvtColor(view, viewGray, CV_BGR2GRAY);
			if(found) cornerSubPix(viewGray, pointbuf, Size(11,11),
				Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

			// If found, save & print corner locations then draw corners on original image view
			for(kk=1; kk<=boardSize.width; kk++){
				for(ll=1; ll<=boardSize.height; ll++){
					corners[ii-1][jj-1][kk-1][ll-1][0] = pointbuf[(ll-1)*boardSize.width+(kk-1)].x;
					corners[ii-1][jj-1][kk-1][ll-1][1] = pointbuf[(ll-1)*boardSize.width+(kk-1)].y;}}
			if(found){drawChessboardCorners(view, boardSize, Mat(pointbuf), found);}

			// Display original image, with corners if found, and wait for user response
			imshow("Light Field Corner Detection -- Tom Milnes -- MIT", view); int key = waitKey(1);//1ms waitKey used to handle re-draw events
			numCorners += pointbuf.size();}}

/*
	CALCULATE LEAST SQUARES GRID FITS -----
*/	

	// Initialize variables
	int gridRange[4] = {4,8,2,6};//[wmin,wmax,hmin,hmax]
	//int gridRange[4] = {1,10,1,10};//[wmin,wmax,hmin,hmax]
	int numGridPoints = (gridRange[1]-gridRange[0]+1)*(gridRange[3]-gridRange[2]+1);
	float *** calibrationGridFit = new float ** [boardSize.width];
	for(ii=1; ii<=boardSize.width; ii++){
		calibrationGridFit[ii-1] = new float * [boardSize.height];
		for(jj=1; jj<=boardSize.height; jj++){
			calibrationGridFit[ii-1][jj-1] = new float[3];}}
	// Define "A" ("C") Matrix -- Common to each cluster (as long as each cluster is complete)
	LaGenMatDouble A(2*numGridPoints,3);
	for(ii=0;ii<numGridPoints;ii++){//1*Ox
		A(ii,0) = 1;
		A(ii,1) = 0;}
	for(ii=numGridPoints;ii<2*numGridPoints;ii++){//1*Oy
		A(ii,0) = 0;
		A(ii,1) = 1;}
	for(ii=0;ii<(gridRange[1]-gridRange[0]+1);ii++){//Sg coefficients for X values
		for(jj=0;jj<(gridRange[3]-gridRange[2]+1);jj++){
			A(ii*(gridRange[1]-gridRange[0]+1)+jj,2) = ii;}}
	for(ii=0;ii<(gridRange[1]-gridRange[0]+1);ii++){//Sg coefficients for Y values
		for(jj=0;jj<(gridRange[3]-gridRange[2]+1);jj++){
			A(numGridPoints+ii*(gridRange[1]-gridRange[0]+1)+jj,2) = jj;}}

	// Find "Grid of Best Fit" for each cluster
	for(ii=1; ii<=boardSize.width; ii++){
		for(jj=1; jj<=boardSize.height; jj++){

			// Define "x" and "b" matrices
			LaVectorDouble x(3);
			LaVectorDouble b(numGridPoints*2);
			mm = 1;
			for(kk=gridRange[0]; kk<=gridRange[1]; kk++){
				for(ll=gridRange[2]; ll<=gridRange[3]; ll++){
					b(mm-1) = corners[kk-1][ll-1][ii-1][jj-1][0];
					b(mm-1+numGridPoints) = corners[kk-1][ll-1][ii-1][jj-1][1];
					mm++;}}
			//cout << b << endl;

			// Calculate solution and save results
			LaLinearSolve(A,x,b);
			//cout<<"Ox = "<<x(0)<<"\n";	cout<<"Oy = "<<x(1)<<"\n";	cout<<"Sg = "<<x(2)<<"\n";
			for(mm=0;mm<3;mm++){
				calibrationGridFit[ii-1][jj-1][mm] = x(mm);}}}

/*
	OUTPUT CALIBRATION DATA FOR PLOTTING AND OTHER USAGE -----
*/

	// Write out corner data in simple gnuplot format
	FILE * cornerData = fopen("cornerData.txt","w"); fprintf(cornerData,"#X			Y\n");
	for(ii=gridRange[0]; ii<=gridRange[1]; ii++){
		for(jj=gridRange[2]; jj<=gridRange[3]; jj++){
			for(kk=1; kk<=boardSize.width; kk++){
				for(ll=1; ll<=boardSize.height; ll++){
					fprintf(cornerData,"%.2f\t%.2f\n",corners[ii-1][jj-1][kk-1][ll-1][0],corners[ii-1][jj-1][kk-1][ll-1][1]);}}}}
	// Write out grid fit data in simple gnuplot format
	FILE * gridFitData = fopen("gridFitData.txt","w"); fprintf(gridFitData,"#X			Y\n");
	for(ii=1; ii<=boardSize.width; ii++){
		for(jj=1; jj<=boardSize.height; jj++){
			for(kk=0; kk<(gridRange[1]-gridRange[0]+1); kk++){
				for(ll=0; ll<(gridRange[3]-gridRange[2]+1); ll++){
					fprintf(gridFitData,"%.2f\t%.2f\n", calibrationGridFit[ii-1][jj-1][0]+calibrationGridFit[ii-1][jj-1][2]*(kk), calibrationGridFit[ii-1][jj-1][1]+calibrationGridFit[ii-1][jj-1][2]*(ll));}}}}
	// Close files when done writing
	fclose(cornerData); fclose(gridFitData);

/*
	LOAD scan LIGHT FIELD AND CALCULATE OPTICAL FLOW -----
*/

	// Determine "Center" of light field and compute flow relative to this central image
	int centralImageX = lightFieldSize.width/2, centralImageY = lightFieldSize.height/2;
	
	// Create "Flow" data structure -- HUGE! -- and initialize all values to zero
	float ***** opticalFlow = new float **** [lightFieldSize.width];
	for(ii=1; ii<=lightFieldSize.width; ii++){
		opticalFlow[ii-1] = new float *** [lightFieldSize.height];
		for(jj=1; jj<=lightFieldSize.height; jj++){
			opticalFlow[ii-1][jj-1] = new float ** [imageSize.width];
			for(kk=1; kk<=imageSize.width; kk++){
				opticalFlow[ii-1][jj-1][kk-1] = new float * [imageSize.height];
				for(ll=1; ll<=imageSize.height; ll++){
					opticalFlow[ii-1][jj-1][kk-1][ll-1] = new float[2];
					for(mm=1; mm<=2; mm++){
						opticalFlow[ii-1][jj-1][kk-1][ll-1][mm-1] = 0;}}}}}

	// Declare some variables
	Mat flow, view1, view2, grayView1, grayView2;
	namedWindow("flow", 1);

	// Load "central," reference image
	sprintf(pathToCurrentImage, "%s/LightField%i_%i.jpg", pathToscanLightField, centralImageX, centralImageY);
	view1 = imread(pathToCurrentImage, 1); cvtColor(view1, grayView1, CV_BGR2GRAY);

	// Calculate and save flow
	for(ii=1; ii<=lightFieldSize.width; ii++){
		for(jj=1; jj<=lightFieldSize.height; jj++){
			if(ii==centralImageX && jj==centralImageY){continue;}//Skip reference image
			sprintf(pathToCurrentImage, "%s/LightField%i_%i.jpg", pathToscanLightField, ii, jj);
			view2 = imread(pathToCurrentImage, 1); cvtColor(view2, grayView2, CV_BGR2GRAY);
			// Calculate and display flow, "destroying" view2
			calcOpticalFlowFarneback(grayView1, grayView2, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
			drawOptFlowMap(flow, view2, 16, 1.5, CV_RGB(0, 255, 0));
			imshow("flow", view2);
			int key = waitKey(1);//1ms waitKey to handle re-draw events
			// Save flow values to opticalFlow matrix
			for(kk=1; kk<=imageSize.width; kk++){
				for(ll=1; ll<=imageSize.height; ll++){
					opticalFlow[ii-1][jj-1][kk-1][ll-1][0] = flow.at<Point2f>(ll,kk).x;
					opticalFlow[ii-1][jj-1][kk-1][ll-1][1] = flow.at<Point2f>(ll,kk).y;}}}}

	// Write out diagnostic data for gnuplot plotting and other uses
	int diagnosticStepSize = 25;
	FILE * flowData = fopen("flowData.txt","w"); fprintf(flowData,"#X			Y\n");
	for(ii=1; ii<=imageSize.width; ii+=diagnosticStepSize){
		for(jj=1; jj<=imageSize.height; jj+=diagnosticStepSize){
			for(kk=gridRange[0]; kk<=gridRange[1]; kk++){
				for(ll=gridRange[2]; ll<=gridRange[3]; ll++){
					fprintf(flowData,"%.2f\t%.2f\n",ii+opticalFlow[kk-1][ll-1][ii-1][jj-1][0],jj+opticalFlow[kk-1][ll-1][ii-1][jj-1][1]);}}}}
	fclose(flowData);
	//int key = waitKey(0);

/*
	CALCULATE GRIDS OF BEST FIT FOR OPTICAL FLOW DATA-----
*/

	// Initialize variables, assuming A can be used as-is from above
	float *** scanGridFit = new float ** [imageSize.width];
	for(ii=1; ii<=imageSize.width; ii++){
		scanGridFit[ii-1] = new float * [imageSize.height];
		for(jj=1; jj<=imageSize.height; jj++){
			scanGridFit[ii-1][jj-1] = new float[4];}}

	// Find "Grid of Best Fit" for each cluster
	for(ii=1; ii<=imageSize.width; ii++){
		for(jj=1; jj<=imageSize.height; jj++){

			// Define "x" and "b" matrices
			LaVectorDouble x(3);
			LaVectorDouble b(numGridPoints*2);
			mm = 1;
			for(kk=gridRange[0]; kk<=gridRange[1]; kk++){
				for(ll=gridRange[2]; ll<=gridRange[3]; ll++){
					b(mm-1) = ii + opticalFlow[kk-1][ll-1][ii-1][jj-1][0];
					b(mm-1+numGridPoints) = jj + opticalFlow[kk-1][ll-1][ii-1][jj-1][1];
					mm++;}}

			// Calculate solution and save results
			LaLinearSolve(A,x,b);
			for(mm=0;mm<3;mm++){
				scanGridFit[ii-1][jj-1][mm] = x(mm);}

			// Calculate and save error value for this solution
			LaVectorDouble bPrime(numGridPoints*2);
			Blas_Mat_Mat_Mult(A,x,bPrime,1.f,-1.f);
			Blas_Add_Mult(bPrime,-1,b);
			scanGridFit[ii-1][jj-1][3] = Blas_Norm1(bPrime)/(numGridPoints*2);
			//if(ii==400 && jj==300){cout << "\n" <<b << "\n\n" << bPrime << "\n\n" << scanGridFit[ii-1][jj-1][3] << "endl";}
	}}

	// Write out grid fit data in simple gnuplot format
	float gridFitErrorRatioThreshold = 0.5;
	FILE * scanGridFitData = fopen("flowGridFitData.txt","w"); fprintf(scanGridFitData,"#X			Y\n");
	for(ii=1; ii<=imageSize.width; ii+=diagnosticStepSize){
		for(jj=1; jj<=imageSize.height; jj+=diagnosticStepSize){
			if(scanGridFit[ii-1][jj-1][3]/scanGridFit[ii-1][jj-1][2] < gridFitErrorRatioThreshold){
				for(kk=0; kk<(gridRange[1]-gridRange[0]+1); kk++){
					for(ll=0; ll<(gridRange[3]-gridRange[2]+1); ll++){
						fprintf(scanGridFitData,"%.2f\t%.2f\n", scanGridFit[ii-1][jj-1][0]+scanGridFit[ii-1][jj-1][2]*(kk), scanGridFit[ii-1][jj-1][1]+scanGridFit[ii-1][jj-1][2]*(ll));}}}}}
	fclose(scanGridFitData);

	// Write out error data in simple gnuplot format
	FILE * scanGridFitErrorData = fopen("flowGridFitErrorData.txt","w"); fprintf(scanGridFitErrorData,"#X\n");
	for(ii=1; ii<=imageSize.width; ii++){
		for(jj=1; jj<=imageSize.height; jj++){
			if(scanGridFit[ii-1][jj-1][3]/scanGridFit[ii-1][jj-1][2] < 1000){ //Filter out invalid data
				fprintf(scanGridFitErrorData,"%.2f\n", scanGridFit[ii-1][jj-1][3]/abs(scanGridFit[ii-1][jj-1][2]));}}}
	fclose(scanGridFitData);

/*
	CALCULATE IMAGE POINT POSITIONS IN 3D-----
*/

	// Initialize 3DPoint matrix
	float *** pointsIn3D = new float ** [imageSize.width];
	for(ii=1; ii<=imageSize.width; ii++){
		pointsIn3D[ii-1] = new float * [imageSize.height];
		for(jj=1; jj<=imageSize.height; jj++){
			pointsIn3D[ii-1][jj-1] = new float[3];
			for(kk=0;kk<6;kk++){
				pointsIn3D[ii-1][jj-1][kk] = -1;}}}
	
	// Try intializing ae_matrix
	float gridSpacing = 5.29;
	ae_matrix boardXCoords, boardYCoords, boardSg;
	ae_state _state;
	ae_matrix_init(&boardXCoords, numCorners, 3, DT_REAL, &_state, ae_true);
	ae_matrix_init(&boardYCoords, numCorners, 3, DT_REAL, &_state, ae_true);
	ae_matrix_init(&boardSg, numCorners, 3, DT_REAL, &_state, ae_true);

	// Try filling ae_matrix with corner values
	for(ii=1; ii<=boardSize.width; ii++){
		for(jj=1; jj<=boardSize.height; jj++){
			// X-Coords matrix
			boardXCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][0] = corners[centralImageX-1][centralImageY-1][ii-1][jj-1][0];
			boardXCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][1] = corners[centralImageX-1][centralImageY-1][ii-1][jj-1][1];
			boardXCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][2] = (ii-1)*gridSpacing;
			// Y-Coords matrix
			boardYCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][0] = corners[centralImageX-1][centralImageY-1][ii-1][jj-1][0];
			boardYCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][1] = corners[centralImageX-1][centralImageY-1][ii-1][jj-1][1];
			boardYCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][2] = (jj-1)*gridSpacing;
			// Sg matrix
			boardSg.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][0] = corners[centralImageX-1][centralImageY-1][ii-1][jj-1][0];
			boardSg.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][1] = corners[centralImageX-1][centralImageY-1][ii-1][jj-1][1];
			boardSg.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][2] = calibrationGridFit[ii-1][jj-1][2];


	// Test the filling...
	//cout << boardXCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][0] << " " << boardXCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][1] << " " << boardXCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][2] << " " << boardYCoords.ptr.pp_double[(ii-1)+(jj-1)*boardSize.width][2] << endl;
}}
			
	// Try making an ALGLIB IDW interpolant -- Nw & Nq of 16 seems incorrect given domain knowledge
	idwinterpolant IDWInterpolantX, IDWInterpolantY, IDWInterpolantSg;
	_idwinterpolant_init(&IDWInterpolantX, &_state, ae_true);
	_idwinterpolant_init(&IDWInterpolantY, &_state, ae_true);
	_idwinterpolant_init(&IDWInterpolantSg, &_state, ae_true);
	idwbuildmodifiedshepard(&boardXCoords,numCorners,2,2,16,9,&IDWInterpolantX,&_state);
	idwbuildmodifiedshepard(&boardYCoords,numCorners,2,2,16,9,&IDWInterpolantY,&_state);
	idwbuildmodifiedshepard(&boardSg,numCorners,2,2,16,9,&IDWInterpolantSg,&_state);

	// Try interpolating a point
	ae_vector interpPoint;
	ae_vector_init(&interpPoint, 2, DT_REAL, &_state, ae_true);
	interpPoint.ptr.p_double[0] = 779;
	interpPoint.ptr.p_double[1] = 579;

	printf("Point has real-world coordinates (%.2f,%.2f) and Sg %.2f\n",idwcalc(&IDWInterpolantX,&interpPoint,&_state),idwcalc(&IDWInterpolantY,&interpPoint,&_state),idwcalc(&IDWInterpolantSg,&interpPoint,&_state));

	// Calculate image center's position in calibration coordinate system
	float imageCenterCoords[2];
	interpPoint.ptr.p_double[0]=(double)(imageSize.width+1)/2.0; interpPoint.ptr.p_double[1]=(double)(imageSize.height+1)/2.0;
	imageCenterCoords[0]=idwcalc(&IDWInterpolantX,&interpPoint,&_state);
	imageCenterCoords[1]=idwcalc(&IDWInterpolantY,&interpPoint,&_state);
	printf("Image center coordinates: (%.2f,%.2f)\n",imageCenterCoords[0],imageCenterCoords[1]);

	// Calculate 3D position of each viable image pixel -- assume openCV takes top left pixel as (1,1) not (0,0)
	float calTargetDistance = 1000; //mm
	float tempInterp[3];
	for(ii=1; ii<=imageSize.width; ii++){
		for(jj=1; jj<=imageSize.height; jj++){
			if(scanGridFit[ii-1][jj-1][3]/scanGridFit[ii-1][jj-1][2] < gridFitErrorRatioThreshold){
					//Get raw interpolations
					interpPoint.ptr.p_double[0] = (double)ii; interpPoint.ptr.p_double[1] = (double)jj;
					tempInterp[0] = idwcalc(&IDWInterpolantX,&interpPoint,&_state);
					tempInterp[1] = idwcalc(&IDWInterpolantY,&interpPoint,&_state);
					tempInterp[2] = idwcalc(&IDWInterpolantSg,&interpPoint,&_state);
					// Caluclate Z (with crappy linear model)
					pointsIn3D[ii-1][jj-1][2] = calTargetDistance + 2*(tempInterp[2]-scanGridFit[ii-1][jj-1][2]);
					// Calculate X & Y accounting for image-centering [Xcalculated = (XCal-XimageCenter)*Zcalibtarget/Zcalculated]
					pointsIn3D[ii-1][jj-1][0] = (tempInterp[0]-imageCenterCoords[0])*calTargetDistance/pointsIn3D[ii-1][jj-1][2];
					pointsIn3D[ii-1][jj-1][1] = (tempInterp[1]-imageCenterCoords[1])*calTargetDistance/pointsIn3D[ii-1][jj-1][2];
					// Print progress
					if(int((ii*imageSize.height+jj)/500)*500 == ii+jj*imageSize.width){
						printf("\rCalculating Points in 3D: %.1f%%",100*(float(ii*imageSize.height+jj)/float(imageSize.width*imageSize.height))); fflush(stdout);}
				}}}
	cout << "\n";

	// Count number of "valid" points for use in PLY file
	int counter = 0; Vec3b pixelColor;
	for(ii=1; ii<=imageSize.width; ii++){
		for(jj=1; jj<=imageSize.height; jj++){
			if(scanGridFit[ii-1][jj-1][3]/scanGridFit[ii-1][jj-1][2] < gridFitErrorRatioThreshold && pointsIn3D[ii-1][jj-1][2]<1300 && pointsIn3D[ii-1][jj-1][2]>700){counter++;}}}
	printf("Found %i valid 3D points.\n",counter);
	// Write out 3D point data in PLY format
	FILE * point3DData = fopen("points3D.ply","w"); fprintf(point3DData,"ply\nformat ascii 1.0\nelement vertex %i\nproperty float x\nproperty float y\nproperty float z\nproperty uchar diffuse_red\nproperty uchar diffuse_green\nproperty uchar diffuse_blue\nend_header\n",counter); //PLY header w/ point count
	for(ii=1; ii<=imageSize.width; ii++){
		for(jj=1; jj<=imageSize.height; jj++){
			if(scanGridFit[ii-1][jj-1][3]/scanGridFit[ii-1][jj-1][2] < gridFitErrorRatioThreshold && pointsIn3D[ii-1][jj-1][2]<1300 && pointsIn3D[ii-1][jj-1][2]>700){ //Filter out invalid data
				pixelColor = view1.at<Vec3b>(jj-1,ii-1);//Sample original reference image 'view1' (BGR format, backward indices??)
				fprintf(point3DData,"%f %f %f %i %i %i\n",pointsIn3D[ii-1][jj-1][0],pointsIn3D[ii-1][jj-1][1],pointsIn3D[ii-1][jj-1][2],pixelColor.val[2],pixelColor.val[1],pixelColor.val[0]);}}}
	fclose(point3DData);

//(int)pixelColor.val[2],(int)pixelColor.val[1],(int)pixelColor.val[0]



	// Report some information and exit
	printf("Found %i (%3.5g%%) of %i expected corner points.\n",numCorners,float(numCorners)/float(expectedNumCorners)*100.0,expectedNumCorners);
	return 0;
}
