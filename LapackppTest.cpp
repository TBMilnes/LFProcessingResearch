#include <stdio.h>
#include <lapackpp/laslv.h>

using namespace std;
//using namespace NR;


int main(){
	int ii,jj;

	float XValues[25] = { 136.94, 136.64, 136.35, 136.24, 137.16, 140.88, 140.44, 140.33, 140.38, 141.13, 145.05, 144.87, 144.39, 144.75, 145.28, 148.67, 148.81, 149.01, 148.88, 149.39, 152.2, 152.3, 152.37, 152.15, 152.43};

	float YValues[25] = {306.32, 309.01, 312.5, 315.02, 317.54, 305.72, 309.08, 312.31, 315.63, 318.16, 305.83, 308.71, 312.42, 315.93, 318.41, 306.33, 309.08, 312.46, 315.49, 318.43, 307.25, 309.55, 312.4, 315.4, 318.14};

	// Define "x" matrix
	LaVectorDouble x(3);

	// Define "b" matrix
	LaVectorDouble b(50);
	for(ii=0;ii<25;ii++){
		b(ii) = XValues[ii];
		b(ii+25) = YValues[ii];}

	// Define "A" ("C") Matrix
	LaGenMatDouble A(50,3);
	for(ii=0;ii<25;ii++){//1*Ox
		A(ii,0) = 1;
		A(ii,1) = 0;}
	for(ii=25;ii<50;ii++){//1*Oy
		A(ii,0) = 0;
		A(ii,1) = 1;}
	for(ii=0;ii<5;ii++){//Sg coefficients for X values
		for(jj=0;jj<5;jj++){
			A(ii*5+jj,2) = ii;}}
	for(ii=0;ii<5;ii++){//Sg coefficients for Y values
		for(jj=0;jj<5;jj++){
			A(25 + ii*5+jj,2) = jj;}}
		
	// Display "A" ("C") matrix
	cout<<"Matrix A:\n";
	for(int i=0; i<50; i++){
		for(int j=0; j<3; j++){
			cout<<A(i,j)<<"\t";}
		cout<<endl;}

	// Calculate solution and display results
	LaLinearSolve(A,x,b);
	cout<<"Ox = "<<x(0)<<"\n";
	cout<<"Oy = "<<x(1)<<"\n";
	cout<<"Sg = "<<x(2)<<"\n";

	// Print out gnuplot-readable file
	FILE * gnuplotData = fopen("LapackppTestOutput.txt","w"); fprintf(gnuplotData,"#X\t\t\tY\n");
		for(ii=0;ii<5;ii++){
			for(jj=0;jj<5;jj++){
				fprintf(gnuplotData,"%.2f\t%.2f\t%.2f\t%.2f\n",XValues[ii*5+jj],YValues[ii*5+jj],x(0)+x(2)*ii,x(1)+x(2)*jj);}}
	fclose(gnuplotData);

	return 0;
}


	/*
	LaGenMatDouble A(2,2);
	LaVectorDouble x(2), b(2);

	A(0,0)=1;
	A(0,1)=2;
	A(1,0)=3;
	A(1,1)=4;

	b(0)=5;
	b(1)=6;
	*/


/*
X			Y
136.94	306.32
136.64	309.01
136.35	312.50
136.24	315.02
137.16	317.54
140.88	305.72
140.44	309.08
140.33	312.31
140.38	315.63
141.13	318.16
145.05	305.83
144.87	308.71
144.39	312.42
144.75	315.93
145.28	318.41
148.67	306.33
148.81	309.08
149.01	312.46
148.88	315.49
149.39	318.43
152.20	307.25
152.30	309.55
152.37	312.40
152.15	315.40
152.43	318.14
*/
