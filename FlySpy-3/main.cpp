/*FlySpy-3
	Developed @ Tavaré Research Labs, USC.
	Platform: Windows

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
	DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
	SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

	YOU MUST CITE THE DEVELOPERS AND REFERENCE GOOGLE CODE PROJECT LINK IN ANY ACADEMIC WORK WHERE YOU HAVE USED THIS SOFTWARE.

	ANY KIND OF COMMERCIAL / FOR - PROFIT USE OF THIS SOFTWARE IS PROHIBITED WITHOUT PRIOR CONFIRMATION FROM DEVELOPERS.
	For Citation please cite: 
	@article{ardekani2012three,
	  title={Three-dimensional tracking and behaviour monitoring of multiple fruit flies},
	  author={Ardekani, Reza and Biyani, Anurag and Dalton, Justin E and Saltz, Julia B and Arbeitman, Michelle N and Tower, John and Nuzhdin, Sergey and Tavar{\'e}, Simon},
	  journal={Journal of The Royal Society Interface},
	  pages={rsif20120547},
	  year={2012},
	  publisher={The Royal Society}
	}



*/

// Sample command line parameters (add in Project properties -> Debugging -> Command Line) : 3 DFS d:\vg\a\2DCentroid-View0.txt d:\vg\long_exp\projmatrix2.txt d:\vg\a\TrackFile.txt
#include "Experiment.h"

using namespace std;

extern int nFlies;
extern int nViews;
////////////////////////////////////// Selective compilation directives below ////////////////////////////
#define __VERBOSE 0
#define __SAVE 0 // For saving the output video file or not

#define __CM_DISPLAY 0// For displaying changemask images
#define __DISPLAY 1 // For turning ON/OFF display

#define __3DTRACKING 1 // To start and display 3D tracking

#define __WRITE2DCENTROID 1

#define MINIMUM_VIEWS_FOR_TRACKING (int(e.V.size())-1) // Right now we track if we have less than one view than total views 

// Flags for 2D centroid file
// //Current stucture for line in 2D centroid file is
//{<Frame Number-integer>,<Flag-integer value>,<Quality score - double value>,<X1-double>,<Y1-double>,<X2>,<Y2>,.....} where Xi,Yi = X and Y coordinates of i-th centroid
#define CENTROID_COMPUTER_GENERATED 0
#define CENTROID_HUMAN_FIXED 1
#define CENTROID_EXTRAPOLATED 2

const double correspondenceErrorThresholdForFlagging = 13.0;

enum en_CorrespondenceMethod
{
	STABLE_POLYAMORY,
	DFS,
	HUNGARIAN
}CorrespondenceMethod;

void printUsage()
{
	printf("\nFlySpy usage help : \n");
	printf("\nFlySpy-3 <Num_Flies> <Num_Views> <INP_METHOD=AVI|CENTROID> <HUNGARIAN|DFS|STABLE_POLYAMORY> <Path for first input file> \
			 <Path for projection matrix file> <prefix for all outputfiles> <First_Frame_to_process_index> <Last_num_frames_to_process>");

	getchar();
}


int main(int argc,char *argv[])
{
	Experiment e; // The main experiment class object - all action takes place thru this

	char inpCentroidFileName[10000] = {0}; // For input = Centroid case
	char projMatFilePath[10000] = {0}; // Projection matrix file - to be given in either case: simulated data or real video

	int lastFramesToProcessIndex = -1;
	int startFrameToProcessIndex = 1;
	string prefixForAllTracks="HUNG_VISUAL_STUDIO"; // All files will be appended with this string (technically a suffix .. but named it already - who cares! :D )

	// File names/paths
	string outputTrackFileName; // This is the final file for 3D tracks
	string outputForCorrespondenceMapFileName; // Correspondence (the complete map) is written in this file for each frame
	string outputErrorFileName; // Error value returned by corresponder 
	char inpAVIName[10000]; // Name of input AVI file - one of the first variables, so, is a char array (only later in project did I start using "string" - should be changed some fine day!)

	// FILE handler to output files
	FILE *fp_CorrespondenceMap = NULL, *fp_ErrorOut=NULL;
	FILE *fp_OutTrackFile; // For saving output 3D tracks

	FILE *fp_OutTotalTimeTakenInProcessing; // For saving total time taken - to compute FPS;
	string outputTotalTimeTakenFileName;
	/////////////////// Input file names/window names, etc ////////////////////////////////
	char outCentroidFileName[10000]="Out_2DCentroid_View0.txt"; // Name of ourpur 2D centroid file for 1st View (View 0) - will be created for other views accordingly 
	

	// The parameter below is used for analyzing efficiency in simulated data (for CVPR paper).
	double Threshold_For_False_Track = INFINITY_ALEPH1; // If the computed 3D point is at a distance greater than this threshold, then discard that point  - For simulated data (CVPR)

	if(argc == 1) // Default case: No command line arguments
	{
		//CorrespondenceMethod = HUNGARIAN; // STABLE_POLYAMORY
		//e.setNFlies(4); // Set number of flies - VERY IMPORTANT
		//e.setNViews(4); // Set number of views - VERY IMPORTANT
		//e.setInputType(View::AVI_ONLY); // Input type for processing - VERY IMPORTANT

		//// If a 2D centroid file is already present, then uncomment line below and set the path
		//strcpy(inpAVIName, "C:\\Users\\rdehestani\\Downloads\\sampleData-20170128T011500Z\\sampleData\\input\\AviFileChunk0_View2.avi");

		////////// Projection Matrix file name - required in all case
		//// Right now projection matrix for a particular view is decided by actual camera number(by reading last character of file name before '.')
		////(so no need to rename video files) - Just supply the original projection matrix file (even if not all views are used in FlySpy, ex., high reoslution cameras)

		//strcpy(projMatFilePath, "C:\\Users\\rdehestani\\Downloads\\sampleData-20170128T011500Z\\sampleData\\input\\projMat.txt");
		//outputTrackFileName = "TrackFile.txt";
		//outputTotalTimeTakenFileName = "running_time_analysis.txt";
		printUsage();
		return 0;
	}

	else // Process command line params !! 
	{
		if(argc!=10)
		{
			std::cout << "input parameters are not sufficient/specified, see below:" << std::endl;
			printUsage();
			return 0;
		}

		int param1 = atoi(argv[1]); // Number of flies;
		e.setNViews(atoi(argv[2])); // Number of views
		if(param1<=0 || param1 >= 100000)
		{
			printf("\nInavlid value (%d) for num_flies",param1);
			printUsage();
			return 0;
		}
		if(nViews<=0 || nViews >= 100000)
		{
			printf("\nInavlid value (%d) for num_flies",nViews);
			printUsage();
			return 0;
		}
		printf("\nFlySpy-3 <Num_Flies> <Num_Views> <INP_METHOD=AVI|CENTROID> <HUNGARIAN|DFS|STABLE_POLYAMORY> <Path for first input file> \
				 <Path for projection matrix file> <prefix for all outputfiles> <first_frame_to_process_index> <last__frames_to_process> [<Ground Truth 3D TrackFile> <Threshold> (only for AVI case)]");

		if(_strcmpi(argv[3],"AVI") == 0)
		{
			e.setInputType(View::AVI_ONLY);
			printf("\nWill use AVI file as input .....");
		}
		else
		{
			if(_strcmpi(argv[3],"CENTROID") == 0)
			{
				e.setInputType(View::CENTROID_ONLY);
				printf("\nWill use Centroid file as input .....");
			}
			else
			{
				printf("\nInvalid INP_METHOD type = %s\n\n",argv[3]);
				printUsage();
				return 0;
			}
		}
		e.setNFlies(param1);
		// Process second parameter == Method for correspondence
		if(_strcmpi(argv[4],"HUNGARIAN") == 0)
		{
			CorrespondenceMethod = HUNGARIAN;
		}
		else
		{
			if(_strcmpi(argv[4],"DFS") == 0)
			{
				CorrespondenceMethod = DFS;
			}
			else
			{
				if(_strcmpi(argv[4],"STABLE_POLYAMORY")==0)
				{
					CorrespondenceMethod = STABLE_POLYAMORY;
				}
				else
				{
					printf("\nInvalid Correspondence method = %s",argv[4]);
					printUsage();
					return 0;
				}
			}
		}

		// Process next 3 params - centroid file name, proj mat file name, trackfile name
		if(_strcmpi(argv[3],"AVI") == 0)
			strcpy(inpAVIName,argv[5]);
		else
			strcpy(inpCentroidFileName,argv[5]);

		strcpy(projMatFilePath,argv[6]);

		////// Create string for output Files ////////
		prefixForAllTracks = argv[7];

		startFrameToProcessIndex = atoi(argv[8]); // First frame to process
		lastFramesToProcessIndex = atoi(argv[9]); // Last frame to process
		if(lastFramesToProcessIndex < 1) // Use default value for absurd case
			lastFramesToProcessIndex = -1;
		string temp = "Out_2DCentroid_"+prefixForAllTracks+"_View0.txt";
		strcpy(outCentroidFileName,temp.c_str());
	}

	// Now create file names and open all output files
	outputTrackFileName = "Out_TrackFile_" + prefixForAllTracks + ".txt";
	outputForCorrespondenceMapFileName = "Out_CorrespondenceMap_"+ prefixForAllTracks + ".txt";
	outputErrorFileName = "Out_Error_" + prefixForAllTracks + ".txt";
	fp_CorrespondenceMap = fopen(outputForCorrespondenceMapFileName.c_str(),"w");
	fp_ErrorOut = fopen(outputErrorFileName.c_str(),"w");
	outputTotalTimeTakenFileName = "running_time_analysis_"+prefixForAllTracks + ".txt";
	fp_OutTotalTimeTakenInProcessing = fopen(outputTotalTimeTakenFileName.c_str(),"w");

	if(!(fp_CorrespondenceMap && fp_ErrorOut))
	{
		printf("\nCould not create output correspondence ( %s ) or ErrorFile( %s )",outputForCorrespondenceMapFileName.c_str(),outputErrorFileName.c_str());
		return 0;
	}

	if( fopen_s(&fp_OutTrackFile,outputTrackFileName.c_str(),"w") !=0 )
	{
		printf( "\nThe output file %s could not be created\n", outputTrackFileName);
		getchar();
		return 0;
	}

	vector<string> wname; // Name for AVI output opencv window
	vector<string> cname; // Change mask opencv windows
	for(int countView=0;countView<nViews;countView++) // Initialize name for OpenCV windows (one for each view)
	{
		char num[100];
		sprintf(num,"%d",countView);
		string num_str(num);
		wname.push_back("File-"+num_str);
		cname.push_back("ChangeMask-File"+num_str);
	}
	//////////////////// Initialization /////////////////////////


#if __WRITE2DCENTROID
	vector<FILE*> fp_OutCentroid(nViews); // Create a file handler for output 2D centroid file for each view - nViews had been set by call to e.SetNViews() above

	for(int i=0;i<fp_OutCentroid.size();i++)
	{
		if(fopen_s(&fp_OutCentroid[i],outCentroidFileName,"w")!=0)
		{
			printf("\nError creating centroid file %s ",outCentroidFileName);
			getchar();
			return 0;
		}
		outCentroidFileName[strlen(outCentroidFileName)-5]++; // To create next file name: 2DCentroid-View0.txt -> 2DCentroid-View1.txt -> 2DCentroid-View2.txt ...and so on
	}
#endif

	// Set properties for each view
	/* Right now if any error occurs in this loop, the program just stops (not cleanly) - and it very well might not release the resources
	allocated to it (for example opened AVI Captures are not released, etc). - Should be changed if restrcturing code sometime. */
	for(int countView=0;countView<nViews;countView++)
	{

		e.addView();
		e.V[countView]->viewID = countView;

		int curr_cam_num=-1; // To identify which projection matrix entry should be used for particular AVI file
		// The method below used for identifying "curr_cam_num" is to use the digit just before .avi (or .txt) in the file name. For ex, if file name: AVIFileCam0.avi ...
		// ... then curr_cam_num = 0 , if file name: 2dCentroidView3.txt, then curr_cam_num = 3; This technique obviously does not work for #views > 9. 
		switch(e.inpType) // Must be called afrer addView();
		{
		case View::AVI_ONLY:
			curr_cam_num = inpAVIName[strlen(inpAVIName)-5]-'0';
			e.setInputAVIFileName(countView,inpAVIName);
			break;
		case View::CENTROID_ONLY:
			curr_cam_num = inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0';
			e.setInputCentroidFileName(countView,inpCentroidFileName);
			break;
		case View::AVI_AND_CENTROID:
			assert(inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0' == inpAVIName[strlen(inpAVIName)-5]-'0');
			curr_cam_num = inpCentroidFileName[strlen(inpCentroidFileName)-5]-'0';
			e.setInputAVIFileName(countView,inpAVIName);
			e.setInputCentroidFileName(countView,inpCentroidFileName);
			break;
		}
		
		if(curr_cam_num<0 || curr_cam_num >9) // Valid digit is NOT extracted - NOTE: this scheme doesn't work if number of views > 10.
		{
			printf("\nCamera number \"%d\" read from file name: \"%s\" is not valid",curr_cam_num,inpAVIName);
			printf("\nPlease check the file name (the last character before '.' in file name should be a valid camera number (does not work for view > 10 (0-9) )\n");
			getchar();
			return 0; // Unclean exit (previously allocated resources are not released). - BAD practise :D
		}
		CvMat* temp = NULL;

		temp = readProjMatrixFromFile(projMatFilePath,curr_cam_num);
		if(temp==NULL)
		{
			printf("\nCannot open Projection Matrix file @ %s\nAborting...",projMatFilePath);
			getchar();
			return 0; // Unclean exit (previously allocated resources are not released). - BAD practise :D
		}
		e.V[countView]->setProjMatrix(temp);

		cvReleaseMat(&temp);

		if(!e.openView(countView,startFrameToProcessIndex,lastFramesToProcessIndex))
		{
			printf("\nError opening input files: one or both from -> %s , %s",inpAVIName,inpCentroidFileName);
			getchar();
			cvDestroyAllWindows();
			return 0;
		}
		inpAVIName[strlen(inpAVIName)-5] += 1; // Increase to next file name
		inpCentroidFileName[strlen(inpCentroidFileName)-5] +=1;

#if __DISPLAY
		cvNamedWindow(wname[countView].c_str());
#endif
#if __CM_DISPLAY
		cvNamedWindow(cname[countView].c_str());
#endif
	} // End for Loop

	assert(nViews == e.V.size()); // These values should match(just a sanity check), because we use them interchangeably. BAD things will happen if they do not ;)

	char video_output_fn[1000];

	switch(CorrespondenceMethod)
	{
	case STABLE_POLYAMORY:
		strcpy(video_output_fn,"Out_STABLE_POLY_");
		break;
	case HUNGARIAN:
		strcpy(video_output_fn,"Out_HUNGARIAN_");
		break;
	case DFS:
		strcpy(video_output_fn,"Out_DFS_");
		break;
	}

#if __SAVE
	assert(e.inpType!=View::CENTROID_ONLY); // Should not happen, since __SAVE makes no sense for centroid_only case
	printf("\nCurrently recording Change Masks (remeber to udpate View class for normal recording again)\n");
	int outVideoWidth=640,outVideoHeight=480;
	if(e.V[0]->AVI_In!=NULL) // If View file has been opened, then get dimension from view file (otherwise assume it to be default (640x480))
	{
		outVideoWidth = (int)cvGetCaptureProperty(e.V[0]->AVI_In,CV_CAP_PROP_FRAME_WIDTH);
		outVideoHeight = (int)cvGetCaptureProperty(e.V[0]->AVI_In,CV_CAP_PROP_FRAME_HEIGHT);
	}

	if(e.initSavingToFile(video_output_fn,cvSize(outVideoWidth,outVideoHeight),60)==false)
	{
		printf("\nCould not start writing ... ");
		getchar();
	}
#endif


	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////// Real processing starts after this point /////////////////////////////////
	int FIndex; // Starting frame number (initilized in for loop below)
	long time_start = clock();
	long f_processed=0; // total number of frames processed (for calculating cumulative processing FPS - "speed of program")


	FILE *fp_numCentroids;
	fp_numCentroids = fopen("numCentroids.csv","w"); // Keep track of how many centroids are detected in each view in each frame (just a quick thingy - need to move up if we are going to use it consistently)

	int countScrollPressed = 0; // Keeps the count of how many time SCROLL_LOCK is pressed - for ending program prematurely.

	// these 4 variables are used for finding stats about effect of doing correspondence of all pair of sequences
	int num_corr_sequences=0;
	int num_unique_initial_corr_sequence = 0;
	int total_mismatch=0,total_MinimumSequences=0;
	////////////////////////////////////////////////////////

	int lastTimeHowManyFlies=0; // Keep note of how many flies were tracked in last frame - used for FLAGGING frames
	
	time_t start_time = time(NULL);
	int howManyFramesCorresponded=0;
	int howManyFramesTracked=0;
	int howManyFrames_Grab_and_SilhDetected=0;
	clock_t total_clocks_in_Grab_And_SilhDetection = 0;
	clock_t total_clocks_corresponder=0;
	clock_t total_clocks_tracker=0;

	clock_t clock_start_grabbing; // needs to be set to clock() before we grab a frame;
	for(FIndex = startFrameToProcessIndex;(clock_start_grabbing = clock()) && e.grabFrame((unsigned long)FIndex)  && (lastFramesToProcessIndex < 0 || FIndex < lastFramesToProcessIndex) ;FIndex++) // && FIndex < n => will process only frames upto "n"
	{
		howManyFrames_Grab_and_SilhDetected++;
		if(GetAsyncKeyState(VK_SCROLL)) // will be true if "scroll lock" is pressed (unfortunately in any program - even in background, therefore a buffer of 4 time before exiting)
		{
			if(countScrollPressed<4)
				countScrollPressed++;
			else
				break;
		}

		long curr_time = clock();
		vector<int> allFliesVisibleViewList;
		allFliesVisibleViewList = e.processNewFrame(); // Runs Silh detetcion, and stores list of view with exactly "nFlies" blobs

		//printf("\n\n############## nFlies = %d, Grabbing frame  = %d, processing frame = %ld ############\n",nFlies,FIndex,e.currFrameNumberProcessed);
#if __VERBOSE
		printf("\n\n++++++++++++++ nFlies = %d, # Frame grabbed  = %d  +++++++++++++++",nFlies,FIndex);
#else
		if (FIndex % 200 == 0)
			printf("\n\n++++++++++++++ nFlies = %d, # Frame grabbed  = %d  +++++++++++++++",nFlies,FIndex);
#endif

#if __WRITE2DCENTROID
		/* Write 2D centroids to file if flag is ON */
		for(int i=0;i<e.V.size() && e.currFrameNumberProcessed!=-1L;i++)
		{
			double quality_score = 1.0; // This value should ideally be spitted out by SilhDetector for each view. Currently a dummy value
			fprintf(fp_OutCentroid[i],"%d,%d,%.2lf,",(int)e.currFrameNumberProcessed,(int)CENTROID_COMPUTER_GENERATED,quality_score);
			for(int j=0;j<e.V[i]->origCentroidsByCentroidCalculator.size();j++)
				fprintf(fp_OutCentroid[i],"%.2lf,%.2lf,",e.V[i]->origCentroidsByCentroidCalculator[j].x,e.V[i]->origCentroidsByCentroidCalculator[j].y);

			fprintf(fp_OutCentroid[i],"\n");

		}
#endif

		fprintf(fp_numCentroids,"\n%d", FIndex);		
		for (int i = 0;i<e.V.size();i++)	
			if(e.V[i]->centroidCalculator.windowContoursOriginalCentroids.size()>2)
				fprintf(fp_numCentroids,",%d,%d",e.V[i]->centroidCalculator.windowContoursOriginalCentroids[1].size(),e.V[i]->centroidCalculator.windowCentroids[1].size());

#if __3DTRACKING
		int corr_mismatch=0,corr_MinimumSequences=0;
		total_clocks_in_Grab_And_SilhDetection += clock()-clock_start_grabbing;

		if(allFliesVisibleViewList.size()>=MINIMUM_VIEWS_FOR_TRACKING) // Atleast 2 views should have == nFlies measurment to do something meaningful .. else just skip to next frame
		{	
			clock_t clock_start_corresponder = clock();
			howManyFramesCorresponded++;

			switch(CorrespondenceMethod)
			{
			case STABLE_POLYAMORY:
				e.correspondViewsUsingStablePolyamory(allFliesVisibleViewList,corr_mismatch,corr_MinimumSequences);
				break;
			case HUNGARIAN:
				e.correspondViewsUsingHungarian(allFliesVisibleViewList,corr_mismatch,corr_MinimumSequences);
				break;
			case DFS:
				e.correspondViewsUsingDFS(allFliesVisibleViewList);
				break;
			}

			num_corr_sequences += factorial(allFliesVisibleViewList.size());
			num_unique_initial_corr_sequence += ( ( allFliesVisibleViewList.size()*(allFliesVisibleViewList.size()-1) )/2);

			total_MinimumSequences += corr_MinimumSequences;
			total_mismatch += corr_mismatch;
#if __VERBOSE
			printf("\nTotal Min seq = %.5lf %%\nTotal Mismatches in initial same views = %.5lf %%",\
				(double(total_MinimumSequences)/(num_corr_sequences))*100.0,(double(total_mismatch)/num_unique_initial_corr_sequence)*100.0);
#endif

			e.clearCorresponderOutput(); // Strip off any dummy points
#if __VERBOSE
			printf("\nReal error (unscaled, but, without dummy points) = %.3lf",e.computeTotalErrorForOutput(e.correspondOutput,false));
#endif
			
			double err = e.computeTotalErrorForOutput(e.correspondOutput,false);
			// To scale the value (so that it is no longer dependent on number of flies). the factor, (e.V.size()*(e.V.size()-1)/2)  = #of views chose 2 (vC2)
			if(allFliesVisibleViewList.size() >1)
				err /= ( double((int(allFliesVisibleViewList.size()) *( int(allFliesVisibleViewList.size())-1 ))/2.0) * double(nFlies) );
			bool corr_flag = false;
			if(err > correspondenceErrorThresholdForFlagging)
				corr_flag = true;

			vector<Point3D> out3D;
			int howManyTrackedInThisFrame=0; // How many flies are actually tracked in this frame - used for flagging frames
			total_clocks_corresponder += clock()-clock_start_corresponder;

			clock_t clock_start_tracker = -1l;
			if((e.howManyCentroidsCorresponded>=nFlies-1 && nFlies!=1) || (e.howManyCentroidsCorresponded==nFlies))
			{	
				clock_start_tracker = clock();
				out3D = e.Track3D();
				for(int countFlies=0;countFlies<out3D.size();countFlies++)
					if(!isInvalidPoint(out3D[countFlies]))
						howManyTrackedInThisFrame++;

				if(e.inpType!=View::CENTROID_ONLY) // A possible source of vector subscript out of range errors (was earlier trying to write in case of centorid only - fatal error)
					e.draw3DPoints(out3D);
			}
			if(out3D.size()>0)
			{
				fprintf(fp_OutTrackFile,"%d,",(corr_flag==false && howManyTrackedInThisFrame==lastTimeHowManyFlies) ? 0 : 1); // If number of lfies tracked changes, then flag this frame
				lastTimeHowManyFlies = howManyTrackedInThisFrame;
				fprintf(fp_OutTrackFile,"%d,",(int)e.currFrameNumberProcessed);
				for(int i=0;i<out3D.size();i++)
				{
					//printf("%.2lf,%.2lf,%.2lf\n",out3D[i].x,out3D[i].y,out3D[i].z);
					if(i<int(out3D.size())-1)
						fprintf(fp_OutTrackFile,"%.2lf,%.2lf,%.2lf,",out3D[i].x,out3D[i].y,out3D[i].z);
					else
						fprintf(fp_OutTrackFile,"%.2lf,%.2lf,%.2lf",out3D[i].x,out3D[i].y,out3D[i].z);
				}
				fprintf(fp_OutTrackFile,"\n");
			}
			//Write correspondence error to file

			/////////// Print out the SortedOutputFile => only make sense for simulated data (checking correspondence) //////

			// Check if there are any dummy 3D points (in case of simulated data it will happen only for first 3 frames - when we do not track).
			bool hasDummy3DPoint = false;
			for(int count3DPoints = 0;count3DPoints<out3D.size() && !hasDummy3DPoint;count3DPoints++)
				if(isInvalidPoint(out3D[count3DPoints]))
					hasDummy3DPoint = true;

			fprintf(fp_ErrorOut,"%d,%.3lf\n",(int)e.currFrameNumberProcessed,err); // Write the scaled correspondence error.
			map<FlyType,vector<ViewCentroidPair> >::iterator it = e.correspondOutput.begin();
			fprintf(fp_CorrespondenceMap,"\n%d,Err = %.2lf,",(int)e.currFrameNumberProcessed,err);
			for(int i=0;it!=e.correspondOutput.end();i++,it++)
			{
				fprintf(fp_CorrespondenceMap,"FT %d,",i);
				for(int j=0;j<(*it).second.size();j++)
					fprintf(fp_CorrespondenceMap,"{V= %d,C= %d},",(*it).second[j].first,(*it).second[j].second);
			}
			
			if(clock_start_tracker>=0l)
			{
				total_clocks_tracker += clock()-clock_start_tracker;
				howManyFramesTracked++;
			}
		}
		else
		{
			lastTimeHowManyFlies=0; // Nothing was tracked in this frame
		}
#endif

#if __SAVE
		e.appendToOutputFile();
#endif
		vector<IplImage*>change_mask_img(e.V.size(),NULL);

#if __CM_DISPLAY || __DISPLAY

		// This loop displays the output and Change Mask
		for(int countView=0;countView<e.V.size();countView++)
		{
#if __CM_DISPLAY
			//int temp = 1;
			if(e.V[countView]->centroidCalculator.changeMaskWindow.size()>e.V[countView]->centroidCalculator.nPrevFrames)
			{
				change_mask_img[countView] = cvCloneImage(e.V[countView]->centroidCalculator.changeMaskWindow[e.V[countView]->centroidCalculator.nPrevFrames]);
				for (int j = 0;j<e.V[countView]->origCentroidsByCentroidCalculator.size();j++)
					cvDrawCircle(change_mask_img[countView],cvPoint((int)e.V[countView]->origCentroidsByCentroidCalculator[j].x,(int)e.V[countView]->origCentroidsByCentroidCalculator[j].y),2,cvScalar(50),-1);
				cvShowImage(cname[countView].c_str(),change_mask_img[countView]);
			}

#endif
#if __DISPLAY
			if(e.V[countView]->centroidCalculator.originalFrameWindow.size()>e.V[countView]->centroidCalculator.nPrevFrames && \
				e.V[countView]->centroidCalculator.originalFrameWindow[e.V[countView]->centroidCalculator.nPrevFrames]!=NULL)
				cvShowImage(wname[countView].c_str(),e.V[countView]->centroidCalculator.originalFrameWindow[e.V[countView]->centroidCalculator.nPrevFrames]);
#endif
		}	
		for(int k=0;k<change_mask_img.size();k++)
			if(change_mask_img[k]!=NULL)
				cvReleaseImage(&change_mask_img[k]);

#endif
		f_processed++; // Increment frames processed (used for keeping track of FPS).
#if __VERBOSE
		printf("\nCumulative Frame Processing speed (FPS) = %.2lf fps ... Current FPS = %.2lf fps",f_processed/(((double)(clock()-time_start))/(CLOCKS_PER_SEC)),\
			1.0/(((double)(clock()-curr_time))/(CLOCKS_PER_SEC)));
#endif

#if __CM_DISPLAY || __DISPLAY
		cvWaitKey(((int)e.currFrameNumberProcessed>25300) ?  1 : 1); // For setting appropriate delay.

#endif

	}
	double total_time_in_silh_detector = ((double)total_clocks_in_Grab_And_SilhDetection)/((double)CLOCKS_PER_SEC);
	double total_time_in_corresponder = (double)total_clocks_corresponder/((double)CLOCKS_PER_SEC);
	double total_time_in_tracker = (double)total_clocks_tracker/((double)CLOCKS_PER_SEC);
	double total_time = difftime(time(NULL),start_time);
	double total_scaled_time_for_just_tracked_frames =
		(total_time_in_silh_detector/howManyFrames_Grab_and_SilhDetected)*howManyFramesTracked + \
		(total_time_in_corresponder/howManyFramesCorresponded)*howManyFramesTracked + \
		total_time_in_tracker;

	fprintf(fp_OutTotalTimeTakenInProcessing,"# of frames Silh Detected = %d\n# of frames Corresponded = %d\n# of frames Tracked = %d\n", \
		howManyFrames_Grab_and_SilhDetected,howManyFramesCorresponded,howManyFramesTracked);
	fprintf(fp_OutTotalTimeTakenInProcessing,"Total Time in processing frames (excluding initial BG computation) = %.2lf\n",total_time);
	fprintf(fp_OutTotalTimeTakenInProcessing,"Total scaled time for frames tracked (includes scaled time for bg computation+correspondence) = %.2lf\n",\
		total_scaled_time_for_just_tracked_frames);
	fprintf(fp_OutTotalTimeTakenInProcessing,"Total time in silh detctor+avi grabber = %.2lf   FPS = %.2lf\n",total_time_in_silh_detector,howManyFrames_Grab_and_SilhDetected/total_time_in_silh_detector);
	fprintf(fp_OutTotalTimeTakenInProcessing,"Total time in corresponder = %.2lf   FPS = %.2lf\n",total_time_in_corresponder,howManyFramesCorresponded/total_time_in_corresponder);
	fprintf(fp_OutTotalTimeTakenInProcessing,"Total time in tracker = %.2lf   FPS=%.2lf\n",total_time_in_tracker,howManyFramesTracked/total_time_in_tracker);
	fprintf(fp_OutTotalTimeTakenInProcessing,"Total scaled FPS = %.2lf\n",howManyFramesTracked/total_scaled_time_for_just_tracked_frames);
	
	fclose(fp_OutTotalTimeTakenInProcessing);

	fclose(fp_numCentroids);
	fclose(fp_CorrespondenceMap);
	fclose(fp_ErrorOut);
	fclose(fp_OutTrackFile);
#if __WRITE2DCENTROID
	for(int i=0;i<fp_OutCentroid.size();i++)
		fclose(fp_OutCentroid[i]);
#endif
#if __SAVE
	e.stopSavingToOutputFile();
#endif

	printf("\nDone .. ");
	cvDestroyAllWindows();


	return 0;
}
