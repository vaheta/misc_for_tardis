/*Function that resizes all the images in the input folder and saves to another output folder*/

#include <vector>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>

using namespace cv;
using namespace std;

void printUsage (const char* progName)
{
	std::cout << "\n\nUsage: "<<progName<<" [input dir]\n\n"
		<< "Input the path to folder with photos\n"
		<< "Example:\n"
		<< "./imresfun foldername\n"
		<< "\n\n";
}

int main(int argc, char** argv )
{
	if (argc != 2)
	{
		printUsage(argv[0]);
		return 0;
	}
	
	Mat input;
	Mat output;
	string dirout = string(argv[1]) + "/out";

	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);

	int status = mkdir(dirout.c_str(), 0777);

	DIR *dpdf;
	struct dirent *epdf;

	dpdf = opendir(argv[1]);

	if (dpdf != NULL){
		while (epdf = readdir(dpdf))
		{
			if ((strcmp(epdf->d_name,".") != 0) && (strcmp(epdf->d_name,"..") != 0) && (strcmp(epdf->d_name,"out") != 0))
			{
				string photo_path = format ("%s/%s", argv[1], epdf->d_name);
				input = imread(photo_path, IMREAD_COLOR);
				resize (input, output, Size(), 0.5, 0.5);
				string photo_path_out = format ("%s/out/%s", argv[1], epdf->d_name);
				imwrite(photo_path_out, output,  compression_params);
			}
		}
	}

	closedir(dpdf);

	cout << "Done!" << endl;
    return 0;
}