#include <unistd.h>
#include <stdlib.h> 
#include <stdio.h>
#include <errno.h>
#include <string>
#include <list>
#include <iostream>
#include <fstream> 

#define GetTextFileDirectory getcwd
#define MAX_BUFFER 512
/* 
 * Function to read in a list of images from text file
 * Input: a String specifying the path to the text file, containing image filenames to be read,
 * 		  relative to containing dir of text file.
 * Output: a List of absolute filenames to images 
 * 
 */

std::string textfilename("textfile.txt");

std::list<std::string> readListofImagesFromTextfile (std::string & path)
{
	std::list<std::string> absoluteFilenames; 
	std::ifstream textfile((path+textfilename).c_str()); 	
	std::cout << "givenpath: " << path << "\n\n";
    perror("error state trying to open file for reading for path");	 
		if(textfile.is_open()) {			
			std::string imgFileName = "";
			char buff[FILENAME_MAX];
			GetTextFileDirectory(buff, FILENAME_MAX);
			std::string currentDirectory(buff);
		//	char * absolutePath = NULL;
			
			std::string absolutePath = "", 
						commandstring = "realpath "; 
			
			// test before reading line for image file name		
			if (textfile.eof()) {
				perror("stream eofbit. error state");						 
			}
			if (textfile.fail()) {
				perror("stream failbit (or badbit). error state");  
			}
			if (textfile.bad()) {
				perror("stream badbit. error state"); 
			}	
			
			while(true)
			{
				//while(imgFileName.length() == 0)
				//{ 
				std::getline(textfile,imgFileName);
				//empty lines in between check
				//if(imgFileName.length() == 0)
					//continue;						
				//}	
				// test after reading line for image file name
				if (textfile.eof()) {
					perror("stream eofbit. error state");				
					break;
				}
				if (textfile.fail()) {
					perror("stream failbit (or badbit). error state"); 
					break;
				}
				if (textfile.bad()) {
					perror("stream badbit. error state");
					break;
				}							
				std::string relativePath = path+imgFileName;  
				std::cout << "rel path: " << relativePath << std::endl;
				//absolutePath = realpath(&relativePath[0],NULL);	 
				std::string command = (commandstring + relativePath);
				//system(command); 
				command.append(" 2>&1");
				FILE * stream; 
			    char buffer[MAX_BUFFER]; 
				stream = popen(command.c_str(), "r");
				if (stream) {
					while (!feof(stream))
						if (fgets(buffer, MAX_BUFFER, stream) != NULL) absolutePath.append(buffer);
					pclose(stream); 
				}				 
				if(!(absolutePath.empty()))
				{					 
					absoluteFilenames.push_back(absolutePath);
				}				
				//std::cout << "absolute path: " << absolutePath << "\n";
				absolutePath = ""; 
				imgFileName = ""; 	
			}  	
			//free (absolutePath); 
		} 	 
	return absoluteFilenames;
}


int main(int argc, char **argv)
{  
	std::string path(argv[1]);
	std::list<std::string> absFileNames = readListofImagesFromTextfile(path);
	std::list<std::string>::iterator it ;
	for(it = absFileNames.begin(); it != absFileNames.end(); it++)
	{
		std::cout << "abs file name: " << *it << std::endl;
	}
	return 0;
}
