#pragma once 
#include <Windows.h>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp> 

#include "config.hpp"


 using namespace std;



 class FILE_UTILS {
 public:
	 static std::vector <std::string>  list_files_in_folder(const std::string path, bool reverseOrder = false);
	 static std::vector <std::string>  list_sorted_files_in_folder(std::string path);

	 static bool is_folder(std::string filename);
	 static bool is_image_extension(std::string fname);

	 static std::string find_path(const std::string  filename);
	 static std::string  find_fname(const std::string  filename);

	 static std::string change_extension(const std::string & filename, const std::string & extension);
	 static std::string remove_extension(const std::string  filename);
	 static std::string find_extension(const std::string & filename);
	 static std::string add_soffix(const std::string & filename, const std::string postfix);
	 static bool file_exists(const std::string & filename);
	 static bool folder_exists(const std::string & filename);


	 static bool readConfigFile(Config& conf);
	 static bool readConfigFile(std::string ConfigFName, Config& conf);


 };


 