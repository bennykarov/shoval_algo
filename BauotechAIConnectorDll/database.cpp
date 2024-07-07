#include <filesystem>
#include <iostream>
#include <fstream>
#include <stdarg.h>
#include <string.h>
#include <vector>
#include <codecvt>



#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/videoio.hpp"

// config file
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/lexical_cast.hpp> 
#include <boost/property_tree/json_parser.hpp>
#include "files.hpp"
#include "utils.hpp"
#include "CObject.hpp"


#include "database.hpp"



namespace efs = std::experimental::filesystem;
namespace fs = std::filesystem;

std::wstring stringToWstring_(const std::string& t_str)
{
    //setup converter
    typedef std::codecvt_utf8<wchar_t> convert_type;
    std::wstring_convert<convert_type, wchar_t> converter;

    //use converter (.to_bytes: wstr->str, .from_bytes: str->wstr)
    return converter.from_bytes(t_str);
}


  std::vector <std::string>  FILE_UTILS::list_files_in_folder(std::string path, bool reverseOrder)
  {
	  std::vector <std::string> filesList, filesList_reverseOrder;
	  //std::string path = "/path/to/directory";
	  for (const auto & entry : fs::directory_iterator(path))
		  filesList.push_back(entry.path().string());
	  //std::cout << entry.path() << std::endl;

	  if (!reverseOrder) 
		return filesList;

	  // REVERSE ORDER 
	  for (int r = filesList.size() - 1; r >= 0; r--)
		  filesList_reverseOrder.push_back(filesList[r]);

	  return filesList_reverseOrder;
  }

  std::vector <std::string>  FILE_UTILS::list_sorted_files_in_folder(std::string path)
  {
	  std::vector <std::string> filesList, filesList_reverseOrder;
	  //std::string path = "/path/to/directory";
	  for (const auto & entry : fs::directory_iterator(path))
		  filesList.push_back(entry.path().string());

	  // Sort file names
	  std::sort(filesList.begin(), filesList.end(), [](const std::string  & a, const std::string & b) -> bool { return a < b; });

	  return filesList;

  }
  
  
  
  std::string FILE_UTILS::change_extension(const std::string & filename, const std::string & extension)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);
    _makepath_s(path, drive, dir, fname, extension.c_str());
    
    return path;
  }

std::string FILE_UTILS::remove_extension(const std::string  filename)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);
    _makepath_s(path, drive, dir, fname, NULL);
    
    return path;
  }

std::string FILE_UTILS::find_path(const std::string  filename)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);
    _makepath_s(path, drive, dir, NULL , NULL);
    
    return std::string(path);
  }

std::string FILE_UTILS::find_fname(const std::string  filename)
{
	char path[_MAX_PATH];
	char drive[_MAX_DRIVE];
	char dir[_MAX_DIR];
	char fname[_MAX_FNAME];
	char ext[_MAX_EXT];

	_splitpath_s(filename.c_str(), drive, dir, fname, ext);
	_makepath_s(path, drive, dir, NULL, NULL);

	return std::string(fname);
}

    std::string FILE_UTILS::find_extension(const std::string  &filename)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);

	std::string extStr(ext);
	extStr.erase(std::remove(extStr.begin(), extStr.end(), '.'), extStr.end());

	return extStr;
  }
 std::string FILE_UTILS::add_soffix(const std::string & filename, const std::string postfix)
  {
    char path[_MAX_PATH];
    char drive[_MAX_DRIVE];
    char dir[_MAX_DIR];
    char fname[_MAX_FNAME];
    char ext[_MAX_EXT];
    
    _splitpath_s(filename.c_str(), drive, dir, fname, ext);
	strcat_s(fname ,postfix.c_str());
    _makepath_s(path, drive, dir, fname, ext);
    
    return path;
  }

 bool FILE_UTILS::file_exists(const std::string& filename)
 {
     return GetFileAttributes(stringToWstring_(filename).c_str()) != (DWORD)-1;
 }

 bool FILE_UTILS::folder_exists(const std::string& filename)
 {
     //return GetFileAttributes((LPCWSTR)filename.c_str()) != (DWORD)-1;
     DWORD ftype = GetFileAttributes(stringToWstring_(filename).c_str());
     if (INVALID_FILE_ATTRIBUTES == ftype)
         return false;
     return (FILE_ATTRIBUTE_DIRECTORY & GetFileAttributes(stringToWstring_(filename).c_str()));
 }


 bool FILE_UTILS::is_folder(std::string filename)
 {
     return (GetFileAttributes(stringToWstring_(filename).c_str()) == FILE_ATTRIBUTE_DIRECTORY);
 }

 bool FILE_UTILS::is_image_extension(const std::string fname_)
 {
     std::string fname = fname_;

     if (is_folder(fname))
         return false;


     std::transform(fname.begin(), fname.end(), fname.begin(), ::tolower);

     if (find_extension(fname) == "jpg" ||
         find_extension(fname) == "png" ||
         find_extension(fname) == "gif" ||
         find_extension(fname) == "tiff" ||
         find_extension(fname) == "tif" ||
         find_extension(fname) == "bmp")
         return true;

     return false;
 }




 using boost::property_tree::ptree;

 template <typename T>
 std::vector<T> as_vector(ptree const& pt, ptree::key_type const& key)
 {
     std::vector<T> r;
     for (auto& item : pt.get_child(key))
         r.push_back(item.second.get_value<T>());
     return r;
 }

void json_as_vector_example()
 {

    ifstream simple("C:\\src\\ConsoleApplication1\\simple.json");
    //ifstream simple("C:\\src\\ConsoleApplication1\\test.json");

    if (!simple.is_open())
        return;

    //std::stringstream ss("{\"a\": [8, 6, 2], \"b\": [2, 2, 1]}");
    std::string  tag, str;
    while (simple >> tag) 
        str.append(tag);

    std::stringstream ss(str);

     ptree pt;
     read_json(ss, pt);

     if (1)
     {
         for (int i = 0; i < 20; i++) {
             std::string camID = std::to_string(i);
             ptree::const_assoc_iterator it = pt.find(camID);
             if (it == pt.not_found()) 
                 continue;

             std::vector <int> vv = as_vector<int>(pt, camID);
             //for (auto i : as_vector<int>(pt, camID)) std::cout << i << ' ';
         }
     }
 }


bool FILE_UTILS::readConfigFile(Config& conf)
{
    return readConfigFile(FILES::CONFIG_FILE_NAME, conf);

}

bool FILE_UTILS::readConfigFile(std::string ConfigFName, Config& conf)
{
    if (!FILE_UTILS::file_exists(ConfigFName)) {
        std::cout << "WARNING : Can't find Config.ini file, use default values \n";
        return false;
    }

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(ConfigFName, pt);
    // [GENERAL]
    conf.videoName = pt.get<std::string>("GENERAL.video", conf.videoName);
    conf.roisName = pt.get<std::string>("GENERAL.rois", conf.roisName);
    conf.waitKeyTime = pt.get<int>("GENERAL.delay-ms", conf.waitKeyTime);
    conf.record = pt.get<int>("GENERAL.record", conf.record);
    conf.demoMode = pt.get<int>("GENERAL.demo", conf.demoMode);
    conf.debugLevel = pt.get<int>("GENERAL.debug", conf.debugLevel);
    //conf.showTruck = pt.get<int>("GENERAL.showTruck", conf.showTruck);
    conf.showMotion = pt.get<int>("GENERAL.showMotion", conf.showMotion);
    conf.camROI = to_array<int>(pt.get<std::string>("GENERAL.camROI", "0,0,0,0"));


    //---------
    // ALGO:
    //---------
    // [OPTIMIZE]  Optimization
    conf.skipMotionFrames = pt.get<int>("ALGO.stepMotion", conf.skipMotionFrames);
    conf.skipDetectionFrames = pt.get<int>("ALGO.stepDetection", conf.skipDetectionFrames);
    conf.skipDetectionFramesInMotion = pt.get<int>("ALGO.stepDetectionInMotion", conf.skipDetectionFramesInMotion);
    conf.skipDetectionFramesInMotion = pt.get<int>("ALGO.stepTracking", conf.skipDetectionFramesInMotion);
    std::vector <int> motionROI_vec = to_array<int>(pt.get<std::string>("ALGO.motionROI", "0,0,0,0"));
    if (motionROI_vec[2] > 0) // width
        conf.motionROI = cv::Rect(motionROI_vec[0], motionROI_vec[1], motionROI_vec[2], motionROI_vec[3]);

    conf.modelFolder = pt.get<std::string>("ALGO.modelFolder", conf.modelFolder);
    conf.scale = pt.get<float>("ALGO.scale", conf.scale);
    conf.MHistory = pt.get<int>("ALGO.MHistory", conf.MHistory);
    conf.MvarThreshold = pt.get<float>("ALGO.MvarThreshold", conf.MvarThreshold);
    conf.MlearningRate = pt.get<float>("ALGO.MlearningRate", conf.MlearningRate);
    conf.motionType = pt.get<int>("ALGO.motion", conf.motionType);
    conf.trackerType = pt.get<int>("ALGO.tracker", conf.trackerType);
    conf.trackerStep = pt.get<int>("ALGO.trackerStep", conf.trackerStep);
    conf.MLType = pt.get<int>("ALGO.ML", conf.MLType);
    conf.GPUBatchSize = pt.get<int>("ALGO.GPUBatchSize", conf.GPUBatchSize);

    return true;
}
