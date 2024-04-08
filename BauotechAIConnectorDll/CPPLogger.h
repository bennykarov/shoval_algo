/*
    Copyright (C) 2017 Scott McKittrick

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SAMLogger.  If not, see <http://www.gnu.org/licenses/>.
    
    By: Scott McKittrick

*/

#ifndef CPPLOGGER_H
#define CPPLOGGER_H

#include <string>
#include <ctime>
#include <iostream>
#include <mutex>
#include <sstream>
#include <fstream>

#define DEBUG 5
#define INFO 4
#define WARN 3
#define ERROR 2
#define FATAL 1

#define LOGMODE_STDOUT 0x01
#define LOGMODE_FILE 0x02
#define LOGMODE_CUSTOM 0x04

//Abstract class used as an interface to define custom logging classes.
//These custom logging classes take a string built in CPPLogger::writeLog()
// and print them through whatever logging mechanism they want. 
class CustomLogger
{
 public:
  virtual void writeLogString(std::string message) = 0;
};

class CPPLogger 
{
 public:
  static CPPLogger& getLog() {
    static CPPLogger instance;
    return instance;
  }

  void setLogLevel(int logLevel)
  {
    
    std::lock_guard<std::mutex> lockGuard(mutex);
    if(logLevel < 6 && logLevel > 0)
      debugLevel = logLevel;
  }
 
  void writeLog(int severity, std::string message) {
    std::lock_guard<std::mutex> lockGuard(mutex);
    if(severity <= debugLevel)
      {
	std::string time = getTimeStamp();
	std::string severityString = getSeverityString(severity);

	std::stringstream out;
	out << time << ":" << severityString << "-" << message << std::endl;
	
	if((logMode & LOGMODE_STDOUT) == LOGMODE_STDOUT)
	  {
	    std::cout << out.str();
	  }
	
	if((logMode & LOGMODE_FILE) == LOGMODE_FILE)
	  {
	    logFile << out.str() << std::flush;
	  }

	if((logMode & LOGMODE_CUSTOM) == LOGMODE_CUSTOM)
	  {
	    if(mCustomLogger != NULL)
	      mCustomLogger->writeLogString(out.str());
	  }
      }
  }

  int setLogModeStdOut()
  {
    std::lock_guard<std::mutex> lockGuard(mutex);
    
    logMode |= LOGMODE_STDOUT;
    
    return 0;
  }

  int setLogModeFile(std::string filePath)
  {
    if(filePath.empty())
      {
	writeLog(ERROR, "Log file path is empty");
	return -1;
      }
    
    if((logMode & LOGMODE_FILE) == LOGMODE_FILE)
      {
	writeLog(INFO, "New log file provided. Closing this file.");
      }
    
    //We don't lock the mutex until here so that we can still write if there are any errors
    //Otherwise we could accidentally block when writeLog waits for the mutex to be freed and this function waits to free it until after writeLog completes
    std::lock_guard<std::mutex> lockGuard(mutex);
    if((logMode & LOGMODE_FILE) == LOGMODE_FILE)
      {
	logFile.close();
      }
    
    logFile.open(filePath);
    if(logFile.fail())
      {
	return -1;
      }

    logMode |= LOGMODE_FILE;
  }

  int unSetLogMode(uint8_t mode)
  {
    logMode &= ~mode;
    std::cout << "Log MOde:" << logMode;
    return 0;
  }

  //Add a custom logger to recieve and process the log messages
  int setLogModeCustom(CustomLogger *c)
  {
    if(c == NULL)
      return -1;
    
    mCustomLogger = c;

    logMode |= LOGMODE_CUSTOM;
    return 0;
  }
    
    
 private:
  int debugLevel = ERROR; //Default to errors
  uint8_t logMode = LOGMODE_STDOUT; //Default log mode to stdout.
  std::ofstream logFile;

  //Used to allow custom logging destinations. 
  CustomLogger *mCustomLogger;
 
  //Block Constructors to ensure singleton
  CPPLogger() {}
  CPPLogger(CPPLogger const&);
  void operator=(CPPLogger const&);

  //Mutex for synchronization
  std::mutex mutex;

  std::string getTimeStamp()
    {
      time_t rawTime;
      time(&rawTime);
      tm *curTime = gmtime(&rawTime);
      char buffer[30];
      std::strftime(buffer, 30, "%Y-%m-%d %H:%M:%S %Z", curTime);
      std::string timeString(buffer);
      return timeString;
    }

  std::string getSeverityString(int severity)
    {
      switch(severity)
	{
	case DEBUG:
	  return "Debug";
	case INFO:
	  return "Info";
	case WARN: 
	  return "Warning";
	case ERROR: 
	  return "Error";
	case FATAL:
	  return "Fatal";
	default:
	  return "Unkown";
	}
    }
};

class LOG
{
 public:
  LOG() { debugLevel = ERROR; }
  LOG(int severity) { debugLevel = severity; }
  ~LOG() {
    CPPLogger::getLog().writeLog(debugLevel, stream.str());
  }

  template <typename T>
    LOG& operator<<(T const & value)
    {
      stream << value;
      return *this;
    }
  
 private:
  int debugLevel;
  std::stringstream stream;

};      
#endif
