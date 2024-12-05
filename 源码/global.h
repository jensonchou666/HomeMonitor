#pragma once

#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <shared_mutex>
#include <atomic>
#include <iostream>
#include <thread>
#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <string>
#include <sstream>


#define MAX_FILENAME_LEN	200
#define MAX_TIMEBUFFER_LEN	100


#define  prompt				"[Home Moniter]"
#define	 main_win_name		"camera"


struct Global {
	std::string Root = "G:\\HomeMonitor";
	std::string history_path = "history";
	std::string exception_path = "exception";
	std::string event_path = "event";
	std::string temp_path = "temp";
	cv::String fNameDelete;
	WCHAR  f_name_delete[MAX_FILENAME_LEN];
	cv::String fNameEventOld;
	cv::String fNameEventNew;
	cv::String fNameEventRest;
	std::mutex eventFile_mutex;
	void initDirectory();
	void initAviFiles();


	int history_length = 60 * 10;//秒
	int history_time_span = 60 * 2;//分
	int	temp_length = 60 * 1;//秒
    std::shared_mutex	param_mutex;
	void show_param(std::string param);
	void show_params();
	void cmd_set(std::istringstream& issm);

	/*
	//彻底删除!!!!!
	void deleteFile(const char* fName);
	void deleteFile(std::string fName);
	void deleteFile(cv::String fName);
	std::mutex deleteFile_mutex;
	*/
};
static Global global;




struct ProcCtrl {

/* 设置proc_stoped标志位应在具体工作实施之前 */
/* 在重启程序时主线程要等待其他监测线程结束 */
/* 各线程首先检查 reload_wait_counter == 0 */
/* 再检查自己对应的标志位 */
#define WAIT_THREADS_COUNT	2
	std::atomic_bool stoped = false;
	std::atomic_int16_t wait_counter = 0;
	std::atomic_bool do_reload = false;

	std::atomic_bool flag_door_m_finished = false;
	std::atomic_bool flag_body_d_finished = false;
	
	inline void stop_begin() {
		wait_counter = WAIT_THREADS_COUNT;
		flag_door_m_finished = false;
		flag_body_d_finished = false;
	}
	inline void stop_finished() {
		flag_door_m_finished = false;
		flag_body_d_finished = false;
		//stoped = false;
		//...
	}

};
static ProcCtrl  proc_ctrl;


/*-----------------------------------------------------------------------------*/




int AnsiToUnicode(const char* szStr, wchar_t* buffer, int size);
//将宽字节wchar_t*转化为单字节char*  
int UnicodeToAnsi(const wchar_t* szStr, char* buffer, int size);
#define toWChar(str,w_str)  	MultiByteToWideChar(CP_ACP, 0, str, \
 strlen(str) + 1, w_str,sizeof(w_str) / sizeof(w_str[0]));
#define toChar(w_str,str)  	MultiByteToWideChar(CP_ACP, 0, str, \
 strlen(str) + 1, w_str,sizeof(w_str) / sizeof(w_str[0]));

struct Timer {
	Timer(int frec=2000) : frec_ms(frec){}
	int frec_ms;
	double t_latest = 0;
	double t_begin = 0;
	void begin() {
		t_begin = (double)cv::getTickCount() * 1000 / cv::getTickFrequency();
	}
	void end(std::string msg,int ms=0) {
		if (ms == 0)ms = frec_ms;

		double tm2 = (double)cv::getTickCount() * 1000 /
			cv::getTickFrequency();
		if (tm2 - t_latest < ms)return;
		std::cout << "【 " <<std::setw(7)<< std::setprecision(4)
			<<tm2 - t_begin << "ms 】" << msg<< std::endl;
		t_latest = tm2;
	}
};

static inline void deleteFile(const char* fName) {
	wchar_t buffer[MAX_FILENAME_LEN];
	AnsiToUnicode(fName, buffer, MAX_FILENAME_LEN);
	DeleteFile(buffer);
}
static inline void  deleteFile(std::string fName) {
	deleteFile(fName.c_str());
}
static inline void  deleteFile(cv::String fName) {
	deleteFile(fName.c_str());
}

void to_string(time_t* time, char* time_str, size_t size);
time_t to_time_t(const char* time_str);
std::string to_string(const std::chrono::system_clock::time_point& time);

void sendWINMessage(HWND win, const char* message);
void sendWINMessage(const char* winName,const char* message);
bool affirm();
#define YES 1
#define NO 0
#define NO_ALL -1
#define YES_ALL -1
short affirm2();
bool affirm_defN();
void slpit(std::string str,std::vector<std::string>* ss);




void on_mouse_mainwin(int event, int x, int y, int flag, void* data);
bool lineKBFunction(const cv::Point& p1, const cv::Point& p2, double& k, double& b);
bool fileAttributes(const char* name, long file_attributes);

/*
cv::Rect rectWinToImg(const cv::Rect& r);
cv::Rect rectImgToWin(const cv::Rect& r);

cv::Point2i pointConvert(const cv::Point2i& p,
	const cv::Size& org, const cv::Size& dst);
cv::Rect rectConvert(const cv::Rect& r,
	const cv::Size& org, const cv::Size& dst);
*/
/*
struct PromptEndl {};
std::ostream& operator << (std::ostream & os, PromptEndl & endlp) {
	os << std::endl;
	os << prompt;
	return os;
}
static PromptEndl endlp;
*/
