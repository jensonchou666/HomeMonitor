
#include "global.h"

std::mutex sendWINMessagelock;
void sendWINMessage(HWND win,
	const char* message) {
	//EnumChildWindows(win, enumChildWin,0);
	sendWINMessagelock.lock();
	//keybd_event(VK_LBUTTON, 0, 0, 0);
	size_t len = strlen(message);

	HWND currentWind = GetForegroundWindow();
	SwitchToThisWindow(win, true);
	for (size_t i = 0; i < len; i++) {
		char c = message[i];
		if (c >= 65 && c <= 90) {
			PostMessageA(win, WM_KEYDOWN, VK_CAPITAL, 0);
			PostMessageA(win, WM_KEYUP, VK_CAPITAL, 0);
			PostMessageA(win, WM_KEYDOWN, c, 0);
			PostMessageA(win, WM_KEYDOWN, VK_CAPITAL, 0);
			PostMessageA(win, WM_KEYUP, VK_CAPITAL, 0);
		}
		else if (c >= 97 && c <= 122)
			PostMessageA(win, WM_KEYDOWN, c - 32, 0);
		else if (c >= 48 && c <= 57) {   
			PostMessageA(win, WM_KEYDOWN,c, 0);
		}	
		else{
			switch (c) {
			case ' ':
				PostMessageA(win, WM_KEYDOWN, VK_SPACE, 0);
				break;
			case '[':
				PostMessageA(win, WM_KEYDOWN, VK_OEM_4, 0);
				break;
			case ']':
				PostMessageA(win, WM_KEYDOWN, VK_OEM_6, 0);
				break;
			default:
				PostMessageA(win, WM_KEYDOWN, VK_MULTIPLY, 0);
			}		
		}

	}
	keybd_event(VK_RETURN, 0, 0, 0);
	keybd_event(VK_RETURN, 0,KEYEVENTF_KEYUP,0);

	SwitchToThisWindow(currentWind, true);

	sendWINMessagelock.unlock();
}
void sendWINMessage(const char* winName,
	const char* message) {
	HWND win = FindWindowA(NULL, winName);
	if (win == NULL) return;
	sendWINMessage(win, message);
}




time_t to_time_t(const char* time_str) {
	struct tm _tm_ = {};

	int ret=sscanf_s(time_str, "%d-%d-%d【%d-%d-%d】",
		& (_tm_.tm_year), &(_tm_.tm_mon),
		&(_tm_.tm_mday), &(_tm_.tm_hour),
		&(_tm_.tm_min),  &(_tm_.tm_sec));

	_tm_.tm_year -= 1900;
	_tm_.tm_mon -= 1;
	if (!ret)return (time_t)-1;
	return mktime(&_tm_);
}
void to_string(time_t* time, char* time_str, size_t size) {
	struct tm _tm_;
	localtime_s(&_tm_, time);

	sprintf_s(time_str, size, "%d-%d-%d【%d-%d-%d】",
		_tm_.tm_year + 1900, _tm_.tm_mon + 1,
		_tm_.tm_mday, _tm_.tm_hour,
		_tm_.tm_min, _tm_.tm_sec);
}


#include <chrono>
std::string to_string(const std::chrono::system_clock::time_point & time) {

	char time_buffer[MAX_TIMEBUFFER_LEN];
	memset(time_buffer, 0, MAX_TIMEBUFFER_LEN);
	time_t t = std::chrono::system_clock::to_time_t(time);
	to_string(&t, time_buffer, MAX_TIMEBUFFER_LEN);
	return time_buffer;
}

void slpit(std::string str, 
	std::vector<std::string> *ss) {
	std::string s ;
	std::istringstream is(str);
	while (is) {
		is >> s;
		ss->push_back(s);
	}
}

#define _buf_affirm_size_	100
char _buf_affirm_[_buf_affirm_size_] = "\0";
bool affirm() {
	memset(_buf_affirm_, '\0', _buf_affirm_size_);
	std::cout << "(def y)[Y/N]";

	std::cin.getline(_buf_affirm_, _buf_affirm_size_);

	if (_buf_affirm_[0] == '\0' || 
		_buf_affirm_[0] == 'y' || 
		_buf_affirm_[0] == 'Y')return true;
	else return false;
}
short affirm2() {
	memset(_buf_affirm_, '\0', _buf_affirm_size_);
	std::cout << "(def y)(U:Y all,M:N all)[Y/N/U/M]";

	std::cin.getline(_buf_affirm_, _buf_affirm_size_);

	if (_buf_affirm_[0] == '\0' ||
		_buf_affirm_[0] == 'y' ||
		_buf_affirm_[0] == 'Y')return YES;
	else if (_buf_affirm_[0] == 'n' ||
		_buf_affirm_[0] == 'N')return NO;
	else if (_buf_affirm_[0] == 'u' ||
		_buf_affirm_[0] == 'U')return  YES_ALL;
	else if (_buf_affirm_[0] == 'm' ||
		_buf_affirm_[0] == 'M')return NO_ALL;
	else return NO;
}
short affirm2_defN() {
	memset(_buf_affirm_, '\0', _buf_affirm_size_);
	std::cout << "(def n)(U:Y all,M:N all)[Y/N/U/M]";

	std::cin.getline(_buf_affirm_, _buf_affirm_size_);

	if (_buf_affirm_[0] == 'y' ||
		_buf_affirm_[0] == 'Y')return YES;
	else if (_buf_affirm_[0] == 'n' ||
		_buf_affirm_[0] == 'N')return NO;
	else if (_buf_affirm_[0] == 'u' ||
		_buf_affirm_[0] == 'U')return  YES_ALL;
	else if (_buf_affirm_[0] == 'm' ||
		_buf_affirm_[0] == 'M')return NO_ALL;
	else return NO;
}
bool affirm_defN() {
	memset(_buf_affirm_, '\0', _buf_affirm_size_);
	std::cout << "(def n)[Y/N]";

	std::cin.getline(_buf_affirm_, _buf_affirm_size_);

	if (_buf_affirm_[0] == 'y' ||
		_buf_affirm_[0] == 'Y')return true;
	else return false;
}

using namespace std;

void global_cmd_help() {
	cout << "Max time length of history video file   (hl)  [W]"	<< std::endl;
	cout << "Max count of history video files        (hc)  [W]"	<< std::endl;
	cout << "Max time length of temp video file      (tl)  [W]"	<< std::endl;
}
void Global::show_param(std::string param) {
	if (param.empty() || param == "all") {
		show_params();
		return;
	}
	param_mutex.lock_shared();
	if		(param == "hl" )
		cout << "历史录像单个文件的最大长度(单位：秒)                   " 
		<< history_length << std::endl;
	else if (param == "ht")
		cout << "历史录像的保存时间跨度，超过期限的会被删除(单位：分)   " 
		<< history_time_span << std::endl;
	else if (param == "tl")
		cout << "短期录像单个文件的最大长度(单位：秒) 决定事件回溯长度  " 
		<< temp_length << std::endl; 
	param_mutex.unlock_shared();

	if (param == "help" || param == "-h")global_cmd_help();
}
void Global::show_params() {
	cout << "Global params :" << std::endl;
	param_mutex.lock_shared();
	cout << "历史录像单个文件的最大长度(单位：秒)                   " 
		<< history_length << std::endl;
	cout << "历史录像的保存时间跨度，超过期限的会被删除(单位：分)   " 
		<< history_time_span << std::endl;
	cout << "短期录像单个文件的最大长度(单位：秒) 决定事件回溯长度  "  
		<< temp_length << std::endl;
	param_mutex.unlock_shared();
}
void Global::cmd_set(std::istringstream& issm) {
	string param; int value = 0;
	if (issm)issm >> param >> value;
	param_mutex.lock();
	if (param == "help" || param == "-h")
		cout << ">> set g/global (parameter) (value)" << endl;
	else if (value == 0) {
		cout << "错误指令" << endl;
	}
	else if (param == "hl")history_length = value;
	else if (param == "ht")history_time_span = value;
	else if (param == "tl")temp_length = value;
	else cout << "错误指令" << endl;
	param_mutex.unlock();
}

//将宽字节wchar_t*转化为单字节char*  
int UnicodeToAnsi(const wchar_t* szStr, char* buffer, int size)
{
	int nLen = WideCharToMultiByte(CP_ACP, 0, szStr, -1, NULL, 0, NULL, NULL);
	if (nLen == 0)	return 0;
	
	nLen = min(size, nLen);
	return WideCharToMultiByte(CP_ACP, 0, szStr, -1, buffer, nLen, NULL, NULL);
}


int AnsiToUnicode(const char* szStr, wchar_t* buffer, int size)
{
	int nLen = MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, szStr, -1, NULL, 0);
	if (nLen == 0)	return 0;
	nLen = min(size, nLen);
	return MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, szStr, -1, buffer, nLen);
}

/*
void Global::deleteFile(const char* fName) {
	deleteFile_mutex.lock();


	if (rename(fName, fNameDelete.c_str())) {
		DeleteFile(f_name_delete);
		if (rename(fName, fNameDelete.c_str())) {
			cout << "!!!!!!!!!!!!!! 不科学 !!!!!!!!!!!!!!" << endl;
			exit(-1);
		}
	}

	wchar_t buffer[MAX_FILENAME_LEN];
	AnsiToUnicode()
	DeleteFile(fName);
	deleteFile_mutex.unlock();
}
void  Global::deleteFile(std::string fName) {
	deleteFile(fName.c_str());
}
void  Global::deleteFile(cv::String fName) {
	deleteFile(fName.c_str());
}
	*/


/*
using namespace cv;
Point2i pointConvert(const Point2i& p,
	const Size& org, const Size& dst) {

	double ratew = (double)dst.width / org.width;
	double rateh = (double)dst.height / org.height;
	//cout << "rate: " << ratew << " " << rateh << endl;
	return Point2i(
		(int)(ratew * (double)p.x),
		(int)(rateh * (double)p.y)
	);
}
Rect rectConvert(const Rect& r, const Size& org, const Size& dst) {
	return Rect(pointConvert(r.br(), org, dst), pointConvert(r.tl(), org, dst));
}
*/



bool lineKBFunction(const cv::Point& p1, const cv::Point& p2, double& k, double& b) {
	if (p1.x == p2.x)return false;
	k = (double)(p1.y - p2.y) / (p1.x - p2.x);
	b = ((double)(p1.y) * p2.x - (double)(p2.y) * p1.x) / (p2.x - p1.x);
	return true;
}
bool fileAttributes(const char* name, long file_attributes) {
	WIN32_FIND_DATA wfd;

	WCHAR  file[MAX_FILENAME_LEN];
	MultiByteToWideChar(CP_ACP, 0, name, strlen(name) + 1, file,
		sizeof(file) / sizeof(file[0]));

	HANDLE hFind = FindFirstFile(file, &wfd);
	if ((hFind != INVALID_HANDLE_VALUE) &&
		(wfd.dwFileAttributes & file_attributes)) {
		FindClose(hFind);
		return true;
	}
	FindClose(hFind);
	return false;
}