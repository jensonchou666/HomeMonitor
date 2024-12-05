#pragma once

#include "global.h"




/* status 在整个事件与录制过程结束后才置为NoEvent */
#define	NoEvent						0	//0b000
#define	EventLeaveHome				1	//0b001
#define	EventIn						2	//0b010
#define EventGoHome					3	//0b010
#define EventInvade					4	//0b110

#define EventGoHomeContinue			5  //回家后门再次被打开的事件类型

//#define EventLeaveHomeContinue		5
//#define EventGoHomeContinue			6


#define idle	0
#define shelt	1
#define leave	2


using namespace std;
struct EventCtrl
{
public:

	mutex param_timeout_mutex;
	/***********     参数    ***********/
	long leave_timeout = 15 * 1000;//遮挡再离开后，超过 ~ms 状态置为闲置
	long shelt_timeout = 60 * 1000;//遮挡时间超过 ~ms，认为标记消失时间过长
	int counter_leave = 0;
	int counter_leave_max = 4;

	//检测人脸超时, (*辨别失败继续辨别，直至成功或超时) 
	//必须小于 event_end_record_time和exception_end_record_time
	long detectFace_timeout = 8 * 1000;
	long safeEvent_timeout =25 * 1000;	//离开时间超时
	//这是防止录制时间过长的超时
	long invade_once_timeout = 2 * 60 * 1000;
	//入侵事件发生后的 ~ms内，所有出门事件均视为入侵
	long invade_timeout = 2 * 60 * 1000;

	long event_end_record_time = 10*1000;//普通事件结束后需继续录制的时间
	long exception_end_record_time = 20 * 1000;//异常事件结束后需继续录制的时间
	//int event_end_wait_time = 6 * 1000;//录制结束后等待一段时间再判断下一事件

	atomic_bool			is_debug_shelt = true;


	/***********     标志、时间    ***********/
	atomic_int16_t		status= NoEvent;

	atomic_bool			event_begin = false;
	chrono::system_clock::time_point time_event_begin;
	chrono::system_clock::time_point time_lastEvent_begin;
	shared_mutex time_event_begin_mutex;

	atomic_bool			event_end = false;
	chrono::system_clock::time_point time_event_end;
	shared_mutex time_event_end_mutex;
	bool is_event_record_timeout(const chrono::system_clock::time_point & now);
	//atomic_bool			event_continue = false;
	atomic_bool			faceDetect_over = false;

	chrono::system_clock::time_point time_event_invade;
	shared_mutex time_event_invade_mutex;

	atomic_bool			event_is_multi=false;
	atomic_bool			event_is_timeout = false;
	mutex event_mutex;

	void reset() {

		status = NoEvent;
		intra_status = idle;

		event_end = false;
		faceDetect_over = false;
		is_open_timeout = false;
		event_begin = false;
		event_is_multi = false;

		mark_exist = true;
		door_opened = false;
	}

	void doorOpening();
	void doorClosing();
	void markDisapear();
	void endEvent() {
		status = NoEvent;
		event_end = false;
	}

	/***********     事件处理接口    ***********/


	const char  * winName_qq_android="我的Android手机";
	mutex sendQQ_mutex;
	void sendQQ(const char* message) {
		sendQQ_mutex.lock();
		HWND win = FindWindowA(NULL, winName_qq_android);
		if (win == NULL) {
			cout << "未找到窗口：" << winName_qq_android << endl;
			sendQQ_mutex.unlock();
			return;
		}
		sendWINMessage(win, message);
		sendQQ_mutex.unlock();
	}
	void forgetCloseDoorLeaveHome(const chrono::system_clock::time_point& time) {

		cout << to_string(time) << "忘记关门" << endl;
		sendQQ("[Warning] Forget to close the door");
	}
	void forgetCloseDoorGoHome(const chrono::system_clock::time_point& time) {

		cout <<to_string(time)<< "忘记关门" << endl;
		sendQQ("[Warning] Forget to close the door");
	}
	void forgetCloseDoorInvade(const chrono::system_clock::time_point& time) {
		cout << to_string(time) << "入侵时间已超过" << 
			invade_once_timeout / 1000 << "分钟" <<
			",未关门,将结束录制" << endl;
		sendQQ("[Warning] Door is opening [invade timeout]");
	}
	void forgetCloseDoorGoHomeContinue(const chrono::
		system_clock::time_point& time) {

		cout << to_string(time) << "(GoHomeContinue)时间已超过" <<
			invade_once_timeout / 1000 << "分钟" <<
			",未关门,将结束录制" << endl;
		sendQQ("[Warning] Door is opening [GoHomeContinue timeout]");
	}

	//FaceDetector模块调用
	void invade(const chrono::system_clock::time_point & time) {
		cout << to_string(time) << "@@@ 注意：入侵事件发生" << endl;
		sendQQ("[Alarm] Intrusion Event Occured");
	}
	//FaceDetector模块调用
	void gohome(const chrono::system_clock::time_point& time) {
		cout << to_string(time) << "欢迎回家" << endl;
		sendQQ("[Info] Welcome to return home");
	}

	void doorOpenedIn(const chrono::system_clock::time_point& time) {
		cout << to_string(time) << "Warning： 门被打开" << endl;
		sendQQ("[Warning] Door is opened");
	}
	void doorOpenedLeaveHome(const chrono::system_clock::time_point& time) {
		cout << to_string(time) << "从室内开门" << endl;
		sendQQ("[Info] Good bye take care");
	}


	void doorClosedNoEvent(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< "门已关闭(打开超时),用时："
			<< (double)time_count / 1000 << "s" << endl;
		sendQQ("[Info] Door is closed [timeout]");
	}
	void doorClosedIn(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< "门已关闭(正在识别人脸),用时："
			<< (double)time_count / 1000 << "s" << endl;
		//sendQQ("[Info] Door is closed [detectng face]");
	}
	void doorClosedInvade(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< " 入侵事件:门已关闭,用时："
		 << (double)time_count / 1000 << "s" << endl;
		//sendQQ("[Warning] Door is closed [intrusion]");
	}

	void doorClosedGoHome(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< " 回家事件:门已关闭,用时："
			<< (double)time_count / 1000 << "s" << endl;
		//sendQQ("[Info] Door is closed");
	}
	void doorClosedGoHomeContinue(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< " 门已关闭,用时："
			<< (double)time_count / 1000 << "s" << endl;
	}
	
	void doorClosedLeaveHome(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< " 出门事件:门已关闭,用时："
			<< (double)time_count / 1000 << "s" << endl;
	}

	void OpenedAgainInvade(const chrono::system_clock::time_point& time) {
		cout << to_string(time)
			<< " 入侵事件:门再次被打开"<< endl;
		sendQQ("[Alarm] Door is opened again");

	}
	void OpenedAgainIn(const chrono::system_clock::time_point& time) {
		//cout << to_string(time)
		//	<< " 门再次被打开" << endl;
	}
	void OpenedAgainAfterGoHome(const chrono::system_clock::time_point& time) {
		cout << to_string(time)
			<< "Warning: 门再次被打开" << endl;
		sendQQ("[Warning] Door is opened again");
	}


	bool door_opened = false;

	void do_cmd(std::istringstream& issm);
	void cmd_set(istringstream& issm);
	void show_param(istringstream& issm);


	//将标记消失并开门视为离家事件，这种方式存在漏洞，后续改进
	#define  shelt_is_leaveHome_style 

	void judge_doorOpened();
private:
	void opening();

	int16_t intra_status = idle;
	bool mark_exist = true;
	bool is_open_timeout = false;


	chrono::system_clock::time_point event_shelt_begin = chrono::system_clock::now();
	chrono::system_clock::time_point event_leave_begin = chrono::system_clock::now();


};

static EventCtrl  event_ctrl;