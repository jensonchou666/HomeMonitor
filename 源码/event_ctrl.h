#pragma once

#include "global.h"




/* status �������¼���¼�ƹ��̽��������ΪNoEvent */
#define	NoEvent						0	//0b000
#define	EventLeaveHome				1	//0b001
#define	EventIn						2	//0b010
#define EventGoHome					3	//0b010
#define EventInvade					4	//0b110

#define EventGoHomeContinue			5  //�ؼҺ����ٴα��򿪵��¼�����

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
	/***********     ����    ***********/
	long leave_timeout = 15 * 1000;//�ڵ����뿪�󣬳��� ~ms ״̬��Ϊ����
	long shelt_timeout = 60 * 1000;//�ڵ�ʱ�䳬�� ~ms����Ϊ�����ʧʱ�����
	int counter_leave = 0;
	int counter_leave_max = 4;

	//���������ʱ, (*���ʧ�ܼ������ֱ���ɹ���ʱ) 
	//����С�� event_end_record_time��exception_end_record_time
	long detectFace_timeout = 8 * 1000;
	long safeEvent_timeout =25 * 1000;	//�뿪ʱ�䳬ʱ
	//���Ƿ�ֹ¼��ʱ������ĳ�ʱ
	long invade_once_timeout = 2 * 60 * 1000;
	//�����¼�������� ~ms�ڣ����г����¼�����Ϊ����
	long invade_timeout = 2 * 60 * 1000;

	long event_end_record_time = 10*1000;//��ͨ�¼������������¼�Ƶ�ʱ��
	long exception_end_record_time = 20 * 1000;//�쳣�¼������������¼�Ƶ�ʱ��
	//int event_end_wait_time = 6 * 1000;//¼�ƽ�����ȴ�һ��ʱ�����ж���һ�¼�

	atomic_bool			is_debug_shelt = true;


	/***********     ��־��ʱ��    ***********/
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

	/***********     �¼�����ӿ�    ***********/


	const char  * winName_qq_android="�ҵ�Android�ֻ�";
	mutex sendQQ_mutex;
	void sendQQ(const char* message) {
		sendQQ_mutex.lock();
		HWND win = FindWindowA(NULL, winName_qq_android);
		if (win == NULL) {
			cout << "δ�ҵ����ڣ�" << winName_qq_android << endl;
			sendQQ_mutex.unlock();
			return;
		}
		sendWINMessage(win, message);
		sendQQ_mutex.unlock();
	}
	void forgetCloseDoorLeaveHome(const chrono::system_clock::time_point& time) {

		cout << to_string(time) << "���ǹ���" << endl;
		sendQQ("[Warning] Forget to close the door");
	}
	void forgetCloseDoorGoHome(const chrono::system_clock::time_point& time) {

		cout <<to_string(time)<< "���ǹ���" << endl;
		sendQQ("[Warning] Forget to close the door");
	}
	void forgetCloseDoorInvade(const chrono::system_clock::time_point& time) {
		cout << to_string(time) << "����ʱ���ѳ���" << 
			invade_once_timeout / 1000 << "����" <<
			",δ����,������¼��" << endl;
		sendQQ("[Warning] Door is opening [invade timeout]");
	}
	void forgetCloseDoorGoHomeContinue(const chrono::
		system_clock::time_point& time) {

		cout << to_string(time) << "(GoHomeContinue)ʱ���ѳ���" <<
			invade_once_timeout / 1000 << "����" <<
			",δ����,������¼��" << endl;
		sendQQ("[Warning] Door is opening [GoHomeContinue timeout]");
	}

	//FaceDetectorģ�����
	void invade(const chrono::system_clock::time_point & time) {
		cout << to_string(time) << "@@@ ע�⣺�����¼�����" << endl;
		sendQQ("[Alarm] Intrusion Event Occured");
	}
	//FaceDetectorģ�����
	void gohome(const chrono::system_clock::time_point& time) {
		cout << to_string(time) << "��ӭ�ؼ�" << endl;
		sendQQ("[Info] Welcome to return home");
	}

	void doorOpenedIn(const chrono::system_clock::time_point& time) {
		cout << to_string(time) << "Warning�� �ű���" << endl;
		sendQQ("[Warning] Door is opened");
	}
	void doorOpenedLeaveHome(const chrono::system_clock::time_point& time) {
		cout << to_string(time) << "�����ڿ���" << endl;
		sendQQ("[Info] Good bye take care");
	}


	void doorClosedNoEvent(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< "���ѹر�(�򿪳�ʱ),��ʱ��"
			<< (double)time_count / 1000 << "s" << endl;
		sendQQ("[Info] Door is closed [timeout]");
	}
	void doorClosedIn(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< "���ѹر�(����ʶ������),��ʱ��"
			<< (double)time_count / 1000 << "s" << endl;
		//sendQQ("[Info] Door is closed [detectng face]");
	}
	void doorClosedInvade(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< " �����¼�:���ѹر�,��ʱ��"
		 << (double)time_count / 1000 << "s" << endl;
		//sendQQ("[Warning] Door is closed [intrusion]");
	}

	void doorClosedGoHome(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< " �ؼ��¼�:���ѹر�,��ʱ��"
			<< (double)time_count / 1000 << "s" << endl;
		//sendQQ("[Info] Door is closed");
	}
	void doorClosedGoHomeContinue(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< " ���ѹر�,��ʱ��"
			<< (double)time_count / 1000 << "s" << endl;
	}
	
	void doorClosedLeaveHome(const chrono::system_clock::time_point& time,
		long long time_count) {
		cout << to_string(time)
			<< " �����¼�:���ѹر�,��ʱ��"
			<< (double)time_count / 1000 << "s" << endl;
	}

	void OpenedAgainInvade(const chrono::system_clock::time_point& time) {
		cout << to_string(time)
			<< " �����¼�:���ٴα���"<< endl;
		sendQQ("[Alarm] Door is opened again");

	}
	void OpenedAgainIn(const chrono::system_clock::time_point& time) {
		//cout << to_string(time)
		//	<< " ���ٴα���" << endl;
	}
	void OpenedAgainAfterGoHome(const chrono::system_clock::time_point& time) {
		cout << to_string(time)
			<< "Warning: ���ٴα���" << endl;
		sendQQ("[Warning] Door is opened again");
	}


	bool door_opened = false;

	void do_cmd(std::istringstream& issm);
	void cmd_set(istringstream& issm);
	void show_param(istringstream& issm);


	//�������ʧ��������Ϊ����¼������ַ�ʽ����©���������Ľ�
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