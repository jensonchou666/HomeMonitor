#include "event_ctrl.h"

void EventCtrl::opening()
{
	if (is_open_timeout)return;
	

	chrono::system_clock::time_point now
		= chrono::system_clock::now();
	time_event_begin_mutex.lock_shared();
	chrono::system_clock::duration time = now - 
		time_lastEvent_begin;
	time_event_begin_mutex.unlock_shared();
	long long time_count = chrono::duration_cast<chrono::milliseconds>
		(time).count();

	event_mutex.lock();
	bool ret = false;

	param_timeout_mutex.lock();
	switch (status) {
	case EventInvade:
		if (time_count > invade_once_timeout) {
			ret = true;
			forgetCloseDoorInvade(now);
		}
		break;
	case EventGoHome:
		if (time_count > safeEvent_timeout) {
			forgetCloseDoorGoHome(now);
			ret = true;
		}
	case EventLeaveHome :
		if (time_count > safeEvent_timeout) {
			forgetCloseDoorLeaveHome(now);
			ret = true;
		}
		break;

	case EventGoHomeContinue:
		if (time_count > invade_once_timeout) {
			forgetCloseDoorGoHomeContinue(now);
			ret = true;
		}
		break;
	}
	if (ret) {
		event_end = true;
		time_event_end = now;
		is_open_timeout = true;
		event_is_timeout = true;
	}
	param_timeout_mutex.unlock();

	event_mutex.unlock();
}

bool EventCtrl::is_event_record_timeout(const chrono::system_clock::time_point& now)
{


	//cout << "@@@ " << time_event_end.time_since_epoch().count()
	//	<< "," << event_ctrl.time_event_end.time_since_epoch().count()<<endl;
	time_event_end_mutex.lock_shared();
	chrono::system_clock::duration duration	= now - 
		time_event_end;
	time_event_end_mutex.unlock_shared();

	long long time_count = chrono::duration_cast<chrono::milliseconds>
		(duration).count();
	param_timeout_mutex.lock();
	bool ret;
	if (status == EventInvade) 
		ret=time_count >= exception_end_record_time;
	else 
		ret=time_count >= event_end_record_time;
	param_timeout_mutex.unlock();
	return ret;
}


void EventCtrl::judge_doorOpened() {

#ifdef shelt_is_leaveHome_style
	switch (intra_status) {
	case idle:
		doorOpenedIn(time_lastEvent_begin);
		status = EventIn;
		break;
	case shelt:
	case leave:
		doorOpenedLeaveHome(time_lastEvent_begin);
		status = EventLeaveHome;
		break;
	}
#else
	cout << "�ݲ�֧���������ж�����¼���ʽ" << endl;
#endif // shelt_is_leaveHome_style

}


void EventCtrl::doorOpening()
{


	if (door_opened) {
		opening();
	}
	else {

		door_opened = true;
		mark_exist = true;
	

		event_mutex.lock();

		if (status != NoEvent) {
			time_event_begin_mutex.lock();
			time_lastEvent_begin  =	chrono::system_clock::now();
			
			switch (status) {
				
				//����״̬�µĺ����¼�����Ϊ����
			case EventInvade:
				OpenedAgainInvade(time_lastEvent_begin);
				//������ʶ��׶����ٴο��ţ�
				//ʼ�ղ���Ϊ�����¼�
				break;
			case EventIn:
				OpenedAgainIn(time_lastEvent_begin);
				break;
			
				//������⣬��Ϊ�ؼҺ��ڵ���״̬�������������
				//�޷��ֱ���š��ٴλؼһ��������֣�����Ϊ�����¼�֪ͨ�û�
				//ʼ�շ���event�����ļ����������ʾǰ׺
			case EventGoHome:
				status = EventGoHomeContinue;
				OpenedAgainAfterGoHome(time_lastEvent_begin);

				event_is_multi = true;

				break;

				//�뿪���ڵ�״̬�������ûؿ��У�ֱ�ӽ�����һ���ж�
				//�¼�����Ϊ���ϣ�������⵽�������exception������event
			case EventLeaveHome:

				judge_doorOpened();
				event_is_multi = true;
				break;

			case EventGoHomeContinue:
				OpenedAgainAfterGoHome(time_lastEvent_begin);
				break;
			default:
				cout << "������¼����ͣ��������" << endl;
				exit(-1);
			}
			time_event_begin_mutex.unlock();
			event_end = false;
			faceDetect_over = false;
			is_open_timeout = false;

			event_mutex.unlock();
			return;
		}

		time_event_begin_mutex.lock();
		time_lastEvent_begin=time_event_begin = 
			chrono::system_clock::now();
		time_event_begin_mutex.unlock();

		event_end = false;
		faceDetect_over = false;
		is_open_timeout = false;

		event_begin = true;
		//event_continue = false;
		event_is_multi = false;

		judge_doorOpened();

		event_mutex.unlock();
	}

}

bool is_shelt_debug_over = false;

void EventCtrl::doorClosing() {
	
	if (door_opened) {

		event_mutex.lock();


		intra_status = idle;
		if (is_open_timeout)
			is_open_timeout = false;
		else
			event_end = true;

		time_event_end_mutex.lock();
		time_event_end= chrono::system_clock::now();
		time_event_begin_mutex.lock_shared();
		chrono::system_clock::duration time = time_event_end - 
			time_lastEvent_begin;
		time_event_begin_mutex.unlock_shared();
		long long time_count = chrono::duration_cast<chrono::milliseconds>
			(time).count();
		switch (status) {
		case NoEvent:
			doorClosedNoEvent(time_event_end, time_count);
			break;
		case EventIn:
			doorClosedIn(time_event_end, time_count);
			break;
		case EventInvade:
			doorClosedInvade(time_event_end, time_count);
			break;
		case EventLeaveHome:
			doorClosedLeaveHome(time_event_end, time_count);
			break;
		case EventGoHome:
			doorClosedGoHome(time_event_end, time_count);
			break;
		case EventGoHomeContinue:
			doorClosedGoHomeContinue(time_event_end, time_count);
			break;
		}
		time_event_end_mutex.unlock();
		event_mutex.unlock();
	}
	else {

		auto now = chrono::system_clock::now();
		switch (intra_status) {
			//  case idle:  ԭʼ���
		case shelt:
			if (++counter_leave > counter_leave_max) {
				counter_leave = 0;
				intra_status = leave;
				is_shelt_debug_over = false;
				event_leave_begin = chrono::system_clock::now();
#ifdef shelt_is_leaveHome_style
				if(is_debug_shelt)
					cout << to_string(now) << "�ڵ�����ʱ�뿪" << endl;
#else
				if (is_debug_shelt)
					cout << time_buffer << "������³���" << endl;

#endif // shelt_is_leaveHome_style
			
			}
			break;
		case leave:
			chrono::system_clock::time_point now
				= chrono::system_clock::now();
			chrono::system_clock::duration time = now - event_leave_begin;
			param_timeout_mutex.lock();
			if (chrono::duration_cast<chrono::milliseconds>
				(time).count() > leave_timeout) {
				intra_status = idle;
#ifdef shelt_is_leaveHome_style
				if (is_debug_shelt)
					cout << to_string(now) << "�ڵ������뿪" << endl;
#else
				if (is_debug_shelt)
					cout << time_buffer << "���ʶ��ָ�����" << endl;
#endif // shelt_is_leaveHome_style
				
			}
			param_timeout_mutex.unlock();
		}
	}
	door_opened = false;
	mark_exist = true;
}

void EventCtrl::markDisapear() {
	if (door_opened) {
		opening();
	}
	else if (mark_exist) {
		event_shelt_begin = chrono::system_clock::now();
		string time_buffer= to_string(event_shelt_begin);

		switch (intra_status) {
		case idle:
			intra_status = shelt;
#ifdef shelt_is_leaveHome_style
			if (is_debug_shelt)
				cout << time_buffer << "�����������ڵ�" << endl;
#else
			if (is_debug_shelt)
				cout << time_buffer << "�����ʧ" << endl;
#endif // shelt_is_leaveHome_style

			break;
		case shelt:
			counter_leave = 0;
			break;
		case leave:	//�����ڵ�
			intra_status = shelt;
			event_shelt_begin = chrono::system_clock::now();
#ifdef shelt_is_leaveHome_style
			if (is_debug_shelt)
				cout << time_buffer << "�����ڵ�" << endl;
#else
			if (is_debug_shelt)
				cout << time_buffer << "����ٴ���ʧ" << endl;
#endif // shelt_is_leaveHome_style
			
			break;
		}
	}
	else {
		chrono::system_clock::time_point now
			= chrono::system_clock::now();
		chrono::system_clock::duration time = now - event_shelt_begin;
		string time_buffer=to_string(now);
		
		switch (intra_status) {
		case idle:
			cout << "!!!!!! ����bug����Ӧ��ͬʱ mark_exist(last)"
				"==false && intra_event==idle" << endl;
			exit(-1);
		case shelt:	//�ڵ���..
			param_timeout_mutex.lock();
			if (chrono::duration_cast<chrono::milliseconds>
				(time).count() > shelt_timeout && !is_shelt_debug_over) {
				is_shelt_debug_over = true;
				cout << time_buffer << 
					"���ڵ��������쳣�����ʹ�ñ����ʧʱ�����" << endl;
			}
			param_timeout_mutex.unlock();
			break;
		case leave:
			cout << "!!!!!! ����bug����Ӧ��ͬʱ mark_exist(last)"
				"==false && intra_event==leave" << endl;
			exit(-1);
		}
	}
	mark_exist = false;

}


void EventCtrl::do_cmd(std::istringstream& issm) {

}
void EventCtrl::cmd_set(istringstream& issm) {
	int param=0;
	long value=0;

	if (issm)issm >> param;
	if (issm)issm >> value;

	if (param == 0 || value == 0) {
		cout << "��ʽ����" << endl;
		return;
	}
	param_timeout_mutex.lock();
	switch (param) {
	case 1:
		invade_once_timeout = value;
		break;
	case 2:
		safeEvent_timeout = value;
		break;
	case 3:
		leave_timeout = value;
		break;
	case 4:
		shelt_timeout = value;
		break;
	case 5:
		event_end_record_time = value;
		break;
	case 6:
		exception_end_record_time = value;
		break;
	}
	param_timeout_mutex.unlock();

}
void EventCtrl::show_param(istringstream& issm) {
	string param;
	if (issm)issm >> param;
	if (param.empty() || param == "all") {
		cout << "Face detector's params :" << std::endl;
		cout.setf(ios::right);
		
		param_timeout_mutex.lock();
		cout << "<1>�����¼����Ŵ򿪵ĳ�ʱʱ��       [ ��λ:ms ]   (invade_once_timeout)   "
			<< setw(10) << invade_once_timeout << endl;
		cout << "<2>��ͨ�¼����Ŵ򿪵ĳ�ʱʱ��       [ ��λ:ms ]   (safeEvent_timeout)     "
			<< setw(10) << safeEvent_timeout << endl;
		cout << "<3>�ڵ����뿪�ĳ�ʱʱ��             [ ��λ:ms ]   (leave_timeout)         "
			<< setw(10) << leave_timeout << endl;
		cout << "<4>�ڵ����ڵ��ĳ�ʱʱ��             [ ��λ:ms ]   (shelt_timeout)         "
			<< setw(10) << shelt_timeout << endl;
		cout << "<5>��ͨ�¼������������¼��ʱ��     [ ��λ:ms ]   (event_end_record_time) "
			<< setw(10) << event_end_record_time << endl;
		cout << "<6>�����¼������������¼��ʱ��     [ ��λ:ms ](exception_end_record_time)"
			<< setw(10) << exception_end_record_time << endl;
		param_timeout_mutex.unlock();
	}
	else if (param == "help" || param == "-h") {

	}
	else {
		cout << "�ݲ�֧�ָ�������鿴��ʽ" << endl;
	}
}