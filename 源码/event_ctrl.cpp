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
	cout << "暂不支持其他的判断离家事件方式" << endl;
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
				
				//入侵状态下的后续事件均视为入侵
			case EventInvade:
				OpenedAgainInvade(time_lastEvent_begin);
				//在人脸识别阶段内再次开门，
				//始终不作为出门事件
				break;
			case EventIn:
				OpenedAgainIn(time_lastEvent_begin);
				break;
			
				//情况特殊，因为回家后遮挡的状态不能马上清除，
				//无法分辨出门、再次回家或是有入侵，将作为复合事件通知用户
				//始终放在event，但文件名会包含警示前缀
			case EventGoHome:
				status = EventGoHomeContinue;
				OpenedAgainAfterGoHome(time_lastEvent_begin);

				event_is_multi = true;

				break;

				//离开后遮挡状态立刻设置回空闲，直接进行下一轮判断
				//事件类型为复合，后续检测到入侵则放exception，否则event
			case EventLeaveHome:

				judge_doorOpened();
				event_is_multi = true;
				break;

			case EventGoHomeContinue:
				OpenedAgainAfterGoHome(time_lastEvent_begin);
				break;
			default:
				cout << "错误的事件类型，程序错误" << endl;
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
			//  case idle:  原始情况
		case shelt:
			if (++counter_leave > counter_leave_max) {
				counter_leave = 0;
				intra_status = leave;
				is_shelt_debug_over = false;
				event_leave_begin = chrono::system_clock::now();
#ifdef shelt_is_leaveHome_style
				if(is_debug_shelt)
					cout << to_string(now) << "遮挡物暂时离开" << endl;
#else
				if (is_debug_shelt)
					cout << time_buffer << "标记重新出现" << endl;

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
					cout << to_string(now) << "遮挡物已离开" << endl;
#else
				if (is_debug_shelt)
					cout << time_buffer << "标记识别恢复正常" << endl;
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
				cout << time_buffer << "有室内物体遮挡" << endl;
#else
			if (is_debug_shelt)
				cout << time_buffer << "标记消失" << endl;
#endif // shelt_is_leaveHome_style

			break;
		case shelt:
			counter_leave = 0;
			break;
		case leave:	//重新遮挡
			intra_status = shelt;
			event_shelt_begin = chrono::system_clock::now();
#ifdef shelt_is_leaveHome_style
			if (is_debug_shelt)
				cout << time_buffer << "重新遮挡" << endl;
#else
			if (is_debug_shelt)
				cout << time_buffer << "标记再次消失" << endl;
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
			cout << "!!!!!! 出现bug，不应该同时 mark_exist(last)"
				"==false && intra_event==idle" << endl;
			exit(-1);
		case shelt:	//遮挡中..
			param_timeout_mutex.lock();
			if (chrono::duration_cast<chrono::milliseconds>
				(time).count() > shelt_timeout && !is_shelt_debug_over) {
				is_shelt_debug_over = true;
				cout << time_buffer << 
					"因遮挡或其他异常情况，使得标记消失时间过长" << endl;
			}
			param_timeout_mutex.unlock();
			break;
		case leave:
			cout << "!!!!!! 出现bug，不应该同时 mark_exist(last)"
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
		cout << "格式错误" << endl;
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
		cout << "<1>入侵事件中门打开的超时时间       [ 单位:ms ]   (invade_once_timeout)   "
			<< setw(10) << invade_once_timeout << endl;
		cout << "<2>普通事件中门打开的超时时间       [ 单位:ms ]   (safeEvent_timeout)     "
			<< setw(10) << safeEvent_timeout << endl;
		cout << "<3>遮挡物离开的超时时间             [ 单位:ms ]   (leave_timeout)         "
			<< setw(10) << leave_timeout << endl;
		cout << "<4>遮挡物遮挡的超时时间             [ 单位:ms ]   (shelt_timeout)         "
			<< setw(10) << shelt_timeout << endl;
		cout << "<5>普通事件结束后继续记录的时间     [ 单位:ms ]   (event_end_record_time) "
			<< setw(10) << event_end_record_time << endl;
		cout << "<6>入侵事件结束后继续记录的时间     [ 单位:ms ](exception_end_record_time)"
			<< setw(10) << exception_end_record_time << endl;
		param_timeout_mutex.unlock();
	}
	else if (param == "help" || param == "-h") {

	}
	else {
		cout << "暂不支持个别参数查看方式" << endl;
	}
}