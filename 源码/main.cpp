
#include "global.h"
#include "camera.h"
#include "door_monitor.h"
#include "face_detector.h"
using namespace std;


void command(Global* global, Camera* camera, DoorMonitor* door_monitor,
	FaceDetector* face_detector, EventCtrl* event_ctrl);

//删除耗时，开新线程
void history_clear(Global * global);
void temp_clear(Global* global);

void exitProc(int ret = -1);

void create_history();
void create_temp();
void create_event_rest();
void release_history();
void release_temp();

void init_camera();
void stop_proc();
void reload();

void cheak_recordEnd_beforeNewRelease();
void record_event_rest(const cv::Mat & pic);
void event_record_end();

static atomic_bool   is_continue = true;
static atomic_bool   is_history_continue = true;
static atomic_bool   is_cmd_sleep = false;
bool event_begin_record_new = false;
bool do_event_record_rest = false;

cv::VideoWriter history_writer;
cv::VideoWriter temp_writer;
cv::VideoWriter event_rest_writer;
time_t time_temp_begin, time_history_begin;

cv::String fNameNew;
cv::String fNameOld;
cv::String fNameTMP;




//std::pair< DoorMonitor*, Camera*> pair_mouse_data =
//std::pair< DoorMonitor*, Camera*>(&door_monitor,&camera);
int main()
{
	/* initiate */
	init_camera();
	global.initDirectory();
	global.initAviFiles();

	door_monitor.init(&proc_ctrl, &camera,&event_ctrl, &rect_detect_parameters);
	face_detector.init(&proc_ctrl, &camera, &event_ctrl,&door_monitor);

	cv::namedWindow(main_win_name, cv::WINDOW_NORMAL);
	cv::resizeWindow(main_win_name, camera.size);
	cv::setMouseCallback(main_win_name,on_mouse_mainwin,(void*)&door_monitor);



	thread(command,&global,&camera,&door_monitor,
		&face_detector,&event_ctrl).detach();
	thread(&DoorMonitor::run, &door_monitor).detach();
	thread(&FaceDetector::run, &face_detector).detach();

	bool flag_lastframe_stoped = false;

	cv::Mat pic;
	while (is_continue) {
		time_t now = time(0);
		camera.cap >> pic;
		
		if (pic.empty()) {
			cout << "!!!!!!! Frame is empty" << endl;
			exitProc();
		}
		if (proc_ctrl.do_reload) {
			reload(); proc_ctrl.do_reload = false;
			cout << "√ Reloaded the progress" << endl << prompt;
		}
		if (proc_ctrl.stoped) {
			if (!flag_lastframe_stoped) {		//首次暂停
				flag_lastframe_stoped = true;
				stop_proc();
				cout << "√ Stoped the progress" << endl << prompt;
			}
		}
		else  {
			if (flag_lastframe_stoped) {			//首次开始
				flag_lastframe_stoped = false;
				create_history();
				create_temp();
				cout << "√ Started the progress" << endl << prompt;
			}

			cam_current_mutex.lock();
			camera.current = pic.clone();
			camera.frame_counter++;
			cam_current_mutex.unlock();
			

			global.param_mutex.lock_shared();
			// New History 
			if ((now - time_history_begin) > global.history_length
				|| !is_history_continue) {
				if (!is_continue) {
					global.param_mutex.unlock_shared();
					break;
				}
				release_history();
				create_history();
				cout << prompt;
			}
			// New Temp
			if ((now - time_temp_begin) > global.temp_length) {
				if (!is_continue) {
					global.param_mutex.unlock_shared();
					break;
				}

				if (event_begin_record_new) {
					//cout << "@@@ record_new," <<
					//	fNameNew.c_str() << " " << global.fNameEventNew << endl;
	
					event_begin_record_new = false;
					temp_writer.release();					
					rename(fNameNew.c_str(), 
global.fNameEventNew.c_str());
					//New Event Rest
					create_event_rest();
					do_event_record_rest = true;
				}
				else {
					release_temp();
				}
				create_temp();
			}
			global.param_mutex.unlock_shared();

			/*
			static Timer timer1(500);
			timer1.end(
				to_string(event_ctrl.status) + " " +
				to_string(event_ctrl.event_begin) + " " +
				to_string(event_ctrl.event_end));
			*/
			// New Event
			if (event_ctrl.event_begin) {
				event_ctrl.event_begin = false;
				event_begin_record_new = true;
				rename(fNameOld.c_str(),global.fNameEventOld.c_str());
			}
			cheak_recordEnd_beforeNewRelease();


			if (door_monitor.is_save)
				door_monitor.when_showPic(pic);
			if (face_detector.is_save)
				face_detector.when_showPic(pic);

			temp_writer << pic;
			history_writer << pic;
			if (do_event_record_rest)
				record_event_rest(pic);

		}

		if(!door_monitor.is_save) 
			door_monitor.when_showPic(pic);

		if (!face_detector.is_save)
			face_detector.when_showPic(pic);

		cv::imshow(main_win_name, pic);
		char c = cv::waitKey(camera.fps / 6);
		if (c == 'a'|| c == 'A')
			is_history_continue = false;
		else if (c == 'q'|| c == 'Q') {
			is_continue = false;
			is_history_continue = false;
		}

		bool is_reload=false;
		camera.command_set(is_reload);
		if (!proc_ctrl.stoped && is_reload)reload();


	}	//while

	exitProc(0);
	return 0;
}

void on_mouse_mainwin(int event, int x, 
	int y, int flag, void* data) {

	((DoorMonitor*)data)->on_mouse_selrange(event, x, y);
}




void deleteAllTmp() {
	global.eventFile_mutex.lock();
	system((string("del /Q ") + global.temp_path + "\\*.*").c_str());
	global.eventFile_mutex.unlock();
}

void stop_wait() {
	proc_ctrl.stop_begin();
	cout << "please wait..." << endl;
	while (proc_ctrl.wait_counter != 0) {
		std::this_thread::sleep_for(chrono::milliseconds(200));
	}
	proc_ctrl.stop_finished();
}
void stop_event_ctrl() {
	event_begin_record_new = false;
	do_event_record_rest = false;
	event_ctrl.reset();
}

void reload() {
	proc_ctrl.stoped = true;
	is_cmd_sleep = true;
	stop_wait();
	//temp_writer.release();
	event_rest_writer.release();
	//deleteAllTmp();
	stop_event_ctrl();

	//create_temp();
	is_cmd_sleep = false;
	proc_ctrl.stoped = false;
}

void stop_proc() {
	is_cmd_sleep = true;
	stop_wait();
	release_history();
	temp_writer.release();
	event_rest_writer.release();
	deleteAllTmp();
	stop_event_ctrl();
	is_cmd_sleep = false;
}

void exitProc(int ret) {
	stop_proc();
	camera.cap.release();
	exit(ret);
}
void release_history() {
	is_history_continue = true;
	history_writer.release();
	std::thread(history_clear,&global).detach();
}
void release_temp() {
	temp_writer.release();
	//frame_counter = 0;
	if (rename(fNameNew.c_str(), fNameTMP.c_str()))
		cout << "New->TMP重命名失败" << endl << prompt;
	thread(temp_clear,&global).detach();
}
void create_history() {
	time_history_begin = time(0);

	char filename[MAX_FILENAME_LEN];
	char time_str[MAX_TIMEBUFFER_LEN];
	memset(filename, 0, MAX_FILENAME_LEN);
	memset(time_str,0, MAX_TIMEBUFFER_LEN);
	to_string(&time_history_begin, time_str, MAX_TIMEBUFFER_LEN);
	sprintf_s(filename, "%s\\%s.avi",
		global.history_path.c_str(),time_str);
	camera.param_mutex.lock();
	if (history_writer.open(filename, CV_FOURCC('M', 'J', 'P', 'G'),
		camera.fps,camera.size))
		cout << "New avi history file has been created:" << filename <<
		endl;

	else {
		cout << "History video writer open failed." << endl;
		exitProc();
	}
	camera.param_mutex.unlock();
}

void create_temp() {
	time_temp_begin = time(0);
	camera.param_mutex.lock();
	if (!temp_writer.open(fNameNew, CV_FOURCC('M', 'J', 'P', 'G'),
		camera.fps, camera.size)) {
		cout << "Temp video writer open failed." << endl;
		exitProc();
	}
	camera.param_mutex.unlock();
}
void create_event_rest() {
	camera.param_mutex.lock();
	event_rest_writer.open(
		global.fNameEventRest, CV_FOURCC('M', 'J', 'P', 'G'),
		camera.fps, camera.size);
	camera.param_mutex.unlock();
}



using namespace std;

/*
string cmd_buf;
std::istringstream issm;
string& next() {
	issm >> cmd_buf;
	return cmd_buf;
}
*/

void command(Global * global,Camera * camera,DoorMonitor *door_monitor,
	FaceDetector * face_detector,EventCtrl *event_ctrl) {

	cout << prompt;
#define CMD_BUFFER_SIZE		2000
	char cmd_buffer[CMD_BUFFER_SIZE];

	this_thread::sleep_for(chrono::microseconds(1000));
	door_monitor->set_range();


	while (1) {
		cin.getline(cmd_buffer, CMD_BUFFER_SIZE);

		while (is_cmd_sleep) {
			std::this_thread::sleep_for(chrono::milliseconds(200));
		}
		//xCon_working.wait();
		std::string cmd(cmd_buffer);
		std::istringstream issm(cmd);
		std::string cmd_f;
		issm >> cmd_f;

		if (cmd_f == "help") {
			cout << "a/save               -- 保存当前录像，开启新录像" << endl;
			cout << "q/quit               -- 保存当前录像，结束程序" << endl;
			cout << "start/stop/reload    -- 开始/暂停/重启程序" << endl;
			cout << "param                -- 查看模块参数(不指定参数则默认全参数)" << endl;
			cout << "set                  -- 设置模块参数(带W的参数为可写,请在param{} help中查看)" << endl;
			//cout << endl;
			cout << "dm                   -- door_monitor  门状态监听模块" << endl;
			cout << "fd                   -- face_detector 人体目标识别模块" << endl;
			cout << "ec                   -- event_control 事件控制模块" << endl;
			cout << "cls                  -- 清屏" << endl;
			cout << "help                 -- 帮助，打开本菜单" << endl<<endl;
		}
		else if (cmd_f == "a" || cmd_f == "save") {
			is_history_continue = false;
		}
		else if (cmd_f == "q" || cmd_f == "quit") {
			is_continue = false;
			is_history_continue = false;
		}
		else if (cmd_f == "cls")
			system("cls");
		else if (cmd_f == "start" && proc_ctrl.stoped) {
			proc_ctrl.stoped = false; 
			continue;
		}
		else if (cmd_f == "stop" && !proc_ctrl.stoped) {
			proc_ctrl.stoped = true; 
			continue;
		}
		else if (cmd_f == "reload" && !proc_ctrl.stoped) {
			proc_ctrl.do_reload = true; 
			continue;
		}
		/*------------------------param------------------------------*/
		else if (cmd_f == "param") {
			string arg1;
			if (issm)issm >> arg1;
			if (arg1.empty() || arg1 == "all") {
				global->show_params();
				cout << "----------------" << endl;
				camera->show_params();
				cout << "----------------" << endl;
				door_monitor->show_param(issm);
				cout << "----------------" << endl;
				face_detector->show_param(issm);
				cout << "----------------" << endl;
				event_ctrl->show_param(issm);
			}
			else if (arg1 == "g" || arg1 == "global") {
				string param;
				if (issm)issm >> param;
				global->show_param(param);
			}
			else if (arg1 == "cam"|| arg1 == "camera") {
				string param;
				if(issm)issm >> param;
				camera->show_param(param);
			}
			else if (arg1 == "dm" || arg1 == "door_moniter") {
				door_monitor->show_param(issm);
			}
			else if (arg1 == "fd" || arg1 == "face_detector") {
				face_detector->show_param(issm);
			}
			else if (arg1 == "ec" || arg1 == "event_ctrl") {
				event_ctrl->show_param(issm);
			}
			else if (arg1 == "help" || arg1 == "-h") {
				cout << "global parameters           (global/g)" << endl;
				cout << "camera parameters           (camera/cam)" << endl;
				cout << "door moniter parameters     (door_moniter/dm)" << endl;
				cout << "face detector parameters    (face_detector/fd)" << endl;
				cout << "event ctrl parameters       (event_ctrl/ec)" << endl;
			}
			else  cout << "错误指令" << endl;
		}
		/*--------------------------set------------------------------*/
		else if (cmd_f == "set") {
			string arg1;
			if (issm)issm >> arg1;
			if (arg1 == "g" || arg1 == "global") {
				global->cmd_set(issm);
			}
			else if (arg1 == "cam" || arg1 == "camera") {	
				char  s [200]="\0";
				if (issm)issm.getline(s,200);
				string set_string(s);
				camera->push_cmd_set(set_string);
				memset(cmd_buffer, 0, 1024);
				continue;
			}
			else if (arg1 == "dm" || arg1 == "door_moniter") {
				door_monitor->cmd_set(issm);
			}
			else if (arg1 == "fd" || arg1 == "face_detector") {
				face_detector->cmd_set(issm);
			}
			else if (arg1 == "ec" || arg1 == "event_ctrl") {
				event_ctrl->cmd_set(issm);
			}
			else if (arg1 == "help" || arg1 == "-h") {
				cout << "global parameters           (global/g)" << endl;
				cout << "camera parameters           (camera/cam)" << endl;
				cout << "door moniter parameters     (door_moniter/dm)" << endl;
				cout << "face detector parameters    (face_detector/fd)" << endl;
				cout << "event ctrl parameters       (event_ctrl/ec)" << endl;
			}
			else  cout << "错误指令" << endl;
		}
		/*----------------------------------------------------------*/
		else if (cmd_f == "dm") {
			door_monitor->do_cmd(issm);
		}
		else if (cmd_f == "fd") {
			face_detector->do_cmd(issm);
		}
		else if (cmd_f == "ec") {
			event_ctrl->do_cmd(issm);
		}
		else if(!cmd_f.empty())cout << "错误指令"<<endl;
		


		memset(cmd_buffer, 0, 1024);
		cout << prompt;
	}

}


//开新线程
void history_clear(Global * global) {
	//	sprintf_s(filename, "%s%s.avi", history_path.c_str(),time_str);
	//cout << "@1" << endl;

	char dir[MAX_FILENAME_LEN];
	memset(dir, 0, MAX_FILENAME_LEN);
	sprintf_s(dir, MAX_FILENAME_LEN, "%s\\*.avi", 
		global->history_path.c_str());

	WIN32_FIND_DATA wfd;
	HANDLE hFind;
	WCHAR dirName[MAX_FILENAME_LEN];
	int len = AnsiToUnicode(dir, dirName,MAX_FILENAME_LEN);
	if (len <= 0)return;
	hFind = FindFirstFile(dirName, &wfd);

	if (hFind == INVALID_HANDLE_VALUE) {
		FindClose(hFind);
		return;
	}
	char file_name[MAX_FILENAME_LEN];
	//char time_str[MAX_TIMEBUFFER_LEN];
	while (1) {
		if (wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
			if (!FindNextFile(hFind, &wfd))break; continue;
		}

		memset(file_name, 0, MAX_FILENAME_LEN);
		len = UnicodeToAnsi(wfd.cFileName, file_name, MAX_FILENAME_LEN);
		if (len <= 0) {
			if (!FindNextFile(hFind, &wfd))break; continue;
		}
		//memset(time_str, 0, MAX_TIMEBUFFER_LEN);
		//vsscanf_s<MAX_FILENAME_LEN>(file_name, "%s.avi", );
		//sscanf_s(file_name,"%s.avi", time_string);
		//time_t ftime=to_time_t(to_string(chrono::system_clock::now()).c_str());

		string time_string = file_name;
		time_string = time_string.substr(0, time_string.size() - 4);		
		time_t ftime = to_time_t(time_string.c_str());
		global->param_mutex.lock_shared();
		//cout << global->history_path << file_name << " " << ftime << endl;
		if (ftime != (time_t)-1 && (time(0)-ftime)>
			global->history_time_span * 60) {
			deleteFile(global->history_path+ "\\"+file_name);
		}
		global->param_mutex.unlock_shared();
		if (!FindNextFile(hFind, &wfd))break;
	}
	FindClose(hFind);
	cout << "1" << endl;
}

void temp_clear(Global * global) {
	deleteFile(fNameOld);
	rename(fNameTMP.c_str(), fNameOld.c_str());
}

void init_camera() {
	cv::Mat  tmp;
	bool ret = camera.init();
	if (ret)camera.cap >> tmp;
	if (tmp.empty()) {
		cout << "摄像头打开失败,是否更换设备号？";
		if (!affirm_defN())
			exitProc();
		else {
			cout << "设备号:";
			int id;
			cin >> id;
			ret = camera.open(id);
			if (ret)camera.cap >> tmp;
			if (tmp.empty()) {
				cout << "摄像头打开失败" << endl;
				exitProc();
			}
		}
	}
	camera.show_params();
	//cout << "是否设置摄像头分辨率？";
}

bool dirExist(const char* name) {
	WIN32_FIND_DATA wfd;

	WCHAR  dir[MAX_FILENAME_LEN];
	MultiByteToWideChar(CP_ACP, 0, name, strlen(name) + 1, dir,
		sizeof(dir) / sizeof(dir[0]));
	HANDLE hFind = FindFirstFile(dir, &wfd);
	if ((hFind != INVALID_HANDLE_VALUE) &&
		(wfd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
		FindClose(hFind);
		return true;
	}
	FindClose(hFind);
	return false;
}
inline bool dirExist(std::string name) {
	return dirExist(name.c_str());
}
void createDir(const char* name) {
	WCHAR  dir[MAX_FILENAME_LEN];
	MultiByteToWideChar(CP_ACP, 0, name, strlen(name) + 1, dir,
		sizeof(dir) / sizeof(dir[0]));
	CreateDirectory(dir, NULL);
}
void createDir(std::string name) {
	createDir(name.c_str());
}
void createDir(cv::String name) {
	createDir(name.c_str());
}
void Global::initDirectory() {

//#include <Windows.h>

	if (!dirExist(Root)) {
		cout << endl << "未找到默认ROOT目录：" << Root << endl
			<< "请手动输入ROOT目录:：";
		cin >> Root;
		if (!dirExist(Root)) {
			camera.cap.release();
			cout << "未找到该目录" << endl;
			exit(-1);
		}
	}
	history_path = Root + "\\" + history_path;
	exception_path= Root + "\\" + exception_path;
	event_path = Root + "\\" + event_path ;
	temp_path = Root + "\\" + temp_path;
	face_detector.haarcascadesDir = Root + "\\" +
		face_detector.haarcascadesDir;
	createDir(history_path);
	createDir(exception_path);
	createDir(event_path);
	createDir(temp_path);
	createDir(face_detector.haarcascadesDir);

}


void  Global::initAviFiles() {
	fNameNew		=		temp_path +	"\\" + "New.avi";
	fNameOld		=		temp_path + "\\" + "Old.avi";
	fNameEventOld	=		temp_path + "\\" + "EventOld.avi";
	fNameEventNew	=		temp_path + "\\" + "EventNew.avi";
	fNameEventRest	=		temp_path + "\\" + "EventRest.avi";
	fNameTMP		=		temp_path + "\\" + "18382135956.avi";
	fNameDelete = temp_path + "\\" + "delete";
	AnsiToUnicode(fNameDelete.c_str(), f_name_delete,MAX_FILENAME_LEN);
	
	system((string("del /Q ") + temp_path+"\\*.*").c_str());

	create_history();
	create_temp();
}


int16_t event_status = 0;
bool event_is_multi = false;
bool event_is_timeout = false;
chrono::system_clock::time_point time_event_begin=chrono::system_clock::time_point();

bool eventRecordEnd() {
	event_ctrl.event_mutex.lock();
	bool ret = event_ctrl.event_end && (event_ctrl.status != EventIn) && 
		event_ctrl.is_event_record_timeout(chrono::system_clock::now());
	if (ret) {
		event_status = event_ctrl.status;
		event_is_multi = event_ctrl.event_is_multi;
		event_is_timeout = event_ctrl.event_is_timeout;
		event_ctrl.time_event_begin_mutex.lock_shared();
		time_event_begin = event_ctrl.time_event_begin;
		event_ctrl.time_event_begin_mutex.unlock_shared();
		//已明确事件结束，
		//中途不准被event_ctrl更改或继续事件
		event_ctrl.endEvent();
	}

	event_ctrl.event_mutex.unlock();
	return ret;
}

//temp new 文件未录制结束前就已经事件结束
void cheak_recordEnd_beforeNewRelease() {

	if (!event_begin_record_new)return;

	if (eventRecordEnd()) {
		//cout << "cheak_recordEnd_beforeNewRelease," <<
		//	fNameNew.c_str() << " " << global.fNameEventNew << endl;
		event_begin_record_new = false;
		temp_writer.release();
		rename(fNameNew.c_str(), global.fNameEventNew.c_str());
		create_temp();
		event_record_end();
	}

}
void record_event_rest(const cv::Mat& pic) {

	if (eventRecordEnd()) {
		//cout << "record_event_rest," <<
		//	fNameNew.c_str() << " " << global.fNameEventNew << endl;
		do_event_record_rest = false;
		event_rest_writer.release();
		event_record_end();
	}
	else event_rest_writer << pic;

}



void _event_record_end_(Global * global,
	long number,cv::String f_out) {

	
	string num = to_string(number);
	cv::String F[3];
	F[0] = global->temp_path + "\\" + num + "_F1.avi";
	F[1] = global->temp_path + "\\" + num + "_F2.avi";
	F[2] = global->temp_path + "\\" + num + "_F3.avi";

	int count = 0;
	global->eventFile_mutex.lock();
	bool ret = fileAttributes(global->fNameEventOld.c_str(), FILE_INVALID_FILE_ID);
	if (ret) ret = (rename(global->fNameEventOld.c_str(), F[count].c_str())==0);
	if (ret) count++;

	ret = fileAttributes(global->fNameEventNew.c_str(), FILE_INVALID_FILE_ID);
	if (ret)ret = (rename(global->fNameEventNew.c_str(), F[count].c_str())==0);
	if (ret) count++;

	ret = fileAttributes(global->fNameEventRest.c_str(), FILE_INVALID_FILE_ID);
	if (ret)ret = (rename(global->fNameEventRest.c_str(), F[count].c_str())==0);
	if (ret) count++;
	deleteFile(global->fNameEventOld);
	deleteFile(global->fNameEventNew);
	deleteFile(global->fNameEventRest);
	global->eventFile_mutex.unlock();

	//cout << "_event_record_end_"<<" " << count <<" "<<f_out<< endl;
	switch (count)
	{
	case 0:return;
		//	rename(fNameEventOld.c_str(), F[0].c_str());
		//	rename(fNameEventNew.c_str(), F[0].c_str());
		//	rename(fNameEventRest.c_str(), F[0].c_str());
	case 1:
		if (rename(F[0].c_str(), f_out.c_str())==0) {
			cout << "New Event file: " << f_out << endl << prompt;
		}
		//else {
		//	cout << "New Event file failed!!" << endl;
		//	exitProc(-1);
		//}
		break;
	default:
		cv::VideoWriter writer;
		cv::VideoCapture cap;
		camera.param_mutex.lock_shared();
		bool ret=writer.open(f_out, CV_FOURCC('M', 'J', 'P', 'G'),
			camera.fps, camera.size);
		camera.param_mutex.unlock_shared();
		if (!ret) {
			cout << "无法创建事件文件: " << f_out << endl;
			exitProc(-1);
		}
		cv::Mat pic;
		for (int i = 0; i < count; i++) {
			if (!cap.open(F[i])) {
				cout << "文件打开失败: " << F[i] << endl;
				continue;
			}
			while (true) {
				cap >> pic;
				if (pic.empty())break;
				writer << pic;
			}
			cap.release();
			deleteFile(F[i]);
		}
		writer.release();
		cout << "New Event file: " << f_out << endl << prompt;
	}
}



void event_record_end() {

	static long number = 0;
	number++;

	cv::String prefix="";
	if (event_is_multi)prefix = "multi ";
	if (event_is_timeout)prefix = prefix+"opening ";
	switch (event_status) {
	case EventInvade:
		prefix = prefix+"invade ";
		break;
	case EventGoHome: 
		prefix = prefix + "gohome ";
		break;
	case EventGoHomeContinue: 
		prefix = prefix + "gohome_continue ";
		break;
	case EventLeaveHome: 
		prefix = prefix + "leavehome ";
		break;
	default:
		prefix = "!!!wrong ";
		break;
	}


	cv::String f_out = ((event_status == EventInvade) ?
		global.exception_path : global.event_path) + "\\"+
		prefix + to_string(time_event_begin) + ".avi";

	thread(_event_record_end_,&global,number,f_out).detach();
}
