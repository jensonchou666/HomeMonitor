#include "face_detector.h"

using namespace cv;
using namespace std;
void FaceDetector::run()
{
	camera->waitFirstFrame();
	flush_min_size();
	while (true)
	{
		check_proc_stoped();

		
		if (event_ctrl->status != EventIn) {
			if (is_test) {
				detect();
			}
			if (event_ctrl->status != EventIn) {
				this_thread::sleep_for(chrono::milliseconds(80));
				continue;
			}
			else {
				if (judgeTimeOut())continue;
				if(judge())continue;
			}
		}
		cam_current_mutex.lock_shared();
		if (current_item != NULL && camera->frame_counter -
			current_item->frame < min_workframe) {
			cam_current_mutex.unlock_shared();
			this_thread::sleep_for(chrono::milliseconds(10));
			continue;
		}
		cam_current_mutex.unlock_shared();


		if(judgeTimeOut())continue;
		detect();
		judge();

	}

}

void FaceDetector::clearData() {
	last_item_mutex.lock();
	if (last_item != NULL)delete last_item;
	last_item = NULL;
	last_item_mutex.unlock();
	if (current_item != NULL)delete current_item;
	current_item = NULL;

	detecting = false;
}

void FaceDetector::check_proc_stoped() {

	while (pc->stoped) {

		if (pc->wait_counter != 0 && !pc->flag_body_d_finished) {

			clearData();
			flush_min_size();
			//....
			pc->wait_counter--;
			pc->flag_body_d_finished = true;
		}
		this_thread::sleep_for(chrono::milliseconds(200));
	}
}



void FaceDetector::when_showPic(cv::Mat& pic)
{

	if (is_show) {

		last_item_mutex.lock();
		if (last_item != NULL) {
			for (Rect fd : last_item->found)
				rectangle(pic, fd, Scalar(255,0,255),2);
		}
		last_item_mutex.unlock();
	}

}

void FaceDetector::detect()
{

	if (current_item != NULL) {
		last_item_mutex.lock();
		if (last_item != NULL)delete last_item;
		last_item = current_item;
		last_item_mutex.unlock();
	}
	cam_current_mutex.lock_shared();
	current_item = new FaceDetectItem(camera->current,
		camera->frame_counter);
	cam_current_mutex.unlock_shared();


	static Timer timer2(2000);

	timer2.begin();
	param_mutex.lock();
	double factor = scale_factor;
	int mn = min_neighbours;
	int flags = detect_flags;
	cv::Size mz = min_size;
	double multi = face_enlarge_multiple;
	param_mutex.unlock();
	classifier.detectMultiScale(current_item->img,
		current_item->found,
		factor, mn, flags, mz);

	for (Rect rect: current_item->found) {
		current_item->found_multi.push_back(
			Rect(
				rect.x - ((multi - 1) / 2) * rect.width,
				rect.y - ((multi - 1) / 2) * rect.height,
				rect.width * multi, rect.height * multi
			));
	}
	if (is_debug)timer2.end("face_detect,size:"+
		to_string(current_item->found.size()));
}

bool  FaceDetector::judgeTimeOut() {

	if (!detecting) {
		detecting = true;
		time_first_detect = chrono::system_clock::now();
	}

	chrono::system_clock::time_point now
		= chrono::system_clock::now();
	chrono::system_clock::duration time_fromOpened = now -
		time_first_detect;

	long long time_count = chrono::duration_cast<chrono::milliseconds>
		(time_fromOpened).count();

	if (time_count > event_ctrl->detectFace_timeout) {
		event_ctrl->status = EventInvade;
		event_ctrl->faceDetect_over = true;
		event_ctrl->invade(now);
		clearData();
		return true;
	}
	return false;
}

bool FaceDetector::judge()
{

	if (!current_item->found.empty()) {
		event_ctrl->status = EventGoHome;
		event_ctrl->faceDetect_over = true;
		event_ctrl->gohome(chrono::system_clock::now());
		clearData();
		return true;
	}
	return false;
}

void FaceDetector::flush_min_size()
{
	camera->param_mutex.lock();
	param_mutex.lock();
	int minLen = min(camera->size.width, camera->size.height)*min_size_rate;
	this->min_size = cv::Size(minLen, minLen);
	param_mutex.unlock();
	camera->param_mutex.unlock();
	
}
void FaceDetector::initClassifier() {

	if (classifier.load(haarcascadesDir + "\\" + classifierFileName))return;
	if (classifier.load(haarcascadesDir + "\\" + classifierFile2Name))return;
	if (classifier.load(haarcascadesDir + "\\" + classifierFile3Name))return;
	cout << "δ�ҵ�����ʶ��������ļ�(" <<
		classifierFileName << "/" << classifierFile2Name << "/"
		<< classifierFile3Name << ")" << endl;
	cout << "������ļ���ȷ��";
	if (!affirm())exit(-1);

	if (classifier.load(haarcascadesDir + "\\" + classifierFileName))return;
	if (classifier.load(haarcascadesDir + "\\" + classifierFile2Name))return;
	if (classifier.load(haarcascadesDir + "\\" + classifierFile3Name))return;

	cout << "δ�ҵ�����ʶ��������ļ�" << endl;
	exit(-1);
}

#include "door_monitor.h"
void  FaceDetector::facePlusCmd() {

	bool is_test_tmp = door_monitor->is_test;
	
	if (!is_test_tmp) {
		door_monitor->is_test = true;
		cout << "��ʱ�ر���DoorMonitor���ϱ��¼�����" << endl;
	}

#define CMD_BUFFER_SIZE 2000
	char cmd_buffer[CMD_BUFFER_SIZE];

	while (cin.getline(cmd_buffer, CMD_BUFFER_SIZE)) {
		
		std::string cmd(cmd_buffer);
		std::istringstream issm(cmd);
		std::string cmd_f;
		issm >> cmd_f;
		if (cmd_f == "help") {
			cout << "detect              -- ����ʶ��" << endl;

			cout << "quit/q              -- �˳�" << endl;
		}
		else if (cmd_f == "detect") {
			cv::String WindowNameDetect = "face detect";
			namedWindow(WindowNameDetect);
			while (1) {
				while (1) {
					detect();
					if (current_item == NULL || current_item->found.empty())
						continue;
				}
				last_item_mutex.lock();
				if (last_item != NULL)delete last_item;
				last_item = NULL;
				last_item_mutex.unlock();

				Mat img1 = current_item->img.clone();
				for (Rect fd : current_item->found_multi) {
					rectangle(img1, fd, Scalar(255, 0, 255), 2);					
				}
				imshow(WindowNameDetect,img1);

				char buffer[CMD_BUFFER_SIZE];
				while (cin.getline(buffer, CMD_BUFFER_SIZE)) {

					std::string cmd(cmd_buffer);
					std::istringstream issm(cmd);
					std::string cmd_f;

				}

				for (Rect fd : current_item->found_multi) {
					
				}
				namedWindow(WindowNameDetect);

				
				
			}
		}
		else if (cmd_f == "quit" || cmd_f == "q") {
			if (!is_test_tmp) {
				door_monitor->is_test = false;
			}
			return;
		}

		cout << "[Face ++]";
		memset(cmd_buffer, 0, CMD_BUFFER_SIZE);
	}
	cout << "cin.getline����" << endl;
	if (!is_test_tmp) {
		door_monitor->is_test = false;
	}
}

#include "curl/curl.h"

void FaceDetector::faceCompare(cv::Mat& img)
{
	vector<uchar> vec_Img;
	vector<int> vecCompression_params;
	vecCompression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	vecCompression_params.push_back(90);
	imencode(".jpg", img, vec_Img, vecCompression_params);
	/*
	CURL* curl;
	CURLcode res;

	string buffer;

	struct curl_slist* http_header = NULL;

	curl = curl_easy_init();
		*/
}



void FaceDetector::do_cmd(istringstream& issm)
{
	string arg1, arg2;
	if (issm)issm >> arg1;
	if (arg1 == "show") {
		if (issm)issm >> arg2;
		if (arg2.empty())
			is_show = true;
	}
	else if (arg1 == "unshow") {
		if (issm)issm >> arg2;
		if (arg2.empty())
			is_show = false;
	}
	else if (arg1 == "debug") {
		is_debug = !is_debug;
		if (is_debug)cout << "�Ѵ򿪵��Թ���" << endl;
	}
	else if (arg1 == "save") {
		is_save = true;
	}
	else if (arg1 == "unsave") {
		is_save = false;
	}
	else if (arg1 == "test") {
		is_test = true;
	}
	else if (arg1 == "untest") {
		is_test = false;
		clearData();
	}
	else if (arg1 == "configures") {
#define T_or_F(is_show)	(is_show?"true":"false")
		cout << "door monitor configures:" << endl;

		cout << "debug         " << T_or_F(is_debug) << endl;
		cout << "test          " << T_or_F(is_test) << endl;
		cout << "show         " << T_or_F(is_show) << endl;
		cout << "save          " << T_or_F(is_save) << endl;

	}
	else if (arg1 == "param") {
		show_param(issm);
	}
	else if (arg1 == "set") {
		cmd_set(issm);
	}
	else if (arg1 == "face++") {
		facePlusCmd();
	}
	else if (arg1 == "help" || arg1 == "-h") {
		cout << "face++           ����Face++������𽻻����򣬿ɽ�����Ȩ" << endl;
		cout << "debug            ��/ȡ������" << endl;
		cout << "test             ����,����ʱ��������ʶ�𣬲������¼�" << endl;
		cout << "untest           �رղ���" << endl;

		cout << "show             ��ʾʶ�𵽵�Ŀ��" << endl;
		cout << "unshow           ����ʾʶ��Ŀ��" << endl;
		cout << "save             ����ʾ���ݱ�����ͼ����" << endl;
		cout << "unsave           ������ʾ���ݱ�����ͼ����" << endl;
		cout << "configures       ��ʾ�������ݵ�����ֵ" << endl;
		cout << "param            ��ʾ����" << endl;
		cout << "set              ���ò���" << endl;
		
	}
	else cout << "����ָ��" << endl;

}

void FaceDetector::cmd_set(istringstream& issm) {
	string param;
	if (issm)issm >> param;
	param_mutex.lock();
	if (param == "1" || param == "scale_factor") {
		double value = 0; issm >> value;
		if (value != 0)scale_factor = value;
	}
	else if (param == "2" || param == "min_neighbours") {
		int value = 0; issm >> value;
		if (value != 0)min_neighbours = value;
	}
	else if (param == "3" || param == "min_size_rate") {
		double value = 0; issm >> value;
		if (value != 0)min_size_rate = value;
	}
	param_mutex.unlock();
}
void FaceDetector::show_param(istringstream& issm){
	string param;
	if (issm)issm >> param;
	if (param.empty() || param == "all") {
		cout << "Face detector's params :" << std::endl;
		cout.setf(ios::right);

		param_mutex.lock();
		cout << "<1>�������ڵı���ϵ��                          [ > 1,Ĭ��1.1 ] (scale_factor)    "
			<< setw(10) << scale_factor << endl;
		cout << "<2>���ɼ��Ŀ������ھ��ε���С����            [ Ĭ��3 ]       (min_neighbours)  "
			<< setw(10) << min_neighbours << endl;
		cout << "<3>��Сʶ�������С����������ͼ���߽�Сֵ��  [ Ĭ��0.12 ]    (min_size_rate)   "
			<< setw(10) << min_size_rate << endl;
		param_mutex.unlock();

	}
	else if (param == "help" || param == "-h") {

	}
	else {
		cout << "�ݲ�֧�ָ�������鿴��ʽ" << endl;
	}
}