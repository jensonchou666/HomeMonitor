#pragma once

#include "global.h"
#include "camera.h"
#include "event_ctrl.h"
#include "door_monitor.h"
struct FaceDetectItem {
	FaceDetectItem(cv::Mat image,long _frame) {
		img = image.clone();
		frame = _frame;
	}
	cv::Mat img;
	long frame;
	vector<cv::Rect> found;
	vector<cv::Rect> found_multi;
};




class FaceDetector {

public:
	
	cv::String haarcascadesDir = "haar";
	const  cv::String classifierFileName = "haarcascade_frontalface.xml";
	const  cv::String classifierFile2Name = "haarcascade_frontalface_alt.xml";
	const  cv::String classifierFile3Name = "haarcascade_frontalface_alt2.xml";

	cv::CascadeClassifier classifier;
	Camera* camera;
	ProcCtrl* pc;
	EventCtrl * event_ctrl;
	DoorMonitor* door_monitor;
	void init(ProcCtrl* proc_ctrl, Camera* camera,
		EventCtrl *  ec, DoorMonitor* _door_monitor) {
		  
		pc = proc_ctrl;
		this->camera = camera;
		event_ctrl = ec;
		door_monitor = _door_monitor;
		initClassifier();

	}

	void run();
	void check_proc_stoped();
	void clearData();
	void flush_min_size();
	void initClassifier();

	void do_cmd(std::istringstream& issm);
	void cmd_set(istringstream& issm);
	void show_param(istringstream& issm);
	void when_showPic(cv::Mat& pic);

	bool judgeTimeOut();
	void detect();
	bool judge();
	void facePlusCmd();

	void faceCompare(cv::Mat & pic);

	double scale_factor = 1.1;
	int min_neighbours=3;
	int detect_flags= CV_HAAR_DO_CANNY_PRUNING;
	cv::Size min_size;
	double min_size_rate = 0.12;
	double face_enlarge_multiple = 1.4;
	mutex param_mutex;



	std::atomic_bool	is_show		= true;
	std::atomic_bool	is_debug	= false;
	std::atomic_bool	is_save		= true;
	std::atomic_bool	is_test		= false;


	//为防止反复开关门，超时判定的起始时间设置为首次人脸识别的时间
//而非event_ctrl->time_lastEvent_begin或event_ctrl->time_event_begin
	bool detecting=false;
	chrono::system_clock::time_point time_first_detect;

	FaceDetectItem* current_item;
	FaceDetectItem* last_item;

	int min_workframe = 1;

	std::mutex last_item_mutex;
};


static FaceDetector face_detector;