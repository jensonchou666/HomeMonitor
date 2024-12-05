#include "camera.h"
#include "global.h"

void Camera::push_cmd_set(std::string cmd) {
	cmd_set_list_mutex.lock();
	cmd_set_list.push_back(cmd);
	cmd_set_list_mutex.unlock();
}

void Camera::push_cmd_set(std::string param, double value, bool is_reload) {
	push_cmd_set(param + " "+std::to_string(value) + " " + (is_reload?"reload":"")  );
}
bool Camera::open(int cam_id) {
	if (!cap.open(cam_id)) {
		std::cout << "打开camera失败" << std::endl;
		return false;
	}


	id = cam_id;
	getAllParam();

	return true;
}


bool Camera::init() {
	return open(DEFAULT_CAMERA_ID);
}
bool Camera::ext_param_map(std::string & param,int & propid,int * & this_param) {

	if (param == "b") {
		propid = cv::CAP_PROP_BRIGHTNESS; this_param = &brightness;
	}
	else if (param == "c") {
		propid = cv::CAP_PROP_CONTRAST; this_param = &contrast;
	}
	else if (param == "s") {
		propid = cv::CAP_PROP_SATURATION; this_param = &saturation;
	}
	else if (param == "h") {
		propid = cv::CAP_PROP_HUE; this_param = &hue;
	}
	else if (param == "e") {
		propid = cv::CAP_PROP_EXPOSURE; this_param = &exposure;
	}
	else return false;
	return true;
}
using namespace std;
void Camera::command_set(bool& is_reload) {
	is_reload = false;
	if (cmd_set_list.empty())return;

	cmd_set_list_mutex.lock();
	is_reload = false;
	for (int i = 0; i < cmd_set_list.size(); i++) {
		istringstream issm(cmd_set_list[i]);
		string param; bool reOpen = false;
		int value = 0;
		if (issm)issm >> param;
		if (issm)issm >> value;

		if (param == "help" || param == "-h")
			camera.cmd_set_help();
		//else if (value == -1) 
		//	cout << "错误指令" << endl;		
		else if (param == "id") {
			int pid = id;
			cap.release();
			bool ret = open(value);
			if (ret) {
				cv::Mat pic; cap >> pic;
				ret = !pic.empty();
			}
			if (!ret) {
				cout << "摄像头(" << value << ")打开失败"<<endl;
				reOpen = true;
			}
			else {
				show_params();
				is_reload = true;
			}
		}
		else if (param == "r" || param == "size") {
			int value2 = -1;
			issm >> value2;
			if (value2 == -1)
				cout << "错误指令" << endl;
			else {
				cap.set(cv::CAP_PROP_FRAME_WIDTH, value);
				cap.set(cv::CAP_PROP_FRAME_HEIGHT, value2);
				cv::Mat pic; cap >> pic;
				if (pic.empty()) {

					cout << "摄像头不支持该分辨率" << endl;
					reOpen = true;
				}
				else {
					param_mutex.lock();
					size = cv::Size(value, value2);
					param_mutex.unlock();
					is_reload = true;
				}
			}
		}
		else if (param == "fps") {
			cap.set(cv::CAP_PROP_FPS, value);
			cv::Mat pic; cap >> pic;
			if (pic.empty()) {
				cout << "摄像头不支持该帧率" << endl;
				reOpen = true;
			}
			else {
				param_mutex.lock();
				fps = value;
				param_mutex.unlock();
				is_reload = true;
			}
		}
		else {
			string str_reload;
			int proid; int* this_param;
			if (!ext_param_map(param, proid, this_param)) {
				cout << "没有该项参数" << endl;
				continue;
			}
			cap.set(proid, value);
			cv::Mat pic; cap >> pic;
			if (pic.empty()) {
				cout << "摄像头不支持该项参数值" << endl;
				reOpen = true;
			}
			else {
				param_mutex.lock();
				*this_param = value;
				issm >> str_reload;
				param_mutex.unlock();
				is_reload = ( str_reload == "reload"
					|| str_reload == "-r");
			}
		}

		if (reOpen) {
			cap.release();
			cap.open(id);
			//setAllParam();
		}
	}
	cmd_set_list.clear();
	cmd_set_list_mutex.unlock();

	if (!is_reload)cout << prompt;

}


void Camera::cmd_set_help() {
	out << "格式: set cam/camera (parameter) (value)"
		" [ reload(def) |continue ]" << std::endl;
	out << "parameter       -- 参考param cam help" << std::endl;
	out << "value           -- 设置参数的值，注意:摄像头必须支持该值"
		"，设置分辨率时依次输入宽高" << std::endl;
	out << "reload/-r       -- 重启程序, id、fps、size默认" << std::endl;
	out << "continue        -- 不重启，其他默认(id、fps、size不可选)" << std::endl;
}
void Camera::getAllParam()
{
	param_mutex.lock_shared();
	fps =  cap.get(cv::CAP_PROP_FPS);
	size = cv::Size(
		(int)cap.get(cv::CAP_PROP_FRAME_WIDTH),
		(int)cap.get(cv::CAP_PROP_FRAME_HEIGHT)
	);
	brightness = cap.get(cv::CAP_PROP_BRIGHTNESS);
	contrast = cap.get(cv::CAP_PROP_CONTRAST);
	saturation = cap.get(cv::CAP_PROP_SATURATION);
	hue = cap.get(cv::CAP_PROP_HUE);
	exposure = cap.get(cv::CAP_PROP_EXPOSURE);
	param_mutex.unlock_shared();
}
void Camera::setAllParam()
{
	param_mutex.lock();
	cap.set(cv::CAP_PROP_FPS,fps);
	cap.set(cv::CAP_PROP_FRAME_WIDTH, size.width);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
	cap.set(cv::CAP_PROP_BRIGHTNESS, brightness);
	cap.set(cv::CAP_PROP_CONTRAST, contrast);
	cap.set(cv::CAP_PROP_SATURATION, saturation);
	cap.set(cv::CAP_PROP_HUE, hue);
	cap.set(cv::CAP_PROP_EXPOSURE, exposure);
	param_mutex.unlock();
}



void Camera::cmd_param_help() {
	out << "ID           (id)          设备号      [W]"	<< std::endl;
	out << "FPS          (fps)         帧率        [W]"	<< std::endl;
	out << "Resolution   (r/size)      分辨率      [W]"	<< std::endl;
	out << "Brightness   (b)           亮度        [W]"	<< std::endl;
	out << "Contrast     (c)           对比度      [W]"	<< std::endl;
	out << "Saturation   (s)           饱和度      [W]"	<< std::endl;
	out << "Hue          (h)           色调        [W]"	<< std::endl;
	out << "Exposure     (e)           曝光        [W]"	<< std::endl;
}

void Camera ::show_param(std::string param) {
	if (param.empty() || param=="all") {
		show_params();
		return;
	}
	param_mutex.lock_shared();
	if (param == "id")
		out << "ID          (id)          设备号" << id << std::endl;
	else if (param == "fps")
		out << "FPS         (fps)         帧率  " << fps << std::endl;
	else if (param == "r" || param == "size")
		out << "Resolution  (r/size)      分辨率" << size.width << "×"
		<< size.height << std::endl;
	else if (param == "b")
		out << "Brightness  (b)           亮度  " << brightness << std::endl;
	else if (param == "c")
		out << "Contrast    (c)           对比度" << contrast << std::endl;
	else if (param == "s")
		out << "Saturation  (s)           饱和度" << saturation << std::endl;
	else if (param == "h")
		out << "Hue         (h)           色调  " << hue << std::endl;
	else if (param == "e")
		out << "Exposure    (e)           曝光  " << exposure << std::endl;
	param_mutex.unlock_shared();

	if (param == "help" || param == "-h")cmd_param_help();
}
void Camera::show_params() {

	out << "Camera's params :" << std::endl;
	param_mutex.lock_shared();
	//out << std::setiosflags(std::ios::right);
	out.setf(ios::right);
	out << "ID          (id)          设备号" << setw(20) << id << std::endl;
	out << "FPS         (fps)         帧率  " << setw(20) << fps << std::endl;
	out << "Resolution  (r/size)      分辨率" << setw(20) <<
		to_string(size.width)+"×"+to_string(size.height)<< std::endl;
	out << "Brightness  (b)           亮度  " << setw(20) << brightness  << std::endl;
	out << "Contrast    (c)           对比度" << setw(20) << contrast  << std::endl;
	out << "Saturation  (s)           饱和度" << setw(20) << saturation  << std::endl;
	out << "Hue         (h)           色调  " << setw(20) << hue  << std::endl;
	out << "Exposure    (e)           曝光  " << setw(20) << exposure  << std::endl;
	param_mutex.unlock_shared();
}

void Camera::waitFirstFrame(int _pd) {
	while (true) {
		cam_current_mutex.lock_shared();
		if (frame_counter > 0)break;
		cam_current_mutex.unlock_shared();
		std::this_thread::sleep_for
		(std::chrono::milliseconds(_pd));
	}
}


/*
Point2i pointWinToImg(const Point2i& p,const Camera & camera) {
	camera.param_mutex.lock_shared();
	double ratew = (double)camera.size.width /
		(double)cv::getWindowImageRect(main_win_name).width;
	double rateh = (double)camera.size.height /
		(double)cv::getWindowImageRect(main_win_name).height;
	//cout << "rate: " << ratew <<" "<< rateh << endl;
	camera.param_mutex.unlock_shared();
	return Point2i(
		(int)(ratew * (double)p.x),
		(int)(rateh * (double)p.y)
	);
}


Point2i pointImgToWin(const Point2i& p, const Camera& camera) {
	camera.param_mutex.lock_shared();
	double ratew = (double)cv::getWindowImageRect(main_win_name).width
		/ (double)camera.size.width;
	double rateh = (double)cv::getWindowImageRect(main_win_name).height
		/ (double)camera.size.height;
	camera.param_mutex.unlock_shared();
	return Point2i(
		(int)(ratew * (double)p.x),
		(int)(rateh * (double)p.y)
	);
}
Rect rectImgToWin(const Rect& r) {
	return Rect(pointImgToWin(r.br()),pointImgToWin(r.tl()));
}
Rect rectWinToImg(const Rect& r) {
	return Rect(pointWinToImg(r.br()), pointWinToImg(r.tl()));
}
*/