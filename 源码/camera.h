#pragma once

#include <opencv2/opencv.hpp>
#include <shared_mutex>
#include <string>
#include <ostream>
//static const std::string cmd_set_camera_prefix = "camera";

#define DEFAULT_CAMERA_ID			0

/*
	out << "ID           (id)          设备号      [W]"	<< std::endl;
	out << "FPS          (fps)         帧率        "	<< std::endl;
	out << "Resolution   (r/size)      分辨率      [W]"	<< std::endl;
	out << "Brightness   (b)           亮度        [W]"	<< std::endl;
	out << "Contrast     (c)           对比度      [W]"	<< std::endl;
	out << "Saturation   (s)           饱和度      [W]"	<< std::endl;
	out << "Hue          (h)           色调        [W]"	<< std::endl;
	out << "Exposure     (e)           曝光        [W]"	<< std::endl;
*/



static 	std::shared_mutex			cam_current_mutex;

struct Camera
{
	cv::Mat						current;
	long						frame_counter = 0;



	std::shared_mutex			param_mutex;

	int								id;
	cv::Size						size;		//分辨率
	int								fps;		//帧率
	int								brightness;	//亮度
	int								contrast;	//对比度
	int								saturation;	//饱和度
	int								hue; 		//色调
	int								exposure;	//曝光


	void push_cmd_set(std::string cmd);
	void push_cmd_set(std::string param,double value,bool is_reload=true);
	bool init();
	bool open(int cam_id);

	void show_params();
	void show_param(std::string param);
	void cmd_param_help();
	void waitFirstFrame(int _pd = 200);

	void cmd_set_help();

	void getAllParam();
	void setAllParam();

	//private
	bool ext_param_map(std::string& param, int& propid, int* & this_param);
	void command_set(bool & is_reload);
	std::vector<std::string>		cmd_set_list;
	std::mutex						cmd_set_list_mutex;

	cv::VideoCapture			cap;	//其他线程不得使用

	std::ostream& out			= std::cout;
};

static Camera camera;
