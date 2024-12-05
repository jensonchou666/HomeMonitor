#pragma once

#include <opencv2/opencv.hpp>
#include <shared_mutex>
#include <string>
#include <ostream>
//static const std::string cmd_set_camera_prefix = "camera";

#define DEFAULT_CAMERA_ID			0

/*
	out << "ID           (id)          �豸��      [W]"	<< std::endl;
	out << "FPS          (fps)         ֡��        "	<< std::endl;
	out << "Resolution   (r/size)      �ֱ���      [W]"	<< std::endl;
	out << "Brightness   (b)           ����        [W]"	<< std::endl;
	out << "Contrast     (c)           �Աȶ�      [W]"	<< std::endl;
	out << "Saturation   (s)           ���Ͷ�      [W]"	<< std::endl;
	out << "Hue          (h)           ɫ��        [W]"	<< std::endl;
	out << "Exposure     (e)           �ع�        [W]"	<< std::endl;
*/



static 	std::shared_mutex			cam_current_mutex;

struct Camera
{
	cv::Mat						current;
	long						frame_counter = 0;



	std::shared_mutex			param_mutex;

	int								id;
	cv::Size						size;		//�ֱ���
	int								fps;		//֡��
	int								brightness;	//����
	int								contrast;	//�Աȶ�
	int								saturation;	//���Ͷ�
	int								hue; 		//ɫ��
	int								exposure;	//�ع�


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

	cv::VideoCapture			cap;	//�����̲߳���ʹ��

	std::ostream& out			= std::cout;
};

static Camera camera;
