#pragma once


#include "global.h"
#include <queue>
#include <vector>
#include <list>
#include "camera.h"
#include "event_ctrl.h"


using namespace std;


struct LineDetectItem;
struct RectDetectItem;
struct RectDetectItemParameters;

enum Direction {
	Left,Right, DirectionUnknown
};


class DoorMonitor
{
public:

	void init(ProcCtrl* pc, Camera* camera,
		EventCtrl* event_ctrl, RectDetectItemParameters*parameters) {
		this->pc = pc;
		this->camera = camera;
		this->event_ctrl = event_ctrl;
		rect_detect_parameters = parameters;
	}

	ProcCtrl* pc;
	Camera* camera;
	EventCtrl *event_ctrl;
	RectDetectItemParameters* rect_detect_parameters;
	void run();
	void when_showPic(cv::Mat& pic);
	void do_cmd(istringstream& issm);
	void cmd_set(istringstream & issm);
	void show_param(istringstream& issm);
	void load_params();
	void save_params();
	void set_range();

private:
	void hough(cv::Mat& image, std::vector<cv::Vec4i>& lines,
		double minLineLength = -1, cv::Mat* image_canny = NULL);

	inline bool detect_rect(cv::Mat& image, cv::Rect& out,
		std::vector<cv::Vec4i>& lines,bool is_org,cv::Mat* canny = NULL);
	bool _detect_rect_(cv::Mat& image, cv::Rect& out,
		std::vector<cv::Vec4i>& lines, bool is_org, cv::Mat* canny);
	void detect_mark_org();
	bool detect_mark();
	void door_monitoring();

	/*---------------------------  Parameters  ----------------------------*/

	boolean do_blur = true;
	double		gaussianblur_sigma = 0.8+0.2;			//标准差，越大越模糊
	cv::Size	gaussianblur_ksize = cv::Size(5,5);	//高斯滤波算子大小
	double		canny_threashhold_min =40;				//最低阈值
	double		canny_threashhold_max =100;			    //最高阈值
	int			canny_aperturesize	= 3;				//canny算子大小
	int			hough_threashhold	= 10;	
	double		hough_maxLineGap	=4;
	double		hough_minLineLength = 6;
	std::mutex hough_mutex; 



	int					h_unit_angle_deviation = 10;
	atomic_int			h_unit_y_gap = 6;
	atomic_int			h_unit_x_deviation = 4;
	atomic_int			w_unit_deviation = 4;
	atomic_int			w_unit_angle_deviation = 10;

	int			h_max_width =  3;
	//double		h_minLineLen_rate = 0.1;
	double		h_minLineLen_org_rate = 0.2;// 依据标记活动范围height修改
	std::mutex  h_length_mutex;

	atomic_int		w_angle_deviation =  35;// 矩形宽角度波动


	//一个周期内识别率低于20,认为目标已消失,高于20则检测成功		
	atomic_int mark_disapeared_maxtimes		= 20;
	atomic_int mark_disapeared_times		= 0;
	atomic_int detectMark_timeout = 1000;

	double threashold_door_closed = 0.08;//>max_w_deviation_rate
	double threashold_door_opened = 0.3;//>max_w_deviation_rate
	std::mutex threashold_door_mutex;



	double max_w_deviation_rate = 0.4;
	double max_h_deviation_rate = 0.4;
	double max_h_deviation_rate_cmp = 0.5;
	bool  is_detect3lines = true;
	std::mutex max_deviation_rate_mutex;

	//atomic_bool is_mark_bigger = false;

	//atomic_uint mark_center_color_deviation =200;
	//atomic_uint max_color_deviation = 20;	//颜色偏差，0 -- 441
	//atomic_uint max_color_deviation_deltaX = 30;//由于标记的移动

	atomic_int16_t min_workframe = 1;
	//short detectRect_threashold=2;	//0--4

	//int door_open_direciton = DirectionUnknown;


public:
	cv::Rect mark_range;	//标记的活动范围
	
	/*+++++++++++++++++++++++++++++++++++++++++++++*/
	atomic_bool  is_show_mark_range = true;
	atomic_bool  is_show_mark = true;
	atomic_bool  is_show_lines = false;
	atomic_bool  is_show_canny = false;
	atomic_bool  is_show_org = false;
	atomic_bool  is_save = true;
	atomic_bool  is_debug = false;
	atomic_bool  is_auto = false;
	atomic_bool  is_test = false;
	

	atomic_bool  flag_setrange = false;
	atomic_bool  flag_finished_setrange = false;
	cv::Point2i pFirst;
	cv::Rect range_temp;
	void on_mouse_selrange(int event, int x, int y);

	cv::Vec3b rectCenterColor(cv::Mat& img, cv::Rect& rect);
	//uint envBrightness();
	uint distance(const cv::Vec3b v1,const  cv::Vec3b v2);

private:


	bool _detect_rect_over_(cv::Rect& rect, bool is_org,
		bool& do_return);

	cv::Rect	mark_org = cv::Rect();
	//cv::Vec3b   mark_center_color;
	//uint		env_brightness;
	//vector<std::string> command_list;
	//std::mutex command_list_mutex;
	void check_proc_stoped();
	bool range_changed = false;

	long detect_mark_times = 0;
	long detect_mark_succeed_times = 0;
	long detect_mark_failed_before_succeed_times = 0;
	int detect_mark_failed_before_succeed_maxtimes = 0;
	double detect_mark_avg_failed_before_succeed_times = 0;

	mutex detect_mark_times_mutex;
	LineDetectItem *current_item=NULL;
	LineDetectItem *last_item = NULL;


	list<RectDetectItem> recentDetectItems;
	list<RectDetectItem> thisDetectItems;
#define WaitTime	10


	std::mutex mark_org_mutex;
	std::mutex range_temp_mutex;
	std::mutex mark_range_mutex;
	std::mutex last_item_mutex;
};


struct LineDetectItem {

	LineDetectItem(const cv::Mat& pic, long frame) {
		img = pic.clone();
		this->frame = frame;
	}
	cv::Mat img;
	cv::Mat img_range;
	//cv::Mat img_;
	long frame;
	cv::Mat out_canny;
	vector<cv::Vec4i> lines;
	cv::Rect rect;
	//bool is_mark = false;
	//vector<cv::Vec3f> circles;
	//cv::Vec4i flag_lineV_left;
	//cv::Vec4i flag_lineV_right;
	//cv::Mat pretreated;

};

struct RectDetectItemParameters {
	double threashold_rect_series_x = 0.3;
	double threashold_rect_series_y = 0.2;
	int threashold_rect_seriesCount = 3;
	int series_frame_out = 15;
	std::shared_mutex this_mutex;
};


typedef list<RectDetectItem>::iterator pRectDetectItem;

struct RectDetectItem {


	RectDetectItemParameters* params;

	cv::Rect rect;
	int dw, dh;
	long lastFrame;
	long seriesCount=0;


	RectDetectItem(RectDetectItemParameters* _params,
		const cv::Rect& _rect, long frame) {
		params = _params;
		newRect(_rect, frame);
	}

	void newRect_locked(const cv::Rect& _rect,long frame) {
		rect = _rect;
		seriesCount ++;
		lastFrame = frame;
		params->this_mutex.lock_shared();
		dw = params->threashold_rect_series_x * rect.width;
		dh = params->threashold_rect_series_y * rect.height;
		params->this_mutex.unlock_shared();
	}
	void newRect(const cv::Rect& _rect, long frame) {
		rect = _rect;
		seriesCount++;
		lastFrame = frame;
		dw = params->threashold_rect_series_x * rect.width;
		dh = params->threashold_rect_series_y * rect.height;
	}
	bool  serial(const cv::Rect & rect_new) {
		return	abs(rect_new.x - rect.x) < dw			&&
				abs(rect_new.y - rect.y) < dh			&&
				abs(rect_new.width - rect.width) < dw	&&
				abs(rect_new.height - rect.height) < dh;
	}
	bool isMark() {
		return seriesCount >= params->threashold_rect_seriesCount;
	}

};


//#define max_w_deviation_rate_cmp			2




static RectDetectItemParameters rect_detect_parameters;
static	DoorMonitor door_monitor;


static void filter_angle(vector<cv::Vec4i>& lines, list<cv::Vec4i>& lines_out,
	double theta, double errThetarange);
static void filter_angle(vector<cv::Vec4i>& lines,
	list<pair< cv::Vec4i, double> >& lines_out_theta,
	double lt, double errThetarange);
//static bool neighber()