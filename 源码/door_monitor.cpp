#include "door_monitor.h"
#include <math.h>



using namespace cv;
void DoorMonitor::run()
{
	camera->waitFirstFrame(200);
	
	while (true)
	{
		check_proc_stoped();

		mark_range_mutex.lock();
		mark_org_mutex.lock();
		if (mark_range.empty() || mark_org.empty()) {
			mark_org_mutex.unlock();
			mark_range_mutex.unlock();
			this_thread::sleep_for(chrono::milliseconds(500));			
			continue;
		}
		mark_org_mutex.unlock();
		mark_range_mutex.unlock();

		cam_current_mutex.lock_shared();
		if (current_item!=NULL && camera->frame_counter -
			current_item->frame < min_workframe) {		
			cam_current_mutex.unlock_shared();
			this_thread::sleep_for(chrono::milliseconds(WaitTime));
			continue;
		}
		cam_current_mutex.unlock_shared();

		if (current_item != NULL) {
			last_item_mutex.lock();
			if (last_item != NULL)delete last_item;
			last_item = current_item;
			last_item_mutex.unlock();
		}
		cam_current_mutex.lock_shared();
		current_item = new LineDetectItem(camera->current,
			camera->frame_counter);
		cam_current_mutex.unlock_shared();
		door_monitoring();
	}

}



void DoorMonitor::check_proc_stoped() {

	while (pc->stoped) {

		if (pc->wait_counter != 0 && !pc->flag_door_m_finished) {
			
			//while (!history.empty())history.pop();
			last_item_mutex.lock();
			if (last_item != NULL)delete last_item;
			last_item = NULL;
			last_item_mutex.unlock();
			if (current_item != NULL)delete current_item;
			current_item = NULL;
			detect_mark_times_mutex.lock();
			detect_mark_times = 0;
			detect_mark_succeed_times = 0;
			detect_mark_failed_before_succeed_times = 0;
			detect_mark_failed_before_succeed_maxtimes = 0;
			detect_mark_avg_failed_before_succeed_times = 0;
			detect_mark_times_mutex.unlock();
			//history = std::queue<LineDetectItem>();
			cout << "door monitor module stoped" << endl;
			pc->wait_counter--;
			pc->flag_door_m_finished = true;
		}
		this_thread::sleep_for(chrono::milliseconds(200));
	}
}

//cmd线程
void DoorMonitor::set_range()
{
	flag_finished_setrange = false;

	flag_setrange = true;
	cout << "请在窗口内框选待监测范围（标记活动范围）" << endl;
	while (!flag_finished_setrange)
		this_thread::sleep_for(chrono::milliseconds(200));

	cout << "请确认";
	bool ret = affirm_defN();

	flag_setrange = false;

	if (ret) {
		range_temp_mutex.lock();
		mark_range_mutex.lock();
		mark_range = range_temp;
		range_temp = Rect();
		range_changed = true;
		mark_range_mutex.unlock();
		range_temp_mutex.unlock();
		cout << "已设置好监测范围,正在识别标记" << endl;
		mark_org_mutex.lock();
		mark_org = Rect();
		mark_org_mutex.unlock();
		detect_mark_org();
		if (!pc->stoped)pc->do_reload = true;
	}
	else {
		range_temp_mutex.lock();
		range_temp = Rect();
		range_temp_mutex.unlock();
	}

}


void DoorMonitor::hough(Mat & img,vector<Vec4i> &lines,
	 double minLineLength, Mat* image_canny) {
	hough_mutex.lock();
	Mat img_blur,img_canny,img_bgr2;
	Mat* img_2= &img;

	if (do_blur) {
		GaussianBlur(img, img_blur,
			gaussianblur_ksize,gaussianblur_sigma,gaussianblur_sigma);
		img_2 = &img_blur;
	}
	cvtColor(*img_2, img_bgr2, CV_BGR2GRAY);
	if (image_canny == NULL)image_canny = &img_canny;

	Canny(img_bgr2,*image_canny,
		canny_threashhold_min, canny_threashhold_max, canny_aperturesize);
	if (minLineLength == -1)minLineLength = hough_minLineLength;
	HoughLinesP(*image_canny,lines, 1,
		CV_PI / 180,hough_threashhold,minLineLength,hough_maxLineGap);
	hough_mutex.unlock();
}





/*
struct FlagDetectItem {
	Rect rect;
	int matchedLines=  0b0000;
	int h_devia, w_devia;
	FlagDetectItem(Rect rt,double max_h_deviation_rate,
		double  max_w_deviation_rate) {
		rect = rt;
		h_devia =  max_h_deviation_rate * rect.height;
		w_devia =  max_w_deviation_rate* rect.width;
	}
};*/
struct LineHData {
	//Vec4i line;
	Point left, right;
	int  width, w_devia;
	//bool is_left_incline;
	bool sellected = false;
	LineHData(const Vec4i& lineH, double max_w_deviation_rate) {
		//line = lineH;
		//y = (line[1] + line[3]) / 2;
		if (lineH[0] < lineH[2]) {
			left = Point(lineH[0], lineH[1]);
			right = Point(lineH[2], lineH[3]);
		}
		else {
			right = Point(lineH[0], lineH[1]);
			left = Point(lineH[2], lineH[3]);
		}
		width = right.x - left.x;
		w_devia = max_w_deviation_rate * width;
		//is_left_incline = (left.y >= right.y);
		//if (lineH[1] != lineH[3])
		//	is_left_incline = ((lineH[0] - lineH[2]) ^ (lineH[1] - lineH[3]) < 0);
		
	}
	LineHData(const Vec4i& lineH) {
		if (lineH[0] < lineH[2]) {
			left = Point(lineH[0], lineH[1]);
			right = Point(lineH[2], lineH[3]);
		}
		else {
			right = Point(lineH[0], lineH[1]);
			left = Point(lineH[2], lineH[3]);
		}
		width = right.x - left.x;
	}
	LineHData() {

	}
};
struct LineVData {
	int x, y_up, y_down, height, h_devia;
	bool sellected = false;
	LineVData(const Vec4i& lineV, double max_h_deviation_rate) {
		x = (lineV[0] + lineV[2]) / 2;
		y_up = min(lineV[1], lineV[3]);
		y_down = max(lineV[1], lineV[3]);
		height = y_down- y_up;
		h_devia = height * max_h_deviation_rate;
	}
	LineVData(const Vec4i& lineV) {
		x = (lineV[0] + lineV[2]) / 2;
		y_up = min(lineV[1], lineV[3]);
		y_down = max(lineV[1], lineV[3]);
		height = y_down - y_up;
	}
	LineVData() {

	}
};

/*
struct LineVUnitData {
	Vec4i *line;
	LineVUnitData(Vec4i* _line) {
		line = _line;
	}

};*/

typedef list< LineVData>::iterator pLineVTmpData;
typedef list< LineHData>::iterator pLineHTmpData;
int nodeY(const pLineHTmpData lineH, const pLineVTmpData lineV) {
	double lineH_k = 0, lineH_b = 0;
	if (!lineKBFunction(lineH->left, lineH->right, lineH_k, lineH_b))
		return 0;
	return lineH_k * lineV->x + lineH_b;
}
int nodeY(const pLineHTmpData lineH , int x) {
	double lineH_k = 0, lineH_b = 0;
	if (!lineKBFunction(lineH->left, lineH->right, lineH_k, lineH_b))
		return 0;
	return lineH_k * x + lineH_b;
}

bool DoorMonitor::_detect_rect_(Mat& image, Rect& out,
	vector<Vec4i>& lines, bool is_org, Mat* canny) {

	list<LineHData>  lineH_list;
	list<LineVData>  lineV_list;
	if (canny == NULL) {
		hough(image, lines);
	}
	else {
		hough(image, lines, -1, canny);
	}

	int min_h_len;
	h_length_mutex.lock();
	if (is_org) {
		mark_range_mutex.lock();
		min_h_len =  h_minLineLen_org_rate* mark_range.height;
		mark_range_mutex.unlock();
	}
	else {
		mark_org_mutex.lock();
		min_h_len =  0.06* mark_org.height;
		mark_org_mutex.unlock();
	}
	int max_h_width = h_max_width;
	h_length_mutex.unlock();

#define Allow_Overran	1

	/*  Unit Vertical Lines */

	list<Vec4i>  lines_v;
	filter_angle(lines, lines_v, 90, h_unit_angle_deviation);
	int h_unit_x_deviation = this->h_unit_x_deviation;
	int h_unit_y_gap = this->h_unit_y_gap;
	for (list<Vec4i>::iterator it_line = lines_v.begin();
		it_line != lines_v.end(); it_line++) {

		Vec4i line = (*it_line);
		if (abs(line[0] - line[2]) <= max_h_width &&
			abs(line[1] - line[3]) >= min_h_len) {
			LineVData data1= LineVData(line,
				max_h_deviation_rate);
			LineVData v_up;
			LineVData v_down;
			bool is_up_ex = false, is_down_ex = false;
			for (list<Vec4i>::iterator it_line2 = lines_v.begin();
				it_line2 != lines_v.end(); it_line2++) {
				if (it_line == it_line2)continue;
				LineVData data2=LineVData(*it_line2);
				if (abs(data1.x - data2.x) <= h_unit_x_deviation) {
					if ((data1.y_up + Allow_Overran) >= data2.y_down &&
						(data1.y_up - data2.y_down) <= h_unit_y_gap) {
						is_up_ex = true;
						v_up = data2;
					}
					else if (data1.y_down <= (data2.y_up + Allow_Overran) &&
						(data2.y_up - data1.y_down) <= h_unit_y_gap) {
						is_down_ex = true;
						v_down = data2;
					}
				}
			}
			if (!is_up_ex && !is_down_ex) {
				lineV_list.push_back(data1);
			}
			else {
				if (is_up_ex && is_down_ex) {
					data1.x = (data1.x + v_up.x + v_down.x) / 3;
					data1.y_up= v_up.y_up;
					data1.y_down = v_down.y_down;
				}
				else if (is_up_ex) {
					data1.x = (data1.x + v_up.x) /2;
					data1.y_up = v_up.y_up;
				}
				else if (is_down_ex) {
					data1.x = (data1.x + v_down.x) / 2;
					data1.y_down = v_down.y_down;
				}
				lineV_list.push_back(data1);
			}
			
		}
	}

	/*  Unit  Horizon Lines */
	list<pair<Vec4i,double>>  lines_h;
	filter_angle(lines, lines_h, 0, w_angle_deviation);
	int w_unit_deviation = this->w_unit_deviation;
	int w_unit_angle_deviation = this->w_unit_angle_deviation;

	typedef list<pair<Vec4i, double>>::iterator LineH_list_iterator;

	for (LineH_list_iterator it_line = lines_h.begin();
		it_line != lines_h.end(); it_line++) {
		Vec4i line = it_line->first;
		double angle= it_line->second;

		LineHData  data1 =	LineHData(line, max_w_deviation_rate);

		LineHData h_left;
		LineHData h_right;
		bool is_left_ex = false, is_right_ex = false;
		for (LineH_list_iterator it_line2 = lines_h.begin();
			it_line2 != lines_h.end(); it_line2++) {
			if (it_line == it_line2)continue;
			if (abs(angle - it_line2->second) > w_unit_angle_deviation)continue;
			LineHData data2 = LineHData(it_line2->first);
			if (abs(data1.left.x - data2.right.x) < w_unit_deviation &&
				abs(data1.left.y - data2.right.y) < w_unit_deviation) {
				is_left_ex = true;
				h_left = data2;
			}
			else if (abs(data1.right.x - data2.left.x) < w_unit_deviation &&
				abs(data1.right.y - data2.left.y) < w_unit_deviation) {
				is_right_ex = true;
				h_right = data2;
			}
		}
		if (!is_left_ex && !is_right_ex) {
			lineH_list.push_back(data1);
		}
		else {
			if (is_left_ex && is_right_ex) {
				data1.left = h_left.left;
				data1.right = h_left.right;
			}
			else if (is_left_ex) {
				data1.left = h_left.left;
			}
			else if (is_right_ex) {
				data1.right = h_left.right;
			}
			data1.width = data1.right.x - data1.left.x;
			data1.w_devia = max_w_deviation_rate * data1.width;
			lineH_list.push_back(data1);
		}
	}


#define AllowOverran 1
#define PARAMENTS_max_deviation_rate	max_h_deviation_rate,max_w_deviation_rate

	bool builed = false;
	list< LineVData>::iterator lineV = lineV_list.begin();


	while (lineV != lineV_list.end()) {
		bool builded = false;
		lineV->sellected = true;
		for (list< LineHData>::iterator lineH = lineH_list.begin();
			lineH != lineH_list.end(); lineH++) {

			lineH->sellected = true;

#define lineH_rightTo_lineV(_lineH,_lineV)  (_lineH->left.x+max_h_width/2+AllowOverran)>=_lineV->x \
&& 	(_lineH->left.x - _lineV->x) < _lineH->w_devia
#define lineH_leftTo_lineV(_lineH,_lineV) (_lineV->x + max_h_width / 2 + AllowOverran) >= _lineH->right.x \
			&& (_lineV->x - _lineH->right.x) < _lineH->w_devia
#define nodeY_upTo_lineV(_node_y,_lineV) (_node_y - 2 * AllowOverran) <= _lineV->y_up \
			&& (_lineV->y_up - _node_y) < _lineV->h_devia
#define nodeY_downTo_lineV(_node_y,_lineV) (_node_y + 2 * AllowOverran) >= _lineV->y_down \
			&& (_node_y - _lineV->y_down) >= _lineV->h_devia

			Rect rect = Rect();
			if (lineH_rightTo_lineV(lineH, lineV)) {
				int node_y = nodeY(lineH, lineV);
				if (nodeY_upTo_lineV(node_y,lineV)) {					
					//左上
					pLineHTmpData lineH2;
					pLineVTmpData lineV2;
					bool lineH2_exist = false, lineV2_exist = false;
					for (lineH2 = lineH_list.begin();
						lineH2 != lineH_list.end(); lineH2++) {
						if (lineH2->sellected)continue;
						if (lineH_rightTo_lineV(lineH2, lineV)
							) {
							int node_y2 = nodeY(lineH2,lineV);
							if (nodeY_downTo_lineV(node_y2,lineV)) {
								//左 上 下
								lineH2_exist = true;
								break;
							}
						}
					}
					for (lineV2 = lineV_list.begin();
						lineV2 != lineV_list.end(); lineV2++) {

						if (lineV2->sellected)continue;
						if (lineH_leftTo_lineV(lineH,lineV2)) {
							int node_y2 = nodeY(lineH, lineV2);
							if (nodeY_upTo_lineV(node_y2, lineV2)) {
								//左 上 右
								lineV2_exist = true;
								break;
							}
						}
					}
			
					 if (is_detect3lines && lineH2_exist && !lineV2_exist) {
						 for (lineV2 = lineV_list.begin();
							 lineV2 != lineV_list.end(); lineV2++) {
							 if (lineV2->sellected)continue;
							 if (lineH_leftTo_lineV(lineH2, lineV2)) {
								 int node_y2 = nodeY(lineH2, lineV2);
								 if (nodeY_downTo_lineV(node_y2, lineV2)) {
									 //左 上 下 右
									 lineV2_exist = true;
									 break;
								 }
							 }
						 }
						 if (!lineV2_exist) {
							 int rectRightX = max(lineH->right.x, lineH2->right.x);
							 int node_y2 = nodeY(lineH, rectRightX);
							 int node_y3 = nodeY(lineH2, lineV);
							 int node_y4 = nodeY(lineH2, rectRightX);
							 int rect_y = (node_y + node_y2) / 2;
							 rect = Rect(lineV->x, rect_y,
								 rectRightX - lineV->x, (node_y3 + node_y4) / 2 - rect_y);
							 builded = true;
							 lineH_list.erase(lineH2);
						 }

					}
					else if (is_detect3lines && lineV2_exist && !lineH2_exist) {

						 for (lineH2 = lineH_list.begin();
							 lineH2 != lineH_list.end(); lineH2++) {
							 if (lineH2->sellected)continue;
							 if (lineH_leftTo_lineV(lineH2, lineV2)
								 ) {
								 int node_y2 = nodeY(lineH2, lineV2);
								 if (nodeY_downTo_lineV(node_y2, lineV2)) {
									 //左 上 右 下 
									 lineH2_exist = true;
									 break;
								 }
							 }
						 }
						 if (!lineH2_exist) {
							 int node_y2 = nodeY(lineH, lineV2);
							 int rect_y = (node_y + node_y2) / 2;
							 int rectdownY = max(lineV->y_down, lineV2->y_down);
							 rect = Rect(lineV->x, rect_y,
								 lineV2->x - lineV->x, rectdownY - rect_y);
							 builded = true;
							 lineV_list.erase(lineV2);
						 }
					}
					 if (lineH2_exist && lineV2_exist) {
						 int node_y2 = nodeY(lineH, lineV2);
						 int node_y3 = nodeY(lineH2, lineV);
						 int node_y4 = nodeY(lineH2, lineV2);
						 int rect_y = (node_y + node_y2) / 2;
						 rect = Rect(lineV->x, rect_y,
							 lineV2->x - lineV->x, (node_y3 + node_y4) / 2 - rect_y);
						 builded = true;
						 lineH_list.erase(lineH2);
						 lineV_list.erase(lineV2);
					 }

				}
				else if (nodeY_downTo_lineV(node_y, lineV)) {
					//左下
					pLineHTmpData lineH2;
					pLineVTmpData lineV2;
					bool lineH2_exist = false, lineV2_exist = false;
					for (lineH2 = lineH_list.begin();
						lineH2 !=lineH_list.end(); lineH2++) {
						if (lineH2->sellected)continue;
						if (lineH_rightTo_lineV(lineH2, lineV)
							) {
							int node_y2 = nodeY(lineH2, lineV);
							if (nodeY_upTo_lineV(node_y2, lineV)) {
								//左  下 上
								lineH2_exist = true;
								break;
							}
						}

					}
					for (lineV2 = lineV_list.begin();
						lineV2 != lineV_list.end(); lineV2++) {
						if (lineV2->sellected)continue;
						if (lineH_leftTo_lineV(lineH, lineV2)) {
							int node_y2 = nodeY(lineH, lineV2);
							if (nodeY_downTo_lineV(node_y2, lineV2)) {
								//左 下 右
								lineV2_exist = true;
								break;
							}
						}
					}
					if (is_detect3lines && lineH2_exist && !lineV2_exist) {
						for (lineV2 = lineV_list.begin();
							lineV2 != lineV_list.end(); lineV2++) {
							if (lineV2->sellected)continue;
							if (lineH_leftTo_lineV(lineH2, lineV2)) {
								int node_y2 = nodeY(lineH2, lineV2);
								if (nodeY_upTo_lineV(node_y2, lineV2)) {
									//左 下 上 右
									lineV2_exist = true;
									break;
								}
							}
						}
						if (!lineV2_exist) {
							int rectRightX = max(lineH->right.x, lineH2->right.x);
							int node_y2 = nodeY(lineH, rectRightX);
							int node_y3 = nodeY(lineH2, lineV);
							int node_y4 = nodeY(lineH2, rectRightX);
							int rect_y = (node_y3 + node_y4) / 2;
							rect = Rect(lineV->x, rect_y,
								rectRightX - lineV->x, (node_y + node_y2) / 2 - rect_y);
							builded = true;
							lineH_list.erase(lineH2);
						}
					}
					else if (is_detect3lines && lineV2_exist && !lineH2_exist) {
						for (lineH2 = lineH_list.begin();
							lineH2 != lineH_list.end(); lineH2++) {
							if (lineH2->sellected)continue;
							if (lineH_leftTo_lineV(lineH2, lineV2)
								) {
								int node_y2 = nodeY(lineH2, lineV2);
								if (nodeY_upTo_lineV(node_y2, lineV2)) {
									//左 下 右 上
									lineH2_exist = true;
									break;
								}
							}
						}
						if (!lineH2_exist) {
							int node_y2 = nodeY(lineH, lineV2);
							int rect_y = min(lineV->y_up, lineV2->y_up);
							int rectdownY = (node_y + node_y2) / 2;
							rect = Rect(lineV->x, rect_y,
								lineV2->x - lineV->x, rectdownY - rect_y);
							builded = true;
							lineV_list.erase(lineV2);
						}
					}
					if (lineH2_exist && lineV2_exist) {
						int node_y2 = nodeY(lineH, lineV2);
						int node_y3 = nodeY(lineH2, lineV);
						int node_y4 = nodeY(lineH2, lineV2);
						int rect_y = (node_y3 + node_y4) / 2;
						rect = Rect(lineV->x, rect_y,
							lineV2->x - lineV->x, (node_y + node_y2) / 2 - rect_y);
						builded = true;
						lineH_list.erase(lineH2);
						lineV_list.erase(lineV2);
					}
				}

			}
			else if (lineH_leftTo_lineV(lineH, lineV)) {
				int node_y = nodeY(lineH, lineV);
				if (nodeY_upTo_lineV(node_y, lineV)) {
					//右上
					pLineHTmpData lineH2 ;
					pLineVTmpData lineV2 ;
					bool lineH2_exist = false, lineV2_exist = false;
					for (lineH2 = lineH_list.begin();
						lineH2 != lineH_list.end(); lineH2++) {
						if (lineH2->sellected)continue;
						if (lineH_leftTo_lineV(lineH2, lineV)
							) {
							int node_y2 = nodeY(lineH2, lineV);
							if (nodeY_downTo_lineV(node_y2, lineV)) {
								//右 上 下
								lineH2_exist = true;
								break;
							}
						}
					}
					for (lineV2 = lineV_list.begin();
						lineV2 != lineV_list.end(); lineV2++) {
						if (lineV2->sellected)continue;
						if (lineH_rightTo_lineV(lineH, lineV2)) {
							int node_y2 = nodeY(lineH, lineV2);
							if (nodeY_upTo_lineV(node_y2, lineV2)) {
								//右 上 左
								lineV2_exist = true;
								break;
							}
						}
					}
					if (is_detect3lines && lineH2_exist && !lineV2_exist) {
						for (lineV2 = lineV_list.begin();
							lineV2 != lineV_list.end(); lineV2++) {
							if (lineV2->sellected)continue;
							if (lineH_rightTo_lineV(lineH2, lineV2)) {
								int node_y2 = nodeY(lineH2, lineV2);
								if (nodeY_downTo_lineV(node_y2, lineV2)) {
									//右 上 下 左
									lineV2_exist = true;
									break;
								}
							}
						}
						if (!lineV2_exist) {
							int rectX = min(lineH->left.x, lineH2->left.x);
							int node_y2 = nodeY(lineH, rectX);
							int node_y3 = nodeY(lineH2, lineV);
							int node_y4 = nodeY(lineH2, rectX);
							int rect_y = (node_y + node_y2) / 2;
							rect = Rect(rectX, rect_y,
								lineV->x - rectX, (node_y3 + node_y4) / 2 - rect_y);
							builded = true;
							lineH_list.erase(lineH2);
						}
					}
					else if (is_detect3lines && lineV2_exist && !lineH2_exist) {
						for (lineH2 = lineH_list.begin();
							lineH2 != lineH_list.end(); lineH2++) {
							if (lineH2->sellected)continue;
							if (lineH_rightTo_lineV(lineH2, lineV2)
								) {
								int node_y2 = nodeY(lineH2, lineV2);
								if (nodeY_downTo_lineV(node_y2, lineV2)) {
									//右 上 左 下
									lineH2_exist = true;
									break;
								}
							}
						}
						if (!lineH2_exist) {
							int node_y2 = nodeY(lineH, lineV2);
							int rect_y = (node_y + node_y2) / 2;
							int rectdownY = max(lineV->y_down, lineV2->y_down);
							rect = Rect(lineV2->x, rect_y,
								lineV->x - lineV2->x, rectdownY - rect_y);
							builded = true;
							lineV_list.erase(lineV2);
						}
					}
					if (lineH2_exist && lineV2_exist) {
						int node_y2 = nodeY(lineH, lineV2);
						int node_y3 = nodeY(lineH2, lineV);
						int node_y4 = nodeY(lineH2, lineV2);
						int rect_y = (node_y + node_y2) / 2;
						rect = Rect(lineV2->x, rect_y,
							lineV->x - lineV2->x, (node_y3 + node_y4) / 2 - rect_y);
						builded = true;
						lineH_list.erase(lineH2);
						lineV_list.erase(lineV2);
					}
				}
				else if (nodeY_downTo_lineV(node_y, lineV)) {
					//右下
					pLineHTmpData lineH2 ;
					pLineVTmpData lineV2 ;
					bool lineH2_exist = false, lineV2_exist = false;
					for (lineH2 = lineH_list.begin();
						lineH2 != lineH_list.end(); lineH2++) {
						if (lineH2->sellected)continue;
						if (lineH_leftTo_lineV(lineH2, lineV)
							) {
							int node_y2 = nodeY(lineH2, lineV);
							if (nodeY_upTo_lineV(node_y2, lineV)) {
								//右  下 上
								lineH2_exist = true;
								break;
							}
						}

					}
					for (lineV2 = lineV_list.begin();
						lineV2 != lineV_list.end(); lineV2++) {
						if (lineV2->sellected)continue;
						if (lineH_rightTo_lineV(lineH, lineV2)) {
							int node_y2 = nodeY(lineH, lineV2);
							if (nodeY_downTo_lineV(node_y2, lineV2)) {
								//右 下 左
								lineV2_exist = true;
								break;
							}
						}
					}
					if (is_detect3lines && lineH2_exist && !lineV2_exist) {
						for (lineV2 = lineV_list.begin();
							lineV2 != lineV_list.end(); lineV2++) {
							if (lineV2->sellected)continue;
							if (lineH_rightTo_lineV(lineH2, lineV2)) {
								int node_y2 = nodeY(lineH2, lineV2);
								if (nodeY_upTo_lineV(node_y2, lineV2)) {
									//右 下 上 左
									lineV2_exist = true;
									break;
								}
							}
						}
						if (!lineV2_exist) {
							int rectX = min(lineH->left.x, lineH2->left.x);
							int node_y2 = nodeY(lineH, rectX);
							int node_y3 = nodeY(lineH2, lineV);
							int node_y4 = nodeY(lineH2, rectX);
							int rect_y = (node_y3 + node_y4) / 2;
							rect = Rect(rectX, rect_y,
								lineV->x - rectX, (node_y + node_y2) / 2 - rect_y);
							builded = true;
							lineH_list.erase(lineH2);
						}
					}
					else if (is_detect3lines && lineV2_exist && !lineH2_exist) {
						for (lineH2 = lineH_list.begin();
							lineH2 != lineH_list.end(); lineH2++) {
							if (lineH2->sellected)continue;
							if (lineH_rightTo_lineV(lineH2, lineV2)
								) {
								int node_y2 = nodeY(lineH2, lineV2);
								if (nodeY_upTo_lineV(node_y2, lineV2)) {
									//右  下 左 上
									lineH2_exist = true;
									break;
								}
							}
						}
						if (!lineH2_exist) {
							int node_y2 = nodeY(lineH, lineV2);
							int rect_y = min(lineV->y_up, lineV2->y_up);
							int rectdownY = (node_y + node_y2) / 2;
							rect = Rect(lineV2->x, rect_y,
								lineV->x - lineV2->x, rectdownY - rect_y);
							builded = true;
							lineV_list.erase(lineV2);
						}
					}
					if (lineH2_exist && lineV2_exist) {
						int node_y2 = nodeY(lineH, lineV2);
						int node_y3 = nodeY(lineH2, lineV);
						int node_y4 = nodeY(lineH2, lineV2);
						int rect_y = (node_y3 + node_y4) / 2;
						rect = Rect(lineV2->x, rect_y,
							lineV->x - lineV2->x, (node_y + node_y2) / 2 - rect_y);
						builded = true;
						lineH_list.erase(lineH2);
						lineV_list.erase(lineV2);
					}
				}
			}
			if (!rect.empty()) {
				bool do_return=false;
				if (_detect_rect_over_(rect, is_org, do_return)) {
					out = rect;
					return true;
				}
				else if (do_return) {
					return false;				
				}

			}
			if (builded) {
				lineH_list.erase(lineH);
				break;
			}
			lineH->sellected = false;
		}
		if (builded) {
			pLineVTmpData pVTmp = lineV;
			lineV++;
			bool isEnd = (lineV == lineV_list.end());
			lineV_list.erase(pVTmp);
			if(isEnd)break;
		}
		else {
			lineV->sellected = false;
			lineV++;
		}
		
	}	
	return false;
}



bool DoorMonitor::_detect_rect_over_(Rect& rect,bool is_org,
	bool & do_return) {
	if (is_org) {
		bool is_show_org_org = is_show_org;
		is_show_org = true;
		mark_org_mutex.lock();
		Rect mark_org_org = mark_org;
		mark_org = rect;
		mark_org_mutex.unlock();
		cout << "是否选择该矩形为标记初始位置?";
		short ret= affirm2();
		if (ret==YES || ret==YES_ALL) {
			is_show_org = is_show_org_org;
			//mark_center_color = rectCenterColor(image, rect);
			return true;
		}
		else if (ret==NO_ALL) {
			do_return = true;
		}
		mark_org_mutex.lock();
		mark_org = mark_org_org;
		mark_org_mutex.unlock();
		is_show_org = is_show_org_org;
		return false;
	}
	else {
		mark_org_mutex.lock();		
		if (abs(rect.height - mark_org.height) >
			max_h_deviation_rate_cmp* mark_org.height)
		{
			mark_org_mutex.unlock();
			return false;
		}
		mark_org_mutex.unlock();
		/*
		last_item_mutex.lock();
		if (recentDetectItems.empty()) {
			camera->param_mutex.lock();
			recentDetectItems.push_back(RectDetectItem(mark_org,camera->frame_counter));
			camera->param_mutex.unlock();
		}
			//mark_org_mutex.unlock();
		*/
		/*
		Vec3b color = rectCenterColor(image,rect);
		uint dist = distance(color, mark_center_color);
		if (dist > mark_center_color_deviation) {
			mark_org_mutex.unlock();
		return false;
		}
		cout << (uint)color[0]  << " " << (uint)color[1]
		<< " " << (uint)color[2]  <<" "<< dist << endl;
		*/
		bool is_serialed = false;


		rect_detect_parameters->this_mutex.lock_shared();
		for (pRectDetectItem item = recentDetectItems.begin(); 
			item != recentDetectItems.end();item++) {
			if (item->serial(rect)) {
				is_serialed = true;
				item->newRect(rect,current_item->frame);
				if (item->isMark()) {
					rect_detect_parameters->this_mutex.unlock_shared();
					return true;
				}
			}
		}
		if (!is_serialed) {
			thisDetectItems.push_back(RectDetectItem
			(rect_detect_parameters,rect, current_item->frame));
		}
		rect_detect_parameters->this_mutex.	unlock_shared();
		return false;

	}

}


//old_version_detect_rect
/*
//也可以,但会出现8种情况
if (lineH->is_left_incline) {
	if ((lineH->left.y - AllowOverran) <= lineV->y_up)//左上
	{
		if () {

			for (LineHTmpData* lineH2 : lineH_list) {
#define NODEY2	nodeY(lineH2,lineV)
								if (lineH2->sellected)continue;
								if ((lineH2->left.x + min_h_width / 2 + AllowOverran)
									>= lineV->x &&
									(lineH->left.x - lineV->x) < lineH->w_devia ) {
									int nodey2 = NODEY2;
									if (nodey2 > lineV->y_down &&
										(nodey2 - lineV->y_down) < lineV->h_devia) {

									}
									//..........
								}
							}
							for (LineVTmpData* lineV2 : lineV_list) {
#define NODEY3	nodeY(lineH,lineV2)
								if (lineV2->sellected)continue;

								if ((lineV2->x + min_h_width / 2 + AllowOverran) >=
									lineH->right.x
									&& (lineV2->x - lineH->right.x) < lineH->w_devia&&
									) {

								}

							}
						}
					}
					else if ((NODEY- lineV->y_down) < lineV->h_devia)//左下
					{

					}
				}
				else {
					if ()//左下
					{
						if ((NODEY - lineV->y_down) < lineV->h_devia) {

						}
					}
					else if ((lineV->y_up - NODEY) < lineV->h_devia) //左上
					{

					}
				}
			}
			else if((lineV->x + min_h_width / 2 + AllowOverran) >= lineH->right.x
				&& (lineV->x - lineH->right.x) < lineH->w_devia){

				if (lineH->is_left_incline) {
					if ((lineH->right.y- AllowOverran) < lineV->y_up)//右上
					{
						if ((lineV->y_up - NODEY) < lineV->h_devia) {

						}
					}
					else if ((NODEY - lineV->y_down) < lineV->h_devia)//右下
					{

					}
				}
				else {
					if ((lineH->right.y+ AllowOverran) > lineV->y_down)//右下
					{
						if ((NODEY - lineV->y_down) < lineV->h_devia) {

						}
					}
					else if ((lineV->y_up - NODEY) < lineV->h_devia) //右上
					{

					}
				}*/

/*
#define PARAMENTS_max_deviation_rate	max_h_deviation_rate,max_w_deviation_rate
		for (LineHTmpData* hData : lineHdatas) {
			//cout << j++ << endl;

			//int devia_short = 2; int devia_long = 10;


#define AllowOverran 2
			if ( (hData->x_left + AllowOverran) > lineV_X &&
				hData->x_left - lineV_X < hData->w_devia) {

				if ((lineV_Y_up + AllowOverran) > hData->y &&
					lineV_Y_up - hData->y < h_devia ) {

					/*
					detect_res.push_back(new FlagDetectItem(
						Rect(lineV_X, hData->y,
						 hData->x_right - lineV_X , lineV_Y_down- hData->y),
						PARAMENTS_max_deviation_rate
					));
					for (Vec4i lineV : lines_v) {



					}

				}
				else if (
					(hData->y + AllowOverran) > lineV_Y_down &&
					hData->y- lineV_Y_down < h_devia) {
					detect_res.push_back(new FlagDetectItem(
						Rect(lineV_X,lineV_Y_up,
							hData->x_right - lineV_X, hData->y- lineV_Y_up),
						PARAMENTS_max_deviation_rate
					));
				}
			}
			else if (abs(lineV_X - hData->x_right) < hData->w_devia) {
				if (abs(lineV_Y_up - hData->y) < hData->w_devia) {
					detect_res.push_back(new FlagDetectItem(
						Rect((lineV_X + hData->x_right) / 2 - hData->width,
						(lineV_Y_up + hData->y) / 2, hData->width, lineV_height),
						PARAMENTS_max_deviation_rate
					));
				}
				else if (abs(lineV_Y_down - hData->y) < hData->w_devia) {
					detect_res.push_back(new FlagDetectItem(
						Rect((lineV_X + hData->x_right) / 2 - hData->width,
						(lineV_Y_down + hData->y) / 2 - lineV_height, hData->width, lineV_height),
						PARAMENTS_max_deviation_rate

			));
				}
			}
		}
	}//for (Vec4i lineV : lines_v)

	//static int i = 0, j = 0, k = 0;
	//static long size_sum = 0;
	//size_sum += detect_res.size();
	//cout << ++i << ",";
	//cout << detect_res.size() << "("<< (double)size_sum/i<<"),";


	for (Vec4i lineV : lines_v) {
		int lineV_X = (lineV[0] + lineV[2]) / 2;
		int y_up = min(lineV[1], lineV[3]);
		int y_down = max(lineV[1], lineV[3]);
		int height = abs(lineV[1] - lineV[3]);
		for (FlagDetectItem* item : detect_res) {
			Rect& rect = item->rect;
			if (abs(height - rect.height) > item->h_devia)continue;
			int h = (height + rect.height) / 2;
			//cout << item->h_devia << " " << item->w_devia  << ",u " <<
			//	y_up << " d "<<y_down <<" "<< rect.y <<" "<<h <<" LL "<< lineV_X << endl;

			if ((abs(y_up - rect.y) < item->h_devia)
				&& (abs(y_down - rect.y - h) < item->h_devia)) {

				if (abs(lineV_X - rect.x) < item->w_devia)
					item->matchedLines = item->matchedLines | 0b0001;
				else if (abs(lineV_X - rect.x - rect.width) < item->w_devia)
					item->matchedLines = item->matchedLines | 0b0010;
			}
		}
	}
	for (LineHTmpData* hData : lineHdatas) {

		for (FlagDetectItem* item : detect_res) {
			Rect& rect = item->rect;

			if (abs(hData->width - rect.width) > item->w_devia)continue;

			if (abs(hData->x_left - rect.x) < item->w_devia
				&& abs(hData->x_right - rect.x - rect.width) < item->w_devia) {

				if (abs(hData->y - rect.y) < item->h_devia)
					item->matchedLines = item->matchedLines | 0b0100;
				else if (abs(hData->y - rect.y - rect.height) < item->h_devia)
					item->matchedLines = item->matchedLines | 0b1000;

			}

			short count = (item->matchedLines >> 3) + ((item->matchedLines >> 2) & 0b0001) +
				((item->matchedLines >> 1) & 0b0001) + ((item->matchedLines) & 0b0001);
			//cout << item->matchedLines<<" "<< ((item->matchedLines >> 2) & 0b0001)<<" " << count
			//	<<endl;
			if (count  >=  3) {

				int lines_counter = 0;
				/*
				for (Vec4i line : lines) {
					if (line[0] > item->rect.x&& line[0] < item->rect.x + item->rect.width &&
						line[2] > item->rect.x&& line[2] < item->rect.x + item->rect.width &&
						line[1] > item->rect.y&& line[1] < item->rect.y + item->rect.height &&
						line[3] > item->rect.y&& line[3] < item->rect.y + item->rect.height &&
						sqrt(pow(line[2] - line[0], 2) + pow(line[3] - line[1], 2))>
						item->rect.width / mark_intra_lines_minlen_rate)
						lines_counter++;
					if (lines_counter >= mark_intra_lines)break;
				}
				if (lines_counter < mark_intra_lines)continue;

				if (is_org) {
					bool is_show_org_org = is_show_org;
					is_show_org = true;
					mark_org_mutex.lock();
					Rect mark_org_org = mark_org;
					mark_org = item->rect;
					mark_org_mutex.unlock();
					cout << "是否选择该矩形为标记初始位置?";
					if (affirm()) {
						for (LineHTmpData* hData : lineHdatas)
							delete hData;

						for (FlagDetectItem* item : detect_res)
							delete item;
						is_show_org = is_show_org_org;
						mark_org_mutex.lock();
						mark_center_color = rectCenterColor(image, mark_org);
						mark_org_mutex.unlock();

						//cout << k++ <<"("<<(float)k/i<<")"<< endl;
						return true;
					}
					else {
						mark_org_mutex.lock();
						mark_org = mark_org_org;
						mark_org_mutex.unlock();
						is_show_org = is_show_org_org;
						continue;
					}
				}
				else {
					mark_org_mutex.lock();
					if (abs(item->rect.height - mark_org.height) >
						max_h_deviation_rate_cmp* mark_org.height) {
						mark_org_mutex.unlock();
						continue;
					}

					if ((item->rect.width - mark_org.width) >
						max_w_deviation_rate* mark_org.width) {
						mark_org_mutex.unlock();
						continue;
					}
					Vec3b color = rectCenterColor(image, item->rect);

					uint dist = distance(color, mark_center_color);
					if (dist > mark_center_color_deviation) {
						mark_org_mutex.unlock();
						continue;
					}

					mark_org_mutex.unlock();

					out = item->rect;
					for (LineHTmpData* hData : lineHdatas)
						delete hData;

					for (FlagDetectItem* item : detect_res)
						delete item;

					//cout << k++ << "(" << (float)k / i << ")" << endl;
					return true;
				}

			}
		}
	}
	for (LineHTmpData* hData : lineHdatas)
		delete hData;
	for (FlagDetectItem* item : detect_res)
		delete item;
	//cout <<"failed " << k << "(" << (float)k / i << ")" << endl;
	return false;
	*/


bool DoorMonitor::detect_rect(Mat& image, Rect& out,
	vector<Vec4i>& lines,bool is_org, Mat* canny) {
	max_deviation_rate_mutex.lock();
	bool ret=_detect_rect_(image, out, lines, is_org, canny);
	max_deviation_rate_mutex.unlock();
	return ret;
}
mutex detect_mark_mutex;
void DoorMonitor::detect_mark_org() {
	detect_mark_mutex.lock();
	long frame_begin;
	long frame = 0;
	mark_range_mutex.lock();
	if (mark_range.empty()) {
		cout << "监测范围为空" << endl;
		mark_range_mutex.unlock();
		detect_mark_mutex.unlock();
		return;
	}
	Rect range = mark_range;
	mark_range_mutex.unlock();

	cam_current_mutex.lock_shared();
	frame_begin = camera->frame_counter;
	cam_current_mutex.unlock_shared();

	camera->param_mutex.lock();
	int detectMark_frameout = (detectMark_timeout * camera->fps)/1000;
	camera->param_mutex.unlock();

	Timer timerAll(100);
	timerAll.begin();
	while (true) {
		
		cam_current_mutex.lock_shared();
		if (camera->frame_counter - frame_begin > detectMark_frameout ) {
			timerAll.end("识别标记失败");
			cam_current_mutex.unlock_shared();
			detect_mark_mutex.unlock();
			return;
		}
		if (camera->frame_counter - frame < min_workframe) {
			cam_current_mutex.unlock_shared();
			this_thread::sleep_for(chrono::milliseconds(WaitTime));
			continue;
		}
		Mat pic = camera->current;
		frame = camera->frame_counter;
		cam_current_mutex.unlock_shared();
		Mat pic_range = pic(range);
		Rect rect;
		vector<Vec4i> lines;
		if (detect_rect(pic_range, rect,lines, true)) {
			timerAll.end("识别标记成功!");
			detect_mark_mutex.unlock();
			return;
		}

	}


}

bool DoorMonitor::detect_mark() {

	detect_mark_mutex.lock();
	mark_range_mutex.lock();
	current_item->img_range = current_item->img(mark_range).clone();//已经判断过!=empty
	mark_range_mutex.unlock();


	thisDetectItems.clear();

	bool ret = detect_rect(current_item->img_range, current_item->rect,
		current_item->lines, false, &(current_item->out_canny));
	detect_mark_mutex.unlock();
	pRectDetectItem item = recentDetectItems.begin();


	rect_detect_parameters->this_mutex.lock_shared();
	/**/
	if (ret) {
		while (item != recentDetectItems.end()) {
			if (!item->isMark()) {
				pRectDetectItem tmp = item;
				item++;
				bool isEnd = (item == recentDetectItems.end());
				recentDetectItems.erase(tmp);
				if (isEnd)break;
			}
			else item++;
		}

	}
	while (item != recentDetectItems.end()) {
		if ((current_item->frame - item->lastFrame) >
			rect_detect_parameters->series_frame_out) {
			pRectDetectItem tmp = item;
			item++;
			bool isEnd = (item == recentDetectItems.end());
			recentDetectItems.erase(tmp);
			if (isEnd)break;
		}
		else item++;
	}
	rect_detect_parameters->this_mutex.unlock_shared();

	if(!ret)recentDetectItems.splice(recentDetectItems.end(), thisDetectItems);
	//recentDetectItems.merge(recentDetectItemsTemp);
	return ret;
}
void DoorMonitor::door_monitoring() {
	static Timer timer_detectmark(2800);
	timer_detectmark.begin();

	bool ret = detect_mark();

	detect_mark_times_mutex.lock();

	static time_t time_point=time(0);

	time_t now= time(0);
	if (now - time_point > 60) {
		time_point = now;
		detect_mark_times = 0;
		detect_mark_succeed_times = 0;
		detect_mark_failed_before_succeed_maxtimes = 0;
	}
	detect_mark_times++;
	if (ret) {
		detect_mark_failed_before_succeed_maxtimes =
			max(detect_mark_failed_before_succeed_maxtimes,(int)mark_disapeared_times);
		detect_mark_failed_before_succeed_times += mark_disapeared_times;
		detect_mark_succeed_times++;

		detect_mark_avg_failed_before_succeed_times =
			(double)detect_mark_failed_before_succeed_times / detect_mark_succeed_times;
	}
	double Psucceed = (double)
		detect_mark_succeed_times / detect_mark_times;
	detect_mark_times_mutex.unlock();
    
	if (ret) {
		mark_disapeared_times = 0;

		if (is_auto) return;	
		mark_org_mutex.lock();
		Rect org = mark_org;
		mark_org_mutex.unlock();

		threashold_door_mutex.lock();
		if (event_ctrl->door_opened) {
			
			if (abs(current_item->rect.x - org.x) <
				threashold_door_closed * org.width &&
				abs(current_item->rect.y - org.y) <
				threashold_door_closed * org.height &&
				abs(current_item->rect.width - org.width)<
				threashold_door_closed * org.width  &&
				abs(current_item->rect.height - org.height) <
				threashold_door_closed * org.height
				) {
				//mark_center_color = rectCenterColor(
				//	current_item->img_range, current_item->rect);
				if(!is_test)event_ctrl->doorClosing();
			}
			else if (!is_test)event_ctrl->doorOpening();
		}
		else {
			//*******************
			if (abs(current_item->rect.x - org.x) >
				threashold_door_opened* org.width) {
				if (!is_test)event_ctrl->doorOpening();
			}
			else {
				//mark_center_color = rectCenterColor(
				//	current_item->img_range, current_item->rect);
				if (!is_test)event_ctrl->doorClosing();

			}
		}
		threashold_door_mutex.unlock();

		detect_mark_times_mutex.lock();
		if (is_debug)timer_detectmark.end("(识别标记成功) 识别成功频率:" + to_string(Psucceed) +
			" 最高识别失败次数:" + to_string(detect_mark_failed_before_succeed_maxtimes)
		+" ( 前1分钟)");
			//+" 平均识别失败次数:"+ to_string(detect_mark_avg_failed_before_succeed_times));
		detect_mark_times_mutex.unlock();
	}
	else {
		mark_disapeared_times++;
		current_item->rect = Rect();

		if (is_auto) {


			if (mark_disapeared_times * 2 > mark_disapeared_maxtimes) {
				mark_disapeared_maxtimes = mark_disapeared_times * 2;
			}

			if (is_debug)timer_detectmark.end("(识别标记失败) 识别成功频率:" + to_string(Psucceed) +
				" 最高识别失败次数:" + to_string(detect_mark_failed_before_succeed_maxtimes) +
				" 判定标签消失阈值:" + to_string(mark_disapeared_maxtimes) +" ( 前1分钟)");
				//+" 平均识别失败次数:" + to_string(detect_mark_avg_failed_before_succeed_times));
			return;
		}


		
		if (mark_disapeared_times >= mark_disapeared_maxtimes) {
			mark_disapeared_times = 0;
			if (!is_test)event_ctrl->markDisapear();
		}
		detect_mark_times_mutex.lock();
		if (is_debug)timer_detectmark.end("(识别标记失败) 识别成功频率:" + to_string(Psucceed) +
			" 最高识别失败次数:" + to_string(detect_mark_failed_before_succeed_maxtimes) +
			" ( 前1分钟)");
			//+" 平均识别失败次数:" + to_string(detect_mark_avg_failed_before_succeed_times));
		detect_mark_times_mutex.unlock();
	}

}
/*
		vector<Vec4i> good_linesV;

		int h_devia;
		int w_devia = mark_org.width / max_w_deviation_rate;
		if (event_ctrl->door_opened)
			h_devia = mark_org.height / dooropened_max_h_deviation_rate;
		else
			h_devia = mark_org.height / max_h_deviation_rate;

		for (Vec4i lineV : current_item->linesV) {
			int height = abs(lineV[1] - lineV[3]);
			int deviation = abs(height - mark_org.height);
			if (deviation < h_devia)
				good_linesV.push_back(lineV);
		}
		for (int i = 0; i < good_linesV.size(); i++) {
			for (int j = 0; j < good_linesV.size(); j++) {
				if (i == j)continue;
				int i_y_mid = (good_linesV[i][1] + good_linesV[i][3]) / 2;
				int j_y_mid = (good_linesV[j][1] + good_linesV[j][3]) / 2;
				if (abs(i_y_mid - j_y_mid) >
					mark_org.height / max_h_deviation_rate)continue;
				int i_X = (good_linesV[i][0] + good_linesV[i][2]) / 2;
				int j_X = (good_linesV[j][0] + good_linesV[j][2]) / 2;
				int width = abs(i_X - j_X);
				if (event_ctrl->door_opened) {
					if (width > mark_org.width)continue;
				}
				else if (abs(width - mark_org.width) > w_devia)continue;

				int i_y_up = min(good_linesV[i][1], good_linesV[i][3]);
				int j_y_up = min(good_linesV[j][1], good_linesV[j][3]);
				current_item->mark = Rect(
					min(i_X, j_X), (i_y_up + j_y_up) / 2,
					abs(i_X - j_X), (i_y_mid - i_y_up) + (j_y_mid - j_y_up)
				);
				if (abs(current_item->mark.x - mark_org.x) >
					mark_range.width / threashold_door_opened) {
					if (!event_ctrl->door_opened) {
						cout << "door opened!" << endl;
						event_ctrl->door_opened = true;
					}
				}
				else {
					if (event_ctrl->door_opened) {
						cout << "door closed!" << endl;
						event_ctrl->door_opened = false;
					}
				}
					mark_org_mutex.unlock();
				return;
			}
		}
*/

//cmd线程
void DoorMonitor::do_cmd(istringstream& issm)
{
	string arg1,arg2;
	if (issm)issm >> arg1;
	if (arg1 == "show") {
		if (issm)issm >> arg2;
		if (arg2 == "range")
			is_show_mark_range = true;
		else if (arg2 == "mark" || arg2.empty())
			is_show_mark = true;
		else if (arg2 == "lines")
			is_show_lines = true;
		else if (arg2 == "canny") {
			is_show_canny = true;			
		}
		else if (arg2 == "org")
			is_show_org = true;
		else cout << "格式错误" << endl;
	}
	else if (arg1 == "unshow") {
		if (issm)issm >> arg2;
		if (arg2 == "range")
			is_show_mark_range = false;
		else if (arg2 == "mark" || arg2.empty())
			is_show_mark = false;
		else if (arg2 == "lines")
			is_show_lines = false;
		else if (arg2 == "canny") {
			is_show_canny = false;
		}
		else if (arg2 == "org")
			is_show_org = false;
		else cout << "格式错误" << endl;
	}
	else if (arg1 == "range" ) {
		set_range();
	}
	else if (arg1 == "debug" ) {
		is_debug = !is_debug;
		if (is_debug)cout << "已打开调试功能" << endl;
	}
	else if (arg1 == "save") {
		if (issm)issm >> arg2;
		string arg3;
		if (issm)issm >> arg3;
		if (arg2 == "on" && arg3 == "image")
			is_save = true;
		else if (arg2 == "params" || arg2 == "parameters")
			save_params();
	}
	else if (arg1 == "unsave") {
		is_save = false;
	}
	else if (arg1 == "configures") {
#define T_or_F(is_show)	(is_show?"true":"false")
		cout << "door monitor configures:" << endl;
		cout << "test          " << T_or_F(is_test) << endl;
		cout << "debug         " << T_or_F(is_debug) << endl;
		cout << "show (mark)   " << T_or_F(is_show_mark) << endl;
		cout << "show lines    " << T_or_F(is_show_lines) << endl;
		cout << "show range    " << T_or_F(is_show_mark_range) << endl;
		cout << "show org      " << T_or_F(is_show_org) << endl;
		cout << "show canny    " << T_or_F(is_show_canny) << endl;
		cout << "save          " << T_or_F(is_save) << endl;
		cout << "auto          " << T_or_F(is_auto) << endl;

	}
	else if (arg1 == "param") {
		show_param(issm);
	}
	else if (arg1 == "set") {
		cmd_set(issm);
	}
	else if (arg1 == "auto") {
		is_auto = true;
		cout << "已开启自动调节参数模式，请勿遮挡标记或进行任何移动标记的行为" << endl;
	}
	else if (arg1 == "unauto") {
		is_auto = false;
	}
	else if (arg1 == "test") {
		is_test = true;
		cout << "已开启测试模式" << endl;
	}
	else if (arg1 == "untest") {
		is_test = false;
	}
	else if (arg1 == "help" || arg1 == "-h") {
		cout << "range                       设置标记活动范围，识别标记，开始监测" << endl;
		cout << "debug                       打开/取消调试" << endl;
		cout << "test                        开启测试模式，不上报事件" << endl;
		cout << "untest                      关闭测试模式" << endl;
		cout << "show (mark)                 显示标记位置" << endl;
		cout << "show lines                  显示捕捉到的边缘直线" << endl;
		cout << "show range                  显示标记的活动范围" << endl;
		cout << "show org                    显示标记初始位置" << endl;
		cout << "show canny                  显示canny边沿图像" << endl<<endl;
		cout << "unshow (mark)               不显示标记位置" << endl;
		cout << "unshow lines                不显示捕捉到的边缘直线" << endl;
		cout << "unshow range                不显示监测范围" << endl;
		cout << "unshow org                  不显示标记初始位置" << endl;
		cout << "unshow canny                不显示显示canny边沿图像" << endl;
		cout << "save   (on image)           将显示内容保存在图像上" << endl;
		cout << "unsave (on image)           不将显示内容保存在图像上" << endl;
		cout << "configures                  显示以上内容的设置值" << endl;
		cout << "param、set                  显示、设置参数值" << endl;
		cout << "auto                        开启自动调节参数模式" << endl;
		cout << "unauto                      关闭自动调节参数模式" << endl;
		cout << "reset parameters/params     重置参数" << endl;
		cout << "save parameters/params      将参数设置保存到本地文件中" << endl;
		cout << "load parameters/params      从本地文件读取各参数设置" << endl;
	}
	else cout << "错误指令" << endl;
}



//主线程
void DoorMonitor::when_showPic(Mat& pic) {

	range_temp_mutex.lock();
	if (!range_temp.empty()) {
		rectangle(pic, range_temp, Scalar(255, 0, 0), 2);
	}
	range_temp_mutex.unlock();

	if (is_show_mark_range) {
		mark_range_mutex.lock();
		if(!mark_range.empty())
		rectangle( pic, mark_range, Scalar(0, 255, 0), 2);
		mark_range_mutex.unlock();
	}

	if (is_show_org) {
		mark_range_mutex.lock();
		Rect range = mark_range;
		mark_range_mutex.unlock();
		mark_org_mutex.lock();
		if (!mark_org.empty()) {
			rectangle(pic, Rect(mark_org + Point2i(range.x, range.y)),
				Scalar(255, 255, 0), 2);
		}
		mark_org_mutex.unlock();

	}
	if (is_show_mark) {
		last_item_mutex.lock();
		if(last_item != NULL){
			mark_range_mutex.lock();
			Rect range = mark_range;
			mark_range_mutex.unlock();
			if (!last_item->rect.empty()) {
				Rect r2 = Rect(last_item->rect.x + range.x,
					last_item->rect.y + range.y, last_item->rect.width,
					last_item->rect.height);
				rectangle(pic, r2, Scalar(0, 0, 255), 2);
			}

		}
		last_item_mutex.unlock();
	}
	if (is_show_lines) {
		last_item_mutex.lock();
		if (last_item != NULL) {
			mark_range_mutex.lock();
			Rect range = mark_range;
			mark_range_mutex.unlock();

			for (Vec4i I : last_item->lines) {
				//cout << I<<" ";
				int i=rand();
				int j = rand();
				int k = rand();
				line(pic, Point(I[0] + range.x, I[1] + range.y),
					Point(I[2] + range.x, I[3] + range.y),	//绘制直线
					Scalar((double)i / RAND_MAX * 255,
					(double)j / RAND_MAX * 255,
						(double)k / RAND_MAX * 255),2);
			}
		}
		last_item_mutex.unlock();
	}



	static bool show_canny_window_opened = false;
	if (is_show_canny) {
		if (!show_canny_window_opened) {
			namedWindow("canny", cv::WINDOW_NORMAL);
			show_canny_window_opened = true;
		}
		mark_range_mutex.lock();
		if (range_changed) {

			int h = (mark_range.height*800)/ mark_range.width;
			resizeWindow("canny",Size(800,h));
			range_changed = false;
		}
		mark_range_mutex.unlock();
		last_item_mutex.lock();
		if(last_item != NULL)
		cv::imshow("canny", last_item->out_canny);
		last_item_mutex.unlock();
	}
	else if (show_canny_window_opened) {
		cv::destroyWindow("canny");
		show_canny_window_opened = false;
	}



}
#define CONVERT_ORG_DST cv::getWindowImageRect(main_win_name).size(),camera.size


void DoorMonitor::on_mouse_selrange(int event, int x, int y)
{
	static bool is_seling = false;
	if (!flag_setrange)return;
	switch (event) { 
	case cv::EVENT_LBUTTONDOWN:
		range_temp_mutex.lock();
		range_temp = Rect();
		range_temp_mutex.unlock();
		flag_finished_setrange = false;

		pFirst = Point2i(x, y);
		is_seling = true;
		return;
	case cv::EVENT_MOUSEMOVE:
		if (!is_seling)return;
		range_temp_mutex.lock();
		range_temp = Rect(pFirst,Point2i(x, y));

		range_temp_mutex.unlock();
		break;
	case cv::EVENT_LBUTTONUP:
		range_temp_mutex.lock();
		range_temp = Rect(pFirst, Point2i(x, y));
		range_temp_mutex.unlock();
		flag_finished_setrange = true;
		is_seling = false;
		break;
	default:return;
	}	
}
bool cmpFunc(Vec4i l1, Vec4i l2) {

	int w1 = abs(l1[2] - l1[0]);
	int w2 = abs(l2[2] - l2[0]);
	int h1 = abs(l1[3] - l1[1]);
	int h2 = abs(l2[3] - l2[1]);
	double len1 = pow(w1, 2) + pow(h1, 2);
	double len2 = pow(w2, 2) + pow(h2, 2);
	return w1<w2 && len1 > len2;

}

void filter_angle(vector<cv::Vec4i>& lines,
	list<cv::Vec4i>& lines_out,
	double lt, double errThetarange) {
	if (lines.empty())return;
	lt = lt * CV_PI / 180;
	errThetarange= errThetarange * CV_PI / 180;
	double lt2= lt > 0 ?
		(lt - CV_PI) : (lt + CV_PI);
	for (Vec4i I : lines) {
		double w = (double)I[2] - I[0];
		double h = (double)I[3] - I[1];
		double theta = atan2(-h, w);
		if (abs(theta - lt) < errThetarange ||
			abs(theta - lt2) < errThetarange)
		lines_out.push_back(I);
	}
}
void filter_angle(vector<cv::Vec4i>& lines,
	list<pair< cv::Vec4i,double> >& lines_out_theta,
	double lt, double errThetarange) {
	if (lines.empty())return;
	lt = lt * CV_PI / 180;
	errThetarange = errThetarange * CV_PI / 180;
	double lt2 = lt > 0 ?
		(lt - CV_PI) : (lt + CV_PI);
	for (Vec4i I : lines) {
		double w = (double)I[2] - I[0];
		double h = (double)I[3] - I[1];
		double theta = atan2(-h, w);
		if (abs(theta - lt) < errThetarange ||
			abs(theta - lt2) < errThetarange)
			lines_out_theta.push_back(pair< cv::Vec4i, double>(I, theta));
	}
}
/*
void DoorMonitor::filter_vertical(
	vector<cv::Vec4i>& lines, vector<cv::Vec4i>& lines_out) {
	

	mark_org_mutex.lock();
	double lt		= CV_PI / 2;
	mark_org_mutex.unlock();
	double alow_t	= max_allow_v_errTheta;

	filter_angle(lines, lines_out, lt, alow_t);
}
*/
Vec3b DoorMonitor::rectCenterColor(cv::Mat& img, cv::Rect& rect)
{
	unsigned long sum[3] = { 0,0,0 };
	int step_w = rect.width / 20;
	int step_h = rect.height / 20;

	int h_index = rect.y + 8 * step_h;
	for (int i = 0; i < 5; i++) {

		Vec3b* it = img.ptr<Vec3b>(h_index);

		int w_index = rect.x + 8 * step_w;
		for (int j = 0; j < 5; j++) {

			sum[0] += it[w_index][0];
			sum[1] += it[w_index][1];
			sum[2] += it[w_index][2];
			w_index += step_w;
		}
		h_index += step_h;
	}

	return Vec3b(sum[0] / 25, sum[1] / 25, sum[2] / 25);
}

uint DoorMonitor::distance(const cv::Vec3b v1, const cv::Vec3b v2)
{
	return (uint)sqrt(
		pow((int)v1[0] - v2[0], 2) +
		pow((int)v1[1] - v2[1], 2) +
		pow((int)v1[2] - v2[2], 2));
}
/*
void DoorMonitor::flush_hough_minLineLength() {
	mark_range_mutex.lock();
	double h = mark_range.height;
	mark_range_mutex.unlock();
	hough_mutex.lock();
	this->hough_minLineLength = (double)h * this->hough_minLineLenRate;
	hough_mutex.unlock();
}
*/

void DoorMonitor::cmd_set(istringstream& issm) {

	string param;
	if (issm)issm >> param;
	
	if(param == "1" || param == "do_blur") {
		string value; issm >> value;
		hough_mutex.lock();
		if (value == "true") 
			do_blur = true;
		else if (value == "false") 
			do_blur = false;		
		else 
			cout << "格式错误，值应为true/false" << endl;
		hough_mutex.unlock();
		return;
	}
	else if (param == "help" || param == "-h") {
		cout << "设置door monitor的参数" << endl;
		cout << "格式: set dm (param) (value) ... " << endl;
		cout << "param -- 参数名 参考 param dm,可为序号或括号内的标识符" << endl;
		cout << "value -- 参数值 多个值以空格分开" << endl;
	}
	double value = 0;
	if (issm)issm >> value;
	if (param == "2" || param == "blur_sigma") {
		hough_mutex.lock(); 
		if (value != 0)gaussianblur_sigma = value;
		else cout << "格式错误" << endl;
		hough_mutex.unlock();
	}
	else if (param == "3" || param == "blur_ksize") {
		hough_mutex.lock();
		int v2 = 0; issm >> v2;
		if (value != 0 && v2 != 0) {
			gaussianblur_ksize = Size((int)value, v2);
		}
		else cout << "格式错误" << endl;
		hough_mutex.unlock();
	}
	else if (param == "4" || param == "canny_threashhold") {
		hough_mutex.lock();
		int v2 = 0; issm >> v2;
		if (value != 0 && v2 != 0) {
			canny_threashhold_min = value;
			canny_threashhold_max = v2;
		}
		else cout << "格式错误" << endl;
		hough_mutex.unlock();
	}
	else if (param == "5" || param == "hough_threashhold") {
		hough_mutex.lock();
		int val_int= (int)value;
		if (value != 0 && value == (double)val_int)
			hough_threashhold = val_int;
		else cout << "格式错误" << endl;
		hough_mutex.unlock();
	}
	else if (param == "6" || param == "hough_minLineLength") {
		hough_mutex.lock();
		if (value != 0)hough_minLineLength = value;
		else cout << "格式错误" << endl;
		hough_mutex.unlock();
	}
	else if (param == "7" || param == "hough_mingap") {
		hough_mutex.lock();
		if (value != 0)hough_maxLineGap = value;
		else cout << "格式错误" << endl;
		hough_mutex.unlock();
	}
	else if (param == "8" || param == "h_max_width") {

		h_length_mutex.lock();
		int val_int = (int)value;
		if (value != 0 && value == (double)val_int)
			h_max_width = val_int;
		else cout << "格式错误" << endl;
		h_length_mutex.unlock();
	}
	else if (param == "9" || param == "h_unit_y_gap") {
		int val_int = (int)value;
		if (value != 0 && value == (double)val_int)
			h_unit_y_gap = val_int;
		else cout << "格式错误" << endl;
	}
	else if (param == "10" || param == "h_minLineLen_org_rate") {
		h_length_mutex.lock();
		if (value != 0)h_minLineLen_org_rate = value;
		else cout << "格式错误" << endl;
		h_length_mutex.unlock();
	}
	else if (param == "11" || param == "w_angle_deviation") {
		int val_int = (int)value;
		if (value != 0 && value == (double)val_int)
			w_angle_deviation = val_int;
		else cout << "格式错误" << endl;
	}
	else if (param == "12" || param == "w_deviation_rate") {		
		max_deviation_rate_mutex.lock();
		if (value != 0)max_w_deviation_rate = value;
		else cout << "格式错误" << endl;
		max_deviation_rate_mutex.unlock();
	}
	else if (param == "13" || param == "h_deviation_rate") {
		max_deviation_rate_mutex.lock();
		if (value != 0)max_h_deviation_rate = value;
		else cout << "格式错误" << endl;
		max_deviation_rate_mutex.unlock();
	}
	else if (param == "14" || param == "h_deviation_rate_cmp") {
		max_deviation_rate_mutex.lock();
		if (value != 0)max_h_deviation_rate_cmp = value;
		else cout << "格式错误" << endl;
		max_deviation_rate_mutex.unlock();
	}
	else if (param == "15" || param == "threashold_door_closed") {
		threashold_door_mutex.lock();
		if (value != 0)threashold_door_closed = value;
		else cout << "格式错误" << endl;
		threashold_door_mutex.unlock();
	}
	else if (param == "16" || param == "threashold_door_opened") {
		threashold_door_mutex.lock();
		if (value != 0)threashold_door_opened = value;
		else cout << "格式错误" << endl;
		threashold_door_mutex.unlock();
	}
	else if (param == "17" || param == "mark_disapeared_maxtimes") {
		int val_int = (int)value;
		if (value != 0 && value == (double)val_int)
			mark_disapeared_maxtimes = val_int;
		else cout << "格式错误" << endl;		
	}
	else if (param == "18" || param == "detectMark_timeout") {
		int val_int = (int)value;
		if (value != 0 && value == (double)val_int)
			detectMark_timeout = val_int;
		else cout << "格式错误" << endl;
	}
	else if (param == "19" || param == "min_frame_interval") {
		int val_int = (int)value;
		if (value != 0 && value == (double)val_int)
			min_workframe = val_int;
		else cout << "格式错误" << endl;
	}

	else if (param == "20" || param == "threashold_rect_series_x") {
	rect_detect_parameters->this_mutex.lock();
	if (value != 0)
		rect_detect_parameters->threashold_rect_series_x = value;
	else cout << "格式错误" << endl;
	rect_detect_parameters->this_mutex.unlock();
	}
	else if (param == "21" || param == "threashold_rect_series_y") {
	rect_detect_parameters->this_mutex.lock();
	if (value != 0)
		rect_detect_parameters->threashold_rect_series_y = value;
	else cout << "格式错误" << endl;
	rect_detect_parameters->this_mutex.unlock();
	}
	else if (param == "22" || param == "threashold_rect_seriesCount") {
	rect_detect_parameters->this_mutex.lock();
	int val_int = (int)value;
	if (value != 0 && value == (double)val_int)
		rect_detect_parameters->threashold_rect_seriesCount = val_int;
	else cout << "格式错误" << endl;
	rect_detect_parameters->this_mutex.unlock();
	}
	else if (param == "23" || param == "rect_detect_series_frame_out") {
	rect_detect_parameters->this_mutex.lock();
	int val_int = (int)value;
	if (value != 0 && value == (double)val_int)
		rect_detect_parameters->series_frame_out = val_int;
	else cout << "格式错误" << endl;
	rect_detect_parameters->this_mutex.unlock();
	}
	
}

void DoorMonitor::show_param(istringstream& issm) {
	string param;
	if (issm)issm >> param;
	if (param.empty() || param == "all") {

		cout << "Door monitor's params :" << std::endl;
		cout.setf(ios::right);
		hough_mutex.lock();
		cout << "<1>是否进行高斯模糊                     [ true/false ]  (do_blur)              " 
			<< setw(10) << (do_blur?"true":"false") << endl;
		cout << "<2>高斯模糊方差                         [ 建议0.8 - 2 ] (blur_sigma)           " 
			<< setw(10) << gaussianblur_sigma << endl;
		cout << "<3>高斯算子大小                         [ 3或5,建议3 ]  (blur_ksize)           " 
			<< setw(10) << to_string(gaussianblur_ksize.width) + " x "
			+ to_string(gaussianblur_ksize.height) << endl;
		cout << "<4>canny边缘检测阈值范围                [ 标准 20 80 ]  (canny_threashhold)    " 
			<< setw(7) << canny_threashhold_min<<" "<<
			canny_threashhold_max<< endl;
		//cout << "   不可更改           (canny_aperturesize)"
		//	<< setw(20) << canny_aperturesize << endl;
		cout << "<5>霍夫直线检测阈值                     [ 标准 10 ]     (hough_threashhold)    "
			<< setw(10) << hough_threashhold << endl;
		cout << "<6>霍夫直线检测最短直线长度             [ 标准 2 ]      (hough_minlen_rate)    "
			<< setw(10) << hough_minLineLength << endl; 
		cout << "<7>霍夫直线检测允许缺口数               [ 标准 4 ]      (hough_mingap)         "
			<< setw(10) << hough_maxLineGap << endl;
		hough_mutex.unlock();

		h_length_mutex.lock();
		cout << "<8>标记高边最大两端点行间距             [ 标准 2 ]      (h_max_width)          "
			<< setw(10) << h_max_width  << endl;
		cout << "<9>组合垂直直线时的最大缺口             [ 标准 0.2 ]    (h_unit_y_gap)         "
			<< setw(10) << h_unit_y_gap << endl;
		cout << "<10>原标记高边最短长度比例(基于范围高)  [ 标准 0.2 ]    (h_minLineLen_org_rate)"
			<< setw(10) << h_minLineLen_org_rate << endl;
		h_length_mutex.unlock();

		cout << "<11>宽边识别的最大角度偏差              [ 标准 40 ]     (w_angle_deviation)    "
			<< setw(10) << w_angle_deviation << "度" << endl;

		max_deviation_rate_mutex.lock();
		cout << "<12>宽边的最大长度偏差                  [ 标准 0.4 ]    (w_deviation_rate      "
			<< setw(10) << max_w_deviation_rate << endl;
		cout << "<13>高边的最大长度偏差                  [ 标准 0.4 ]    (h_deviation_rate)     "
			<< setw(10) << max_h_deviation_rate << endl;
		cout << "<14>比较原始位置与当前位置高边的最大长度偏差 [标准 0.4] (h_deviation_rate_cmp) "
			<< setw(10) << max_h_deviation_rate_cmp << endl;
		max_deviation_rate_mutex.unlock();

		threashold_door_mutex.lock();
		cout << "<15>门打开时判定再次关闭的最大位移（基于标记宽度和原始位置）[标准 0.05] (threashold_door_closed) "
			<< setw(10) << threashold_door_closed << endl;
		cout << "<16>门关闭时判定已打开的最小位移  （基于标记宽度和原始位置）[标准  0.2] (threashold_door_opened) "
			<< setw(10) << threashold_door_opened << endl;
		threashold_door_mutex.unlock();

		cout << "<17>判定标记消失所需检测失败次数            [ 标准 20 ]  (mark_disapeared_maxtimes)     "
			<< setw(10) << mark_disapeared_maxtimes<< endl;

		cout << "<18>标记位置初始化的超时时间                (detectMark_timeout)                        "
			<< setw(10) << detectMark_timeout <<"ms"<< endl;
		cout << "<19>最小识别帧间隔(工作频率)                 [标准 1]   (min_frame_interval)            "
			<< setw(10) << min_workframe << "ms" << endl;
		
		rect_detect_parameters->this_mutex.lock_shared();
		cout << "<20>一帧最大水平位移和宽度变化比例(基于宽)   [标准 0.25]   (threashold_rect_series_x)   "
			<< setw(10) << rect_detect_parameters->threashold_rect_series_x << endl;
		cout << "<21>一帧最大垂直位移和高度变化比例(基于高)   [标准 0.15]   (threashold_rect_series_y)   "
			<< setw(10) << rect_detect_parameters->threashold_rect_series_y << endl;
		cout << "<22>最小连续帧数，高于此则视为标记           [标准 3]      (threashold_rect_seriesCount)"
			<< setw(10) << rect_detect_parameters->threashold_rect_seriesCount << endl;
		cout << "<23>识别连续矩形帧超时                       [标准 10]     (rect_detect_series_frame_out)"
			<< setw(10) << rect_detect_parameters->series_frame_out << endl;
		rect_detect_parameters->this_mutex.unlock_shared();

	}
	else if (param == "help" || param == "-h") {

	}
	else {
		cout << "暂不支持个别参数查看方式" << endl;
	}
}
void DoorMonitor::load_params()
{
}

void DoorMonitor::save_params()
{
}
