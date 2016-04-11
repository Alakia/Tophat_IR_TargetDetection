/*
Edition: 1.0.
Editor: Yang Heng, Ma Ke.
OpenCV Edition: 2.3.1.
Ide Edition: Visual Studio 2010.
Time: 2016.4.11.
*/
#include "define.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
using namespace std;

int AdptiveThresh(Mat image, float diffx, float diffy);
void OptimizeForeground(unsigned char * s_img, int img_width, int img_height);
void SeedFill(Mat image, bool* flags, int pos, RECTDEF* rect);
void ObjSegmentation(Mat image, OBJECTSTATE* p_obj_list, int* p_obj_num);
void MorphErosion(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape);
void MorphDilition(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape);
void MorphOpen(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape);
void MorphClose(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape);
void MorphTopHat(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape);
void MorphButHat(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape);
void ImgThread(Mat src, Mat dst, int height, int width, int tre);


int main(int argc, char * argv[])
{
	char filePath[100];
	Rect box; // [x y width height] tracking position

	Mat frame;
	Mat gray_img, temp_img;
	Mat temp_img3, temp_img5, temp_img7, temp_img9;
	int i,k, morph_rad = 4;
	Mat element;
	OBJECTSTATE obj_list[100];
	int obj_num = 0;
	int obj_id = 0, area, max_area = 0;
	int work_mode = SEARCH;
	int work_mode_count = 0;
	RECTDEF search_rect, track_rect; // 搜索和跟踪区域
	// 检测和跟踪是否有效判断
	bool search_valid = false, track_valid = false; 
	Mat model_img;
	Rect model_rect;
	RECTDEF result_rect;

	element  = getStructuringElement(0, Size(2*morph_rad+1, 2*morph_rad+1), Point(morph_rad,morph_rad));
	for(i = 1; i < 300; i++)
	{
		sprintf(filePath, "E:/img/imagesequences/ir_img_sky_1/%04d.jpeg", i);
		frame = imread(filePath);


		if(frame.data == NULL)
		{
			printf("\nCannot open the image!");
			return 0;
		}
		cvtColor(frame, gray_img, CV_RGB2GRAY);
		temp_img = gray_img.clone();temp_img3 = gray_img.clone();temp_img5 = gray_img.clone();temp_img7 = gray_img.clone();temp_img9 = gray_img.clone();

		if(work_mode == SEARCH)
		{
			// 按照指标要求，视场为70*40%
			search_rect.x = frame.cols*0.15;

			search_rect.y = frame.rows*0.3;
			search_rect.width = 0.7*frame.cols;
			search_rect.height = 0.4*frame.rows;
			printf("\n进入检测模式");
			// 1. 形态学滤波
			double t1 = (double)getTickCount();
			//morphologyEx(gray_img, temp_img, CV_MOP_ERODE, element);
			int shape = SQUARE;
			float diffx = 0.3, diffy = 0.15;
			MorphTopHat(temp_img, temp_img, gray_img.rows, gray_img.cols, diffx, diffy, 9, 9, shape);
			
		    double t2 = ((double)getTickCount() - t1) / getTickFrequency();
			printf("\nMorph time is %f\n", t2);
			imshow("Morph", temp_img);

			// 2. 自适应阈值分割
			int thresh = AdptiveThresh(temp_img, diffx, diffy);
			threshold(temp_img, temp_img, thresh, 255, 0);
			imshow("tre", temp_img);
			printf("\nthreshold = %d", thresh);

			// 3. 填补空洞
			OptimizeForeground(temp_img.data, temp_img.cols, temp_img.rows);
			imshow("Opt", temp_img);

			// 4. 种子填充，获取目标波门
			ObjSegmentation(temp_img, obj_list, &obj_num);
			imshow("Obj", temp_img);

			//5. 筛选结果
			max_area = 0; obj_id = 0;
			RECTDEF r;
			for(k = 0; k < obj_num; k++)
			{
				r = obj_list[k].rect;
				// 判断出界
				if(r.x < search_rect.x || r.x+r.width > search_rect.x+search_rect.width 
					|| r.y < search_rect.y || r.y+r.height > search_rect.y+search_rect.height)
					continue;
				area = r.width*r.height;
				if( area > 1000)
					continue;
				if(area > max_area)
				{
					max_area = area;
					obj_id = k;
				}
			}
			if(max_area > 0)
			{
				work_mode_count++;
				search_valid = true;
			}
			else // 必须是连续发现目标，否则统计计数置0
			{
				work_mode_count = 0;
				search_valid = false;
			}
		}

		// 6. 显示结果
		if(work_mode == SEARCH)
		{
			rectangle(frame, Point(search_rect.x,search_rect.y), 
				Point(search_rect.x+search_rect.width, search_rect.y+search_rect.height),Scalar(255,255,255));
			if(search_valid)
				result_rect = obj_list[obj_id].rect;
		}
		if(work_mode == TRACK)
		{
			rectangle(frame, Point(track_rect.x,track_rect.y), 
				Point(track_rect.x+track_rect.width, track_rect.y+track_rect.height),Scalar(255,255,255));
			if(track_valid)
			{
				result_rect.x = model_rect.x;
				result_rect.y = model_rect.y;
				result_rect.width = model_rect.width;
				result_rect.height = model_rect.height;
			}
		}
		//for(k = 0; k < obj_num; k++)
		{
			rectangle(frame, Point(result_rect.x,result_rect.y), 
				Point(result_rect.x+result_rect.width, result_rect.y+result_rect.height),
				Scalar(0,0,255), 3);
		}

		imshow("result", frame);// Display
		waitKey(1);	
	}

	return 0;
}
