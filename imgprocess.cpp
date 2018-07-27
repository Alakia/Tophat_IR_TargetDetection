#include "define.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace cv;
using namespace std;

extern int AdptiveThresh(Mat image, float diffx, float diffy)
{
	int w = image.cols;
	int h = image.rows;
	int thresh = 166;

	unsigned char*np;
	unsigned char pixel;
	int thresholdValue=1;
	int ihist[33];

	int i, j, k; 
	int n, n1, n2, gmin, gmax;
	double m1, m2, sum, csum, fmax, sb;

	
	memset(ihist, 0, sizeof(ihist));

	gmin=255; gmax=0;

	for (i =h*diffx; i < h-h*diffx; i++) 
	{
		np = (unsigned char*)(image.data + w*i);
		for (j =w*diffy; j < w-w*diffy; j++) 
		{
			pixel = np[j];
			ihist[ pixel/8 ]++;
			if(pixel > gmax) gmax= pixel;
			if(pixel < gmin) gmin= pixel;
		}
	}
	ihist[32] = 0;
	thresh = 160;
	for(i = 31; i >=0; i--)
	{
		ihist[i] = ihist[i] + ihist[i+1];
		if(ihist[i] > 20)//change from 20
		{
			thresh = (i-1)*8;
			break;
		}
	}
	return thresh;
}

extern void OptimizeForeground(unsigned char * s_img, int img_width, int img_height)
{
	const short N = 4;   
	const short N2 = 81; 
	const short TC = 8; 
	int i, j, idx_i, idx_s, cur, s_uv, avg_d, idx_lu, idx_ru, idx_lb, idx_rb;
	short s[1500]; 
	unsigned short* i_img;

	i_img = (unsigned short*)malloc(sizeof(unsigned short)*img_width*img_height);
	memset(i_img, 0, sizeof(unsigned short)*img_width*img_height);
	
	// 计算积分图
	for(i = 0; i < img_height; i++)
	{
		for(j = 0; j < img_width; j++)
		{
			idx_i = i * img_width + j;
			cur = *(s_img+idx_i) >= 255 ? 1 : 0;
			if(i == 0)
				s[j] = 0;
			s_uv = s[j] + cur;
			s[j] = s_uv;
			if(j == 0)
				*(i_img+idx_i) = s_uv;
			else
				*(i_img+idx_i) = *(i_img+idx_i-1) + s_uv;
		}
	}

	// 重新提取前景点
	for(i = N; i < img_height-N; i++)
	{
		for(j = N; j < img_width-N; j++)
		{
			idx_i = i * img_width + j;
			idx_lu = (i-N)*img_width + j - N;
			idx_ru = idx_lu + (N<<1);
			idx_lb = (i+N)*img_width + j - N;
			idx_rb = idx_lb + (N<<1);
			avg_d = (*(i_img+idx_lu)+*(i_img+idx_rb)-*(i_img+idx_lb)-*(i_img+idx_ru));
			*(s_img+idx_i) = avg_d > TC ? 255 : 0;
		}
	}

	free(i_img);
}

extern void SeedFill(Mat image, bool* flags, int pos, RECTDEF* rect)
{
	const int max_pts  = 15000;
	int x, y, idx, i, j;
	int img_width, img_height;
	POINTDEF list[max_pts];
	int point_num = 0;
	POINTDEF point, temp_point;
	unsigned char data;

	img_width = image.cols;
	img_height = image.rows;
	x = pos % img_width;
	y = pos / img_width;
	temp_point.x = x;
	temp_point.y = y;
	
	list[point_num] = temp_point;
	point_num = (point_num+1) % max_pts;

	// 初始化外接矩形参数
	rect->x = x;
	rect->y = y;
	rect->width = 1;
	rect->height =1;

	while(point_num)
	{
		point = list[point_num-1];
		point_num--;
		for(i=-2; i<=2; i++)
		{
			for(j=-2; j<=2; j++)
			{
				idx = (point.y+i)*img_width + (point.x+j);
				if(flags[idx] || point.y+i < MARGIN || point.y+i > img_height-MARGIN 
					|| point.x+j < MARGIN || point.x+j > img_width-MARGIN)
					continue;
				flags[idx] = true;
				data = (unsigned char)image.data[idx];
				if(data == 255 ) 
				{
					temp_point.x = point.x+j;
					temp_point.y = point.y+i;
					list[point_num] = temp_point;
					point_num = (point_num+1) % max_pts;
		
					if(rect->x > point.x+j)
					{
						rect->width += (rect->x - point.x-j);
						rect->x = point.x+j;
					}
					else if(point.x+j > rect->x + rect->width)
					{
						rect->width = (point.x+j - rect->x); 
					}
					if(rect->y > point.y+i)
					{
						rect->height += (rect->y - point.y - i);
						rect->y = point.y+i;
					}
					else if(point.y+i > rect->y + rect->height)
					{
						rect->height = (point.y+i - rect->y); 
					}
				}
			}// end for
		} //end for
	}// end while 再找不到连通区域
}

extern void ObjSegmentation(Mat image, OBJECTSTATE* p_obj_list, int* p_obj_num)
{
	long int i;
	int j,k, idx, num;
	int img_width, img_height,row,col;
	bool* flags_arr;
	RECTDEF rect; // 种子填充算法的外接矩形
	OBJECTSTATE obj_ste;
	unsigned char data;

	img_width = image.cols;
	img_height = image.rows;

	flags_arr = (bool*)malloc(sizeof(bool)*img_width*img_height);
	memset(flags_arr, 0, sizeof(bool)*img_width*img_height);
	
	num = 0;
	// 遍历整个图像
	for(i = 0; i<img_width*img_height; i++)
	{
		row = i / img_width;
		col = i % img_width;

		if(flags_arr[i] || row < MARGIN || row > img_height-MARGIN || col < MARGIN || col > img_width-MARGIN)
			continue;

		data = (unsigned char)image.data[i];
		if(data >= 255) //区域搜索
		{
			flags_arr[i] = true;

			SeedFill(image, flags_arr, i, &rect);

		
			if(rect.width < 2 || rect.height<2 || rect.width > 0.7*img_width || rect.height > 0.7*img_width )
			{
				for(j = 0; j < rect.height; j++)
				{
					for(k = 0; k < rect.width; k++)
					{
						idx = (rect.y+j) * img_width + (rect.x+k);
						flags_arr[idx] = true;
					}
				}
				continue;
			}
			rect.x -= 8; 
			rect.y -= 8;
			rect.width += 16;
			rect.height += 16;
			obj_ste.rect = rect;
			p_obj_list[num++] = obj_ste;

			for(j = 0; j < rect.height; j++)
			{
				for(k = 0; k < rect.width; k++)
				{
					idx = (rect.y+j) * img_width + (rect.x+k);
					flags_arr[idx] = true;
				}
			}
		}// end 区域搜索
		else
		{
			flags_arr[i] = true;
		}
	}// end 图像遍历
	free(flags_arr);

	*p_obj_num = num;
}

/*Morphology*/
extern void MorphErosion(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape)//xsize, ysize should be even
{
	if((width - ysize < 0) && (height - xsize < 0))
		return;
	if(ysize != xsize)
		return;
	int mid = (int)(xsize/2); int val = 0; int xdif = diffx * height; int ydif = diffy * width;
	for(int i = mid + xdif; i < height - mid - xdif; i++){
		for(int j = mid + ydif; j < width - mid - ydif; j++){
			int min = 255;
			if(shape == SQUARE){
				for(int m = -mid; m < mid; m++){
					for(int n = -mid; n < mid; n++){
						val = src.data[(i+m)*width + j + n];
						if(val < min)
							min = val;
					}
				}
			}
			else if(shape == CROSS){
				for(int m = -mid; m < mid; m++){
					for(int n = -mid; n < mid; n++){
						if(m == 0 || n == 0){
							val = src.data[(i+m)*width + j + n];
							if(val < min)
								min = val;
						}
						else
							continue;
					}
				}
			}
			dst.data[i * width + j] = min;
		}
	}
}

extern void MorphDilition(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape)
{
	if((width - ysize < 0) && (height - xsize < 0))
			return;
		if(ysize != xsize)
			return;
		int mid = (int)(xsize/2); int val = 0; int xdif = diffx * height; int ydif = diffy * width;
		for(int i = mid + xdif; i < height - mid - xdif; i++){
			for(int j = mid + ydif; j < width - mid - ydif; j++){
				int max = 0;
				if(shape == SQUARE){
					for(int m = -mid; m < mid; m++)	{
						for(int n = -mid; n < mid; n++){
							val = src.data[(i+m)*width + j + n];
								if(val > max)
									max = val;
						}
					}
				}
				else if(shape == CROSS){
				for(int m = -mid; m < mid; m++){
					for(int n = -mid; n < mid; n++){
						if(m == 0 || n == 0){
							val = src.data[(i+m)*width + j + n];
							if(val > max)
								max = val;
						}
						else
							continue;
					}
				}
			}
				dst.data[i * width + j] = max;
			}
		}
}

extern void MorphOpen(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape)
{
	Mat tmp = src.clone();
	MorphErosion(src, tmp, height, width, diffx, diffy, xsize, ysize, shape);
	MorphDilition(tmp, dst, height, width, diffx, diffy, xsize, ysize, shape);
}

extern void MorphClose(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape)
{
	Mat tmp = src.clone();
	MorphDilition(src, tmp, height, width, diffx, diffy, xsize, ysize, shape);
	MorphErosion(tmp, dst, height, width, diffx, diffy, xsize, ysize, shape);
}

extern void MorphTopHat(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape)
{
	Mat tmp = dst.clone();
	MorphOpen(src, tmp, height, width, diffx, diffy, xsize, ysize, shape);
	for(int i = 0; i < height * width; i++){
			int idx = i / width;
			int idy = i % width;
			if(idx < xsize || idx > height - xsize || idy < ysize || idy > width - ysize)
				dst.data[i] = 0;
			else
				dst.data[i] = abs(src.data[i] - tmp.data[i]);
	}
}

extern void MorphButHat(Mat src, Mat dst, int height, int width, float diffx, float diffy, int xsize, int ysize, int shape)
{
	Mat tmp = dst.clone();
	MorphClose(src, tmp, height, width, diffx, diffy, xsize, ysize, shape);
	for(int i = 0; i < height * width; i++){
		int row = i / width;
		int col = i % width;
		if(row < xsize || row > height - xsize || col < ysize || col > width - ysize)
			continue;
		else
			dst.data[i] = abs(tmp.data[i] - src.data[i]);
	}
}

extern void ImgThread(Mat src, Mat dst, int height, int width, int tre)
{
	for(int i = 0; i < src.cols * src.rows; i++)
	{
		if(src.data[i] >= tre)
			src.data[i] = 255;
		else
			dst.data[i] = 0;
	}
}

