#define MARGIN 10
#define SQUARE 1
#define CROSS  2

enum
{
	SEARCH = 1,
	TRACK = 2
};

/*结构体定义*/
typedef struct __PointDef   /* 目标状态变量 */
{
	int x;
	int y;
}POINTDEF;

typedef struct __RectDef
{
	int x;					/* 最小x坐标位置 */
	int y;					/* 最小y坐标位置 */
	int width;				/* 目标全宽度 */
	int height;				/* 目标全高度 */
}RECTDEF;

typedef struct __ObjectState  /* 目标状态变量 */
{  
	RECTDEF rect;
	POINTDEF centroid;        /* 质心坐标 */
	int area;				  /* 目标面积 */
	int catrgory;			  
	void* user_data;		  /* 用户数据，保留 */
} OBJECTSTATE; 
