/*
Edition: 1.0.
Editor: Yang Heng, Ma Ke.
OpenCV Edition: 2.3.1.
Ide Edition: Visual Studio 2010.
Time: 2016.4.11.
*/
#define MARGIN 10
#define SQUARE 1
#define CROSS  2

enum
{
	SEARCH = 1,
	TRACK = 2
};

/*�ṹ�嶨��*/
typedef struct __PointDef   /* Ŀ��״̬���� */
{
	int x;
	int y;
}POINTDEF;

typedef struct __RectDef
{
	int x;					/* ��Сx����λ�� */
	int y;					/* ��Сy����λ�� */
	int width;				/* Ŀ��ȫ��� */
	int height;				/* Ŀ��ȫ�߶� */
}RECTDEF;

typedef struct __ObjectState  /* Ŀ��״̬���� */
{  
	RECTDEF rect;
	POINTDEF centroid;        /* �������� */
	int area;				  /* Ŀ����� */
	int catrgory;			  /* Ŀ������ 0:��Ŀ��; 1:��������; 2:�ɻ�; 3:����; 4:ֱ����; 5:����ĸ��; 6:̹�� */
	void* user_data;		  /* �û����ݣ����� */
} OBJECTSTATE; 
