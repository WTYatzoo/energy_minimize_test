#ifndef _HALFEDGE_
#define _HALFEDGE_

#include <math.h>
#include "myvector.h"

class halfedge
{
public:

	int index_vertex_towards;
	int index_vertex_begin;
	int index_face;
	int index_nextHE;
	int index_prevHE;
	int index_oppHE;
	double angle_towards; //在所在三角形中相对的角的角度
	double angle_accompany; //在所在三角形中左手边的角度(因为是逆时针排布)
	double area_accompany;  //在所在三角形中左手边的有限面积域
	double length; //边长
	myvector itself; //向量

	
	~halfedge(){}

	halfedge(int index_vertex_towards,int index_vertex_begin,int index_face,int index_nextHE,int index_prevHE,int index_oppHE,double angle_towards,
		double angle_accompany,double area_accompany,double length,myvector itself):index_vertex_towards(index_vertex_towards),index_vertex_begin(index_vertex_begin),index_face(index_face),index_nextHE(index_nextHE),index_prevHE(index_prevHE),
			index_oppHE(index_oppHE),angle_towards(angle_towards),angle_accompany(angle_accompany),area_accompany(area_accompany),length(length),itself(itself){} //对于一条边的第二条halfedge，因为第一条halfedge已经得到，所以反向边初始化即可得到
	
	
    halfedge(){}

	halfedge(int index_vertex_towards,int index_vertex_begin,int index_face,int index_nextHE,int index_prevHE,double angle_towards,double angle_accompany,
		double area_accompany,double length,myvector itself):index_vertex_towards(index_vertex_towards),index_vertex_begin(index_vertex_begin),index_face(index_face),index_nextHE(index_nextHE),index_prevHE(index_prevHE)
		,angle_towards(angle_towards),angle_accompany(angle_accompany),area_accompany(area_accompany),length(length),itself(itself){} //对于一条边的第一条halfedge，无法预知第二条halfedge索引，故初始化时不管他

};
#endif