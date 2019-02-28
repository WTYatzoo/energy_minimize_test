#ifndef _VERTEX_
#define _VERTEX_

#include <math.h>
#include "myvector.h"
class vertex
{
public:
	myvector location;
	myvector normal;
	int index_HE_towards; //指向的一条halfedge的索引
	double area_mixed; //one ring 后的混合型有限面积域

        int isBoundaryPoint; //是不是边界点（这里的边界点指的是包含了真正的边界点和被固定的点）
        int rankOfInside; //对于不是边界点的点　在内部的新序数
        int rankOfOutside; //对于是边界点的点　在外部的新序数

        int FixedOrRealBoundary; //区分是被固定的点还是真正的边界上的点

	vertex();
	vertex(myvector location);
	~vertex();
};

#endif
