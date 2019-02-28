#ifndef _OBJECT_
#define _OBJECT_

#include <vector>
#include <map>
#include "halfedge.h"
#include "vertex.h"
#include "face.h"
#include "testface.h"
using namespace std;

class object
{
public:
	vector<vertex > myvertexs;
	vector<face > myfaces;
	vector<halfedge > myhalfedges;
	vector<testface > mytestfaces;

	int num_vertex;
	int num_face;
	int num_halfedge;
        int num_vertex_inside;
        int num_vertex_outside;
        int which;

        map<int ,int > mpFromRToC;
        map<int ,int > mpFromCToR;

	object();
	~object();
	int calData(int index[3],double data[3][3],double &area,myvector &normal);
	void getObjData();
	void calLocForEnergyOptimal();

	void testdraw();
	
};
#endif
