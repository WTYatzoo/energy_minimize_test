#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <map>
#include <GL/glut.h>
#include <vector>

#include </usr/include/eigen3/Eigen/Eigen>
#include </usr/include/eigen3/Eigen/Dense>
#include </usr/include/eigen3/Eigen/Cholesky>
#include </usr/include/eigen3/Eigen/LU>
#include </usr/include/eigen3/Eigen/Sparse>
#include </usr/include/eigen3/Eigen/SparseQR>
#include </usr/include/eigen3/Eigen/SparseLU>
#include </usr/include/eigen3/Eigen/IterativeLinearSolvers>

#include "object.h"
#include "physic.h"

using namespace std;
using namespace Eigen;

#define pi 3.1415926535898
#define min(a,b) (((a) < (b)) ? (a) : (b))
object::object()
{
  getObjData();
  which=1;
   // calLocForEnergyOptimal();
}

object::~object()
{
	
}

void  object::testdraw()
{
  int i;
  int siz=mytestfaces.size();
  testface now;
  glColor3d(0,0,1);
  for(i=0;i<siz;i++)
    {
      now=mytestfaces[i];
      glBegin(GL_POLYGON);
      {
	glVertex3d(myvertexs[now.index_vertex[0]].location.x,myvertexs[now.index_vertex[0]].location.y,myvertexs[now.index_vertex[0]].location.z);
	glVertex3d(myvertexs[now.index_vertex[1]].location.x,myvertexs[now.index_vertex[1]].location.y,myvertexs[now.index_vertex[1]].location.z);
	glVertex3d(myvertexs[now.index_vertex[2]].location.x,myvertexs[now.index_vertex[2]].location.y,myvertexs[now.index_vertex[2]].location.z);
      }
      glEnd();
    }
}

int object::calData(int index[3],double data[3][3],double &area,myvector &normal) //角度 0，需要的面积 1，边长 2
{
  int i;
  myvector here[3];
  int one,two;
  for(i=0;i<3;i++)
    {
      one=index[i%3]; two=index[(i+1)%3];
      here[i]=myvector(myvertexs[two].location-myvertexs[one].location);
      data[i][2]=here[i].len();
    }

  normal=here[0].cross(here[1]);
  normal.normalize();

  double cosData[3];
  int returnValue;
  for(i=0;i<3;i++)
    {
      cosData[i]=here[i].dot(here[(i+2)%3]*-1)/here[i].len()/here[(i+2)%3].len();
      data[i][0]=acos(cosData[i]); //弧度制
    }
  if(cosData[0]>=0&&cosData[1]>=0&&cosData[2]>=0)
    {
      returnValue=1;
    }
  else
    {
      returnValue=0;
    }
	
  if(returnValue==1)
    {
      for(i=0;i<3;i++)
	{
	  data[i][1]=(here[i].len_sq()/tan(data[(i+2)%3][0])+here[(i+2)%3].len_sq()/tan(data[(i+4)%3][0]))/8.0;
	}
    }
  else
    {
      ;
    }
  //海伦秦九韶公式
  double p=(data[0][2]+data[1][2]+data[2][2])/2.0;
  area=sqrtf(p*(p-data[0][2])*(p-data[1][2])*(p-data[2][2]));
	
  return returnValue;
}

void object::getObjData()
{

  string name="/home/wtyatzoo/project/model/torus_p2.txt";
  FILE* file=fopen(name.c_str(),"r");
  fscanf(file,"%d%d",&num_vertex,&num_face);
  printf("%d %d \n",num_vertex,num_face);
	
  int i,j;
  double x,y,z;
  vertex here;
  char  filter[5];
  for(i=0;i<num_vertex;i++)
    {
      fscanf(file,"%s%lf%lf%lf",filter,&x,&y,&z);
      myvertexs.push_back(vertex(myvector(x,y,z)));
    }
	
  printf("%lf %lf %lf \n",x,y,z);
	
  map<pair<int,int >,int > mp1,mp2,mp3;
  map<pair<int,int >,int > ::iterator it;
  int index[3];
  int one,two,help;
  int mark;
  int is;
	
  double data[3][3]; //每一个行向量保存计算一条有向边在一个逆时针围绕的三角形中左手边的角度，需要的面积 ，边长 
  double area; //当前处理的三角形的面积
  myvector normal;

  for(i=0;i<num_face;i++)
    {
      fscanf(file,"%s%d%d%d",filter,&index[0],&index[1],&index[2]);
      for(j=0;j<3;j++)
	{
	  index[j]--;
	}
		
      is=calData(index,data,area,normal);

      int siz_begin=myhalfedges.size(); //这轮循环开始的size

      myfaces.push_back(face(siz_begin,area,is,normal));
      mytestfaces.push_back(testface(index[0],index[1],index[2]));
		
      myvector itself; //保存当前halfedge的向量形式

		
      for(j=0;j<3;j++)
	{
	  one=index[j%3]; two=index[(j+1)%3]; //当前处理的halfedge的起点和终点
	  itself=myvertexs[two].location-myvertexs[one].location;
	  if(myvertexs[one].index_HE_towards==-1)
	    {
	      myvertexs[one].index_HE_towards=siz_begin+j;
	    }
	  else
	    {
	      ;
	    }
			
	  if(one<two)
	    {
	      mark=1;
	    }
	  else
	    {
	      help=one; one=two; two=help;
	      mark=-1;
	    }
	  int siz=myhalfedges.size(); //当前halfedge数目，就是当前处理的halfedge的索引
	  if(mp1.find(make_pair(one,two))==mp1.end())
	    {	
	      if(mark==1)
		{
		  mp1[make_pair(one,two)]=1;
		  mp2[make_pair(one,two)]=siz; //保存这条halfedge的索引号，待其反向的halfedge出现时建立联系
		  myhalfedges.push_back(halfedge(two,one,i,siz_begin+(j+1)%3,siz_begin+((j-1)%3+3)%3,data[(j+2)%3][0],data[j][0],data[j][1],data[j][2],itself));
                    
		}
	      else if(mark==-1)
		{
		  mp1[make_pair(one,two)]=-1;  
		  mp2[make_pair(one,two)]=siz; //保存这条halfedge的索引号，待其反向的halfedge出现时建立联系
		  myhalfedges.push_back(halfedge(one,two,i,siz_begin+(j+1)%3,siz_begin+((j-1)%3+3)%3,data[(j+2)%3][0],data[j][0],data[j][1],data[j][2],itself));
		}
				
	    }
	  else if(mp1[make_pair(one,two)]==1||mp1[make_pair(one,two)]==-1)
	    {
	      int key=mp1[make_pair(one,two)];
	      if(key==1||key==-1)
		{
		  int oppHE=mp2[make_pair(one,two)];
		  myhalfedges[oppHE].index_oppHE=siz;
					
		  if(mark==1)
		    {
		      myhalfedges.push_back(halfedge(two,one,i,siz_begin+(j+1)%3,siz_begin+((j-1)%3+3)%3,oppHE,data[(j+2)%3][0],data[j][0],data[j][1],data[j][2],itself));
		      mp1[make_pair(one,two)]=2;
		    }
		  else if(mark==-1)
		    {
		      myhalfedges.push_back(halfedge(one,two,i,siz_begin+(j+1)%3,siz_begin+((j-1)%3+3)%3,oppHE,data[(j+2)%3][0],data[j][0],data[j][1],data[j][2],itself));
		      mp1[make_pair(one,two)]=2;  
		    }
		}
	    }
	}
		
    }
  int x1,y1;
	
  for(it=mp1.begin();it!=mp1.end();it++)
    {
      if(it->second==1||it->second==-1)
	{
	  //it->first 是一个pair　因此是一个实体对象，不是指针
	  x1=it->first.first;
	  y1=it->first.second;
	  myvertexs[x1].isBoundaryPoint=1;
	  myvertexs[x1].FixedOrRealBoundary=1; // 1表示是真的边界上的点而２表示是固定的内部点
	  myvertexs[y1].isBoundaryPoint=1;
	  myvertexs[y1].FixedOrRealBoundary=1;
	}
    }

  int loop=4;
  int loop_now=0;
  for(loop_now=0;loop_now<loop;loop_now++)
    {
      for(i=0;i<num_face;i++)
	{
	  testface now=mytestfaces[i];
	  int mark=0;
	  for(j=0;j<3;j++)
	    {
	      int here=mytestfaces[i].index_vertex[j];
	      if(myvertexs[here].isBoundaryPoint==1)
		{
		  mark=1;
		  break;
		}
	    }
	  if(mark==1)
	    {
	      for(j=0;j<3;j++)
		{
		  int here=mytestfaces[i].index_vertex[j];
		  if(myvertexs[here].isBoundaryPoint!=1)
		    {
		      myvertexs[here].isBoundaryPoint=-1;
		    }
		}
	    }
	}
      for(i=0;i<num_vertex;i++)
	{
	  if(myvertexs[i].isBoundaryPoint==-1)
	    {
	      myvertexs[i].isBoundaryPoint=1;
	      myvertexs[i].FixedOrRealBoundary=2;
	    }
	}
    }

  
  
  int rank_now=0;
  for(i=0;i<num_vertex;i++)
    {
      if(myvertexs[i].isBoundaryPoint==0)
	{
	  myvertexs[i].rankOfInside=rank_now;
	  rank_now++;
	}
    }
  num_vertex_inside=rank_now;
  
  rank_now=0;
  for(i=0;i<num_vertex;i++)
    {
      if(myvertexs[i].isBoundaryPoint==1)
	{
	  myvertexs[i].rankOfOutside=rank_now+num_vertex_inside;
	  rank_now++;
	}
    }
  num_vertex_outside=rank_now;
  
  printf("inside %d + outside %d == all %d\n",num_vertex_inside,num_vertex_outside,num_vertex);

  mpFromRToC.clear();
  mpFromCToR.clear();

  int c;
  for(i=0;i<num_vertex;i++)
    {
      if(myvertexs[i].isBoundaryPoint==1)
	{
	  c=myvertexs[i].rankOfOutside;
	  mpFromRToC[i]=c;
	  mpFromCToR[c]=i;
	}
      else
	{
	  c=myvertexs[i].rankOfInside;
	  mpFromRToC[i]=c;
          mpFromCToR[c]=i;
	}
    }
	
}

void object::calLocForEnergyOptimal()
{
  int i,j;
  vertex vertex_here;
  int index_HE_begin; // one ring开始时的halfedge索引
  int index_now,index_now_opp; //当前halfedge的索引以及其反向halfedge的索引
	
  int index_face_now;
  double area_mixed; //mixed面积
        
  double area; //无校正
  double angle_sum; //one ring 角度和

  double sum;
  double cot;


  SparseLU<SparseMatrix<double>,COLAMDOrdering<int>> linearSolver; //使用LU分解不要用QR分解

  MatrixXd Lc = MatrixXd::Random(num_vertex,num_vertex);
  MatrixXd AInverse =MatrixXd::Random(num_vertex,num_vertex);

  MatrixXd LAI1=MatrixXd::Random(num_vertex,num_vertex); //本身
  MatrixXd LAI2=MatrixXd::Random(num_vertex,num_vertex);// 平方
  MatrixXd LAI3=MatrixXd::Random(num_vertex,num_vertex);//三次方

  Lc.fill(0.0);
  AInverse.fill(0.0);
  
  SparseMatrix < double > LcForAll(num_vertex,num_vertex);

  vector< Triplet< double > > tripletsForLcForAll;

  VectorXd x0(num_vertex);
  VectorXd y0(num_vertex);
  VectorXd z0(num_vertex);

  VectorXd x(num_vertex);
  VectorXd y(num_vertex);
  VectorXd z(num_vertex);
  
  for(i=0;i<num_vertex;i++)
    {
      vertex_here=myvertexs[i];
      if(vertex_here.FixedOrRealBoundary==1)
	{
	  continue;
	} 
      area_mixed=0; //清零
      area=0;
      angle_sum=0;
		
      index_HE_begin=vertex_here.index_HE_towards;
      index_now=index_HE_begin; //当前的处理的halfedge索引
      index_now_opp=myhalfedges[index_now].index_oppHE;
      index_face_now=myhalfedges[index_now].index_face;

      sum=0;

      do 
	{
	  int index_vertex_towards=myhalfedges[index_now].index_vertex_towards;

	  cot=1.0/tan(myhalfedges[index_now].angle_towards)+1.0/tan(myhalfedges[index_now_opp].angle_towards);
	  Lc(mpFromRToC[i],mpFromRToC[index_vertex_towards])=cot*0.5;
	  Lc(mpFromRToC[index_vertex_towards],mpFromRToC[i])=cot*0.5;
	      
          sum+=(-1*cot*0.5);
   
	  face face_here=myfaces[index_face_now];
	  area+=(face_here.area/3.0);
	  if(face_here.is_non_obtuse==1)
	    {
	      area_mixed+=myhalfedges[index_now].area_accompany;
	    }
	  else
	    {
	      if(cos(myhalfedges[index_now].angle_accompany)>=0) //钝角三角形  但是 该角不是钝角
		{
		  area_mixed+=face_here.area/4.0;
		}
	      else
		{
		  area_mixed+=face_here.area/2.0;
		}
	    }
			
	  index_now=myhalfedges[index_now_opp].index_nextHE;
	  index_now_opp=myhalfedges[index_now].index_oppHE;
	  index_face_now=myhalfedges[index_now].index_face;
			
	} while (index_now!=index_HE_begin);

      Lc(mpFromRToC[i],mpFromRToC[i])=sum;
      AInverse(mpFromRToC[i],mpFromRToC[i])=1.0/area_mixed;	
      myvertexs[i].area_mixed=area_mixed;
    }

  x0.fill(0);
  y0.fill(0);
  z0.fill(0);

  for(i=num_vertex-1;i>=num_vertex_inside;i--)
    {
      int r=mpFromCToR[i];
      x0(i)=myvertexs[r].location.x;
      y0(i)=myvertexs[r].location.y;
      z0(i)=myvertexs[r].location.z;
    }

  LAI1=AInverse*Lc;
  LAI2=LAI1*LAI1;
  LAI3=LAI2*LAI1;
  
  for(i=0;i<num_vertex_inside;i++)
    {
      for(j=0;j<num_vertex;j++)
	{
          switch(which)
          {
          case 1:
              if(abs(LAI1(i,j))>=0.00000000000001)
                {
                  tripletsForLcForAll.emplace_back(i,j,LAI1(i,j));
                }
              break;
          case 2:
              if(abs(LAI2(i,j))>=0.00000000000001)
                {
                  tripletsForLcForAll.emplace_back(i,j,LAI2(i,j));
                }
              break;
          case 3:
              if(abs(LAI3(i,j))>=0.00000000000001)
                {
                  tripletsForLcForAll.emplace_back(i,j,LAI3(i,j));
                }
              break;
          }

	}	
    }
  for(i=num_vertex_inside;i<num_vertex;i++)
    {
      tripletsForLcForAll.emplace_back(i,i,1);
    }
  LcForAll.setFromTriplets(tripletsForLcForAll.begin(),tripletsForLcForAll.end());
  LcForAll.makeCompressed();
  linearSolver.compute(LcForAll);
  x=linearSolver.solve(x0);
  y=linearSolver.solve(y0);
  z=linearSolver.solve(z0);


  for(i=0;i<num_vertex;i++)
    {
      int r=mpFromCToR[i];
      myvertexs[r].location.x=x(i);
      myvertexs[r].location.y=y(i);
      myvertexs[r].location.z=z(i);
    }
  
  printf("end\n");
}
