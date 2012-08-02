#include "main.h"
#include "ViewingModel.h"

#include <lib3ds.h>
#include <iostream>
#include <stdio.h>

using namespace std;

#define REP(i,n) for(int i=0;i<(int)n;++i)
//const char*  modelname = "Data/Cars/HatuneMiku.3DS";

void ViewingModel::Load3dsModel()
{
  Lib3dsFile *m_model; //モデル全体
  Lib3dsMesh *mesh; //メッシュ単位
  
  //モデル読み込み
	m_model = lib3ds_file_open(modelname.c_str());
  if(m_model==NULL){
    std::cerr << "can't Open file\n";
    exit(0);
  }
    
  m_mesh.resize(m_model->nmeshes);//メッシュ分メモリ確保
  for(int loop = 0; loop < m_model->nmeshes;++loop){
    mesh = m_model->meshes[loop];//mLoop番目のメッシュへのポインタ
    if(mesh->nfaces == 0) {
      m_mesh[loop].flag = 1;
      continue;
    }//メッシュが無い場合はカット
    
    
    //法線データの取り出し
    float (*normal)[3] = new float[mesh->nfaces][3];
    lib3ds_mesh_calculate_face_normals(mesh,&normal[0]);//面法線の取り出し
    
    m_mesh[loop].normal.resize(mesh->nfaces*3*3);//法線用メモリ確保(面の数*3頂点*3座標)
    m_mesh[loop].vertex.resize(mesh->nfaces*3*3);//頂点用メモリ確保(頂点数*3要素)
    
    m_mesh[loop].nIndex = mesh->nfaces * 3;
    m_mesh[loop].ind.resize( m_mesh[loop].nIndex );//面の数*3頂点分=総インデックス数
    
    //インデックス最大値の初期化
    m_mesh[loop].ind_max = 0;
    //頂点データと法線データをインデックスに合わせて格納
    for(int loopX = 0; loopX < mesh->nfaces;++loopX){
      //1頂点目
      memcpy(&m_mesh[loop].normal[loopX*9],&normal[loopX][0],sizeof(float)*3);
      memcpy(&m_mesh[loop].vertex[loopX*9],&mesh->vertices[ mesh->faces[loopX].index[0] ][0],sizeof(float)*3);
      //２頂点目
      memcpy(&m_mesh[loop].normal[loopX*9+3],&normal[loopX][0],sizeof(float)*3);
      memcpy(&m_mesh[loop].vertex[loopX*9+3],&mesh->vertices[ mesh->faces[loopX].index[1] ][0],sizeof(float)*3);
      //3頂点目
      memcpy(&m_mesh[loop].normal[loopX*9+6],&normal[loopX][0],sizeof(float)*3);
      memcpy(&m_mesh[loop].vertex[loopX*9+6],&mesh->vertices[ mesh->faces[loopX].index[2] ][0],sizeof(float)*3);

      //各x,y,zの最大・最小値探索
			REP(ind,3){
				if( m_mesh[loop].vertex[loopX*9 + ind*3] < xmin){
					xmin = m_mesh[loop].vertex[loopX*9];
				}
				if( m_mesh[loop].vertex[loopX*9 + ind*3] > xmax){
					xmax = m_mesh[loop].vertex[loopX*9];
				}
				if( m_mesh[loop].vertex[loopX*9 + ind*3 + 1] < ymin){
					ymin = m_mesh[loop].vertex[loopX*9];
				}
				if( m_mesh[loop].vertex[loopX*9 + ind*3 + 1] > ymax){
					ymax = m_mesh[loop].vertex[loopX*9];
				}
				if( m_mesh[loop].vertex[loopX*9 + ind*3 + 2] < zmin){
					zmin = m_mesh[loop].vertex[loopX*9];
				}
				if( m_mesh[loop].vertex[loopX*9 + ind*3 + 2] > zmax){
					zmax = m_mesh[loop].vertex[loopX*9];
				}
			}


      // 現在のメッシュの3頂点に対してインデックスの設定
      for(int idmesh=0;idmesh<3;idmesh++){
				m_mesh[loop].ind[loopX*3+idmesh] = 0;
				// インデックス番号の重複をさけるために
				// これまでのメッシュグループのインデックス最大値を足す
				REP(i,loop){
					m_mesh[loop].ind[loopX*3+idmesh] += m_mesh[i].ind_max;			    
				}
				// 頂点のインデックス値を設定
				m_mesh[loop].ind[loopX*3+idmesh] += mesh->faces[loopX].index[idmesh];
				// 現在のメッシュグループの中で最大のインデックスを探索
				if( m_mesh[loop].ind_max < mesh->faces[loopX].index[idmesh]){
					m_mesh[loop].ind_max = mesh->faces[loopX].index[idmesh];
				}
      }
    }
    
    m_mesh[loop].nIndex = mesh->nfaces * 3;
    m_mesh[loop].index.resize( m_mesh[loop].nIndex );//面の数*3頂点分=総インデックス数
    
    delete [] normal;    
  }        
  lib3ds_file_free(m_model);
  cout << "Mesh Size : " << m_mesh.size() << endl;
}

int ViewingModel::GetMeshSize() const 
{ 
  return ( static_cast<int>(m_mesh.size()) ); 
}

int ViewingModel::GetMeshIndicesSum(const int & outer_loop) const 
{
  if( static_cast<int>(m_mesh.size()) <= outer_loop){
    cerr << "Error(GetMeshIndicesSum): out of range of mesh array" << endl;
    return 1;
  }
  return ( m_mesh[outer_loop].nIndex ); 
}

int ViewingModel::GetMeshFlag(const int & outer_loop) const
{
  if( static_cast<int>(m_mesh.size()) <= outer_loop){
    cerr << "Error(GetMeshFlag): out of range of mesh array" << endl;
    return 1;
  }
  return m_mesh[outer_loop].flag;
}

ViewingModel::ViewingModel()
  :sum_of_vertex(0)
{ 
	modelname = "Data/Cars/banana.3DS";
	xmin = 0.0; xmax = 0.0;
	ymin = 0.0; ymax = 0.0;
	zmin = 0.0; zmax = 0.0;
  Load3dsModel();
}

ViewingModel::ViewingModel(string name)
  :sum_of_vertex(0), modelname(name)
{  
	xmin = 0.0; xmax = 0.0;
	ymin = 0.0; ymax = 0.0;
	zmin = 0.0; zmax = 0.0;
  Load3dsModel();
}

ViewingModel::~ViewingModel()
{
}
