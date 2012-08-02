#ifndef VIEWINGMODEL_H_
#define VIEWINGMODEL_H_

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <string>

#include "UserConstant.h"

struct Mesh
{
  unsigned int nIndex;
  std::vector<float> vertex;
  std::vector<float> normal;
  std::vector<unsigned int> index;
  std::vector<unsigned int> ind;
  int ind_max;
  int flag;
};

class ViewingModel{
 public:
  ViewingModel();
  ViewingModel(std::string name);
  ~ViewingModel();

  int GetMeshSize() const;
  int GetMeshIndicesSum(const int & outer_loop) const;
  int GetMeshFlag(const int & outer_loop) const;

	double GetXMin(void) const { return xmin; }
	double GetXMax(void) const { return xmax; }
	double GetYMin(void) const { return ymin; }
	double GetYMax(void) const { return ymax; }
	double GetZMin(void) const { return zmin; }
	double GetZMax(void) const { return zmax; }

 private:
  void Load3dsModel();

 private:
  int sum_of_vertex; // the number of vertices of this model
	double xmin, xmax;
	double ymin, ymax;
	double zmin, zmax;
	std::vector<Mesh> m_mesh;

	std::string modelname;
  
};

#endif
