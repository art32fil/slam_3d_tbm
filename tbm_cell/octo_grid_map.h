#ifndef OCTO_GRID_MAP_H_
#define OCTO_GRID_MAP_H_

#include "tbm3.h"
#include <vector>

using std::vector;

struct Coord3D {
	int i;
	int j;
	int k;
};

class OctoGridMap {

	vector<vector<vector<TBM3>>> cells;
	const TBM3 empty_cell = TBM3(0,0,0,0,0,0,1);
	int zero_x;
	int zero_y;
	int zero_z;

public:
	OctoGridMap(int Nx = 0, int Ny = 0, int Nz = 0);
	OctoGridMap(TBM3*** cells, int Nx, int Ny, int Nz);
	OctoGridMap(const vector<vector<vector<TBM3>>>& cells);
	OctoGridMap(vector<vector<vector<TBM3>>>&& cells);

	TBM3& operator [] (Coord3D c);
};

#endif  /* OCTO_GRID_MAP_H_ */
