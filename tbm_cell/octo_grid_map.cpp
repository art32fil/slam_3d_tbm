#include "octo_grid_map.h"
#include "common_features.h"
#include <iostream>
#include <stdlib.h>

using std::cout;
using std::endl;

OctoGridMap::OctoGridMap(int Nx, int Ny, int Nz):
  cells(Nx, vector<vector<TBM3>>(Ny, vector<TBM3>(Nz, empty_cell))) {
	zero_x = Nx/2; zero_y = Ny/2; zero_z = Nz/2;
}

OctoGridMap::OctoGridMap(TBM3*** cells, int Nx, int Ny, int Nz) {
	zero_x = Nx/2; zero_y = Ny/2; zero_z = Nz/2;
	this->cells = vector<vector<vector<TBM3>>>((Nx, vector<vector<TBM3>>(Ny, vector<TBM3>(Nz, empty_cell))));
	for (int i = 0; i < Nx; i++) {
		for (int j = 0; j < Ny; j++) {
			for (int k = 0; k < Nz; k++) {
				this->cells[i][j][k] = cells[i][j][k];
			}
		}
	}
}

OctoGridMap::OctoGridMap(const vector<vector<vector<TBM3>>>& cells):
  empty_cell(0, 0, 0, 0, 0, 0, 1) {
	zero_x = cells.size()/2; zero_y = cells[0].size()/2; zero_z = cells[0][0].size()/2;
	this->cells = cells;
}

OctoGridMap::OctoGridMap(vector<vector<vector<TBM3>>>&& cells):
  empty_cell(0, 0, 0, 0, 0, 0, 1) {
	zero_x = cells.size()/2; zero_y = cells[0].size()/2; zero_z = cells[0][0].size()/2;
	this->cells = cells;
}

TBM3& OctoGridMap::operator [](int x, int y, int z) {
	int i = x + zero_x;
	int j = y + zero_y;
	int k = z + zero_z;

	if (i < 0 or j < 0 or k < 0) {
		cout << "(" << x << ", " << y << ", " << z << ") coords where zero is at "
		     << "(" << zero_x << ", " << zero_y << ", " << zero_z << ") comes to negative array index!" << endl;
		std::exit(0);
	}

	if (i < cells.size() and j < cells[0].size() and k < cells[0][0].size())
		return cells[i][j][k];
}
