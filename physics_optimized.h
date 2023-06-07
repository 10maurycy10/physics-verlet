#include "physics.h"

/////////////////
// Access Grid //
////////////////

#define MAX_PARTICLES_IN_CELL 256

// A datastructure used to optimize collision detection
// This works by breaking up space into a array of grid cells, binning all objects into a cell, to be able to quickly find nearby objects.
typedef struct AccessGrid {
	float start_x;
	float start_y;
	float cellsize;

	int x_size;
	int y_size;
	int** object_list_length;
	int** object_list;
} AccessGrid;

AccessGrid new_access_grid(int x, int y, float start_x, float start_y, float cellsize) {
	AccessGrid grid = {
		.cellsize = cellsize,
		.start_x = start_x,
		.start_y = start_y,
		.x_size = x,
		.y_size = y,
		.object_list_length = malloc(x * sizeof(int*)),
		.object_list = malloc(x * sizeof(int**)),
	};
	
 	for (int cx = 0; cx < x; cx++) {
		grid.object_list_length[cx] = malloc(x * sizeof(int));
		grid.object_list[cx] = malloc(y * sizeof(int*) * MAX_PARTICLES_IN_CELL);
		for (int cy = 0; cy < y; cy++) {
			grid.object_list_length[cx][cy] = 0;
		}
	}
	return grid;
}

void free_access_grid(AccessGrid* grid) {
	for (int x = 0; x < grid->x_size; x++) {
		free(grid->object_list_length[x]);
		free(grid->object_list[x]);
	}
}

int* access_grid_get(AccessGrid* grid, int x, int y) {
	return &grid->object_list[x][y * MAX_PARTICLES_IN_CELL];
}

void access_grid_clear(AccessGrid* grid) {
	for (int x = 0; x < grid->x_size; x++)
		for (int y = 0; y < grid->y_size; y++)
			grid->object_list_length[x][y] = 0;
}

void access_grid_append(AccessGrid* grid, int x, int y, int idx) {	
	int position = grid->object_list_length[x][y]++;
	access_grid_get(grid, x, y)[position] = idx;
}

////////////////////
// Physics solver //
////////////////////

void physics_single_check(World* w, int idx1, int idx2) {
	if (idx1 == idx2) return;
	float mindistance = w->objects[idx1].radius + w->objects[idx2].radius;

	Vector2* object1 = &w->objects[idx1].position;
	Vector2* object2 = &w->objects[idx2].position;
	Vector2 difference = vector_sub(*object1, *object2);

	float distance = vector_length(difference);

	// Check for intersections
	if (mindistance > distance) {
		float delta = (mindistance - distance) / 2;
		Vector2 adjustment = vector_mul_scaler(vector_mul_scaler(difference, 1.0/distance), delta);
		*object1 = vector_add(*object1, adjustment);
		*object2 = vector_sub(*object2, adjustment);
	}

}

// Do collison checks between all cells in 
void collide_with_cell(World* w, AccessGrid* grid, int x, int y, int idx) {
	if (x < 0 || x >= grid->x_size || y < 0 || y >= grid->y_size) return;

	int* indecies = access_grid_get(grid, x, y);
	int length = grid->object_list_length[x][y];
	for (int i = 0; i < length; i++)
		physics_single_check(w, idx, indecies[i]);
}

// An optiminzed collision solver
// max_x, min_x, max_y, min_y are the dimentrions for any particles
// Cell size should be the largest radius in the simulation
void world_optimized_collide(World* w, AccessGrid* grid) {
	// Populate the access grid with all the particles

	access_grid_clear(grid);

	for (int i = 0; i < w->size; i++) {
		Vector2 location = w->objects[i].position;
		int cellx = ((location.x - grid->start_x)/grid->cellsize);
		int celly = ((location.y - grid->start_y)/grid->cellsize);
		if (cellx >= 0 && cellx < grid->x_size && celly >= 0 && celly < grid->y_size )
			access_grid_append(grid, cellx, celly, i);
	}
	
	// For every cell in the grid
	for (int x = 0; x < grid->x_size; x++) {
		for (int y = 0; y < grid->y_size; y++) {
			// For every object in that cell
			int* indecies = access_grid_get(grid, x, y);
			int length = grid->object_list_length[x][y];
			for (int i = 0; i < length; i++) {
				collide_with_cell(w, grid, x - 1, y - 1, indecies[i]);
				collide_with_cell(w, grid, x - 1, y, indecies[i]);
				collide_with_cell(w, grid, x - 1, y + 1, indecies[i]);
				collide_with_cell(w, grid, x, y - 1, indecies[i]);
				collide_with_cell(w, grid, x, y, indecies[i]);
				collide_with_cell(w, grid, x, y + 1, indecies[i]);
				collide_with_cell(w, grid, x + 1, y - 1, indecies[i]);
				collide_with_cell(w, grid, x + 1, y, indecies[i]);
				collide_with_cell(w, grid, x + 1, y + 1, indecies[i]);
			}
		}
	}
}
