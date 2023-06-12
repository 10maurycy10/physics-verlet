// Click on the window to add objects, objects are confined to a circle in the midle of the window.

#include <stdlib.h>
#include <stdio.h>
#include <SDL2/SDL.h>
#include <assert.h>

#include "shape.h"
#include "physics.h"

#define SCREEN_WIDTH 1500
#define SCREEN_HEIGHT 1200
#define PIXELS_PER_UNIT 50

#define OBJECT_RADIUS 0.2
#define CONSTRAINT_RADIUS 0.5
#define STRAIN_THRESHOLD 10

///////////////////////////////
// Constraint array handling //
///////////////////////////////

typedef struct Constraint {
	int broken;
	int idx1;
	int idx2;
} Constraint;

typedef struct Constraints {
	Constraint* constraints;
	int size;
	int capacity;
} Constraints;

int add_constraint(Constraints* c, int idx1, int idx2) {
	Constraint constraint = { .idx1 = idx1, .idx2 = idx2, .broken = 0};
	if (c->capacity > c->size) {
		c->constraints[c->size] = constraint;
		c->size++;
		return 1;
	} else {
		return 0;
	}
}

Constraints constraints_with_capacity(int capacity) {
	Constraints c = {
		.constraints = malloc(capacity * sizeof(Constraint)),
		.size = 0,
		.capacity = capacity
	};
	return c;
}

void constraints_free(Constraints c) {
	free(c.constraints);
	c.constraints = 0;
	c.size = 0;
	c.capacity = 0;
}

void constraints_apply(World* w, Constraints* c, float dt) {
	for (int i = 0; i < c->size; i++) {
		int idx1 = c->constraints[i].idx1;
		int idx2 = c->constraints[i].idx2;
		assert(idx1 >= 0 && w->size > idx1);
		assert(idx2 >= 0 && w->size > idx2);
		if (!c->constraints[i].broken) {
			Vector2 preconstrain1 = w->objects[idx1].position;
			constrain_distance_between_objects(w, idx1, idx2, CONSTRAINT_RADIUS);
			float distance = vector_length(vector_sub(w->objects[idx1].position, preconstrain1));
			if ((distance / (dt)) > STRAIN_THRESHOLD) {
				c->constraints[i].broken = 1;
			}
		}
	}
}

/////////////////////
// Object spawning //
/////////////////////

void create_rope(
	World* w, Constraints* c,
	int count,
	float startx, float starty,
	float xoffset, float yoffset
) {
	float x = startx;
	float y = starty;
	for (int i = 0; i < count; i++) {
		world_spawn(w, x, y, OBJECT_RADIUS);
		if (i != 0)
			add_constraint(c, w->size-2, w->size-1);
		x += xoffset;
		y += yoffset;
	}	
}

void create_cloth(
	World* w, Constraints* c,
	int xcount, int ycount,
	float startx, float starty,
	float seperation
) {
	float y = starty;
	float x = startx;
	for (int ix = 0; ix < xcount; ix++) {
		for (int iy = 0; iy < ycount; iy++) {
			world_spawn(w, x, y, OBJECT_RADIUS);
			if (ix!=0) {
				add_constraint(c, w->size - 1, w->size - 1 - ycount);
			}
			if (iy!=0) {
				add_constraint(c, w->size - 1, w->size - 2);
			}
			x += seperation;
		}
		x = startx;
		y += seperation;
	} 
}
	

/////////////////////////////
// UI Helpers              //
/////////////////////////////

int get_object_at_point(World* w, Vector2 point) {
	for (int i = 0; i < w->size; i++) {
		if (vector_length(vector_sub(point, w->objects[i].position)) <= w->objects[i].radius) {
			return i;
		}
	}
}

/////////////////////////////
// The main function       //
/////////////////////////////

int main() {
	// Setup window
	int rendererFlags = SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC;
	int windowFlags = 0;

	SDL_Init(SDL_INIT_VIDEO);
	SDL_Window* window = SDL_CreateWindow("Physics", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, windowFlags);
	if (!window) {
		printf("Opening window failed: %s\n", SDL_GetError());
		return 1;
	}

	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, rendererFlags);
	if (!renderer) {
		printf("Creating renderer failed: %s\n", SDL_GetError());
		return 1;
	}

	// Setup physics engine
	World world = world_with_capacity(1024);
	Constraints constraints = constraints_with_capacity(1024);

	create_cloth(&world, &constraints, 20, 20, 5, 5, -0.5);
//	create_rope(&world, &constraints, 10, -6, 0, 0, -1);
	world_spawn(&world, -10, -10, 1);

	float dt = 1.0/60;
	
	int held_object = 0;
	int is_mouse_down = 0;
	Vector2 mouse_position;

	// Run simulation
	while (1) {
		world_update_positions(&world, dt);
		world_apply_gravity(&world, 9.8);

		// Apply constraits
		for (int steps = 0; steps < 4; steps++) {
			world_collide(&world);
			constraints_apply(&world, &constraints, dt);
                	for (int i = 0; i < world.size; i++) {
				constrain_bounding_box(&world, i, -10, 10, -10, 10);
	                }

                	for (int y = 0; y < 20; y++) {
				int x = 0;
        	                constrain_distance_from_point(&world, x * 10 + y, 5-((float)y/2), 5, 0);
			}
	
			if (is_mouse_down) {
        	                constrain_distance_from_point(
					&world,
					held_object,
					mouse_position.x, mouse_position.y,
					0);
			}
		}

		// Check for input
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
				case SDL_QUIT:
					return 0;
					break;
				case SDL_MOUSEMOTION:
					float x = -((float)event.motion.x - SCREEN_WIDTH/2) / PIXELS_PER_UNIT;
					float y = -((float)event.motion.y - SCREEN_HEIGHT/2) / PIXELS_PER_UNIT;
					mouse_position.x = x;
					mouse_position.y = y;
					break;
				case SDL_MOUSEBUTTONDOWN:
					is_mouse_down = 1;
					held_object = get_object_at_point(&world, mouse_position);
					break;
				case SDL_MOUSEBUTTONUP:
					is_mouse_down = 0;
					break;
				default:
					break;
			}
		}

		// Draw to screen
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);

		for (int i = 0; i < world.size; i++) {
			int color = (i * 20 * i % 256);

			SDL_SetRenderDrawColor(renderer, color, 255-color, 255, 255);
			int x = (-world.objects[i].position.x * PIXELS_PER_UNIT) + (SCREEN_WIDTH/2);
			int y = (-world.objects[i].position.y * PIXELS_PER_UNIT) + (SCREEN_HEIGHT/2);
			int r = (world.objects[i].radius * PIXELS_PER_UNIT);
		
			draw_circle(renderer, x, y, r);
			
		}

		SDL_RenderPresent(renderer);
	}
	

	world_cleanup(&world);
}
