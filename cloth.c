// A super hacky cloth simualtion, an object will folow the cursor

#include <stdlib.h>
#include <stdio.h>
#include <SDL2/SDL.h>

#include "shape.h"
#include "physics.h"

#define SCREEN_WIDTH 1500
#define SCREEN_HEIGHT 1200
#define PIXELS_PER_UNIT 35

#define OBJECT_RADIUS 0.4

#define CLOTH_X	14
#define CLOTH_Y	14

int get_cloth_idx(int x, int y) {
	return y + x * CLOTH_Y;
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
	World world = world_with_capacity(1 + CLOTH_X * CLOTH_Y);

	// Create cloth
	for (int x = 0; x < CLOTH_X; x++) {
		for (int y = 0; y < CLOTH_Y; y++) {
			float wx = (float)x - ((float)CLOTH_X/2);
			float wy = (float)y - ((float)CLOTH_Y/2);
			world_spawn(&world, wx, wy, OBJECT_RADIUS);
		}
	}
	float dt = 1.0/60;
	
	// Create object for user to move
	world_spawn(&world, -10, -10, 1);
	int mx = 0, my = 0;
	
	// Run simulation
	while (1) {
		world_update_positions(&world, dt);
		
		// Object collison 
		for (int steps = 0; steps < 4; steps++) {
		
		world_collide(&world);
		
		// Fix position of top row of cloth
		for (int x = 0; x < CLOTH_X; x++) {
			constrain_distance_from_point(&world, get_cloth_idx(x, CLOTH_Y-1) , (float)x - (float)CLOTH_X/2, ((float)CLOTH_Y-1)/2, 0);
		}
		
		// Constrain cloth to be withing a certan distance of neibors
		for (int x = 0; x < CLOTH_X; x++) {
			for (int y = 0; y < CLOTH_Y; y++) {
				int idx = x * CLOTH_X + y;
				if (y < CLOTH_Y - 1)
					constrain_distance_between_objects(&world, idx, idx + 1, 1.1);
				if (x < CLOTH_X - 1)
					constrain_distance_between_objects(&world, idx, idx + CLOTH_X, 1.1);
			}
		}

		// Give user control of an object
		float control_x = -((float)mx - SCREEN_WIDTH/2) / PIXELS_PER_UNIT;
		float control_y = -((float)my - SCREEN_HEIGHT/2) / PIXELS_PER_UNIT;
		constrain_distance_from_point(&world, CLOTH_Y*CLOTH_X , control_x, control_y, 0);

		// Keep particles withing a circle
		for (int i = 0; i < world.size; i++) {
			constrain_distance_from_point(&world, i, 0, 0, 15);
		}
		}
		
		world_apply_gravity(&world, 9.8);

		// Check for input
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
				case SDL_QUIT:
					return 0;
					break;
				case SDL_MOUSEMOTION:
					mx = event.motion.x;
					my = event.motion.y;
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
