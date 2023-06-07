// Click on the window to add objects, objects are confined to a circle in the midle of the window.

#include <stdlib.h>
#include <stdio.h>

#include "shape.h"
#include "physics.h"
#include <SDL2/SDL.h>

#define SCREEN_WIDTH 1500
#define SCREEN_HEIGHT 1200
#define PIXELS_PER_UNIT 25

#define OBJECT_RADIUS 0.4

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

	float dt = 1.0/60;
	
	// Run simulation
	while (1) {
		world_update_positions(&world, dt);
	
		// Apply constraits. The constaints limiting distance between objects are run multiple times to improve rigity.
		world_collide(&world);
		for (int steps = 0; steps < 10; steps++) {
			for (int i = 0; i < world.size - 1; i++) {
				constrain_distance_between_objects(&world, i, i+1, 1);
			}
			constrain_distance_from_point(&world, 0, 0, 0, 0);
		}

		// Step physics
		world_apply_gravity(&world, 9.8);

		// Check for input
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
				case SDL_QUIT:
					return 0;
					break;
				case SDL_MOUSEBUTTONDOWN:
					float x = -((float)event.button.x - SCREEN_WIDTH/2) / PIXELS_PER_UNIT;
					float y = -((float)event.button.y - SCREEN_HEIGHT/2) / PIXELS_PER_UNIT;
					world_spawn(&world, x, y, OBJECT_RADIUS);
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
