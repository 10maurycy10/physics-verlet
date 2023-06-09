// Click on the window to add objects, objects are confined to a circle in the midle of the window.

#include <stdlib.h>
#include <stdio.h>
#include <SDL2/SDL.h>

#include "shape.h"
#include "physics_optimized.h"

#define SCREEN_WIDTH 1500
#define SCREEN_HEIGHT 1200
#define PIXELS_PER_UNIT 25

#define TIMESTEP (1.0/60/3)
#define SPAWN_DELAY 2
#define SPAWN_Y 19
#define MAX_COUNT 20000

/////////////////////////////
// The main function       //
/////////////////////////////

int main() {
	
	// Setup window
	int rendererFlags = SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC;
	int windowFlags = 0;

	SDL_Init(SDL_INIT_VIDEO);
	SDL_Window* window = SDL_CreateWindow("Physics: Stress test", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, windowFlags);
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
	AccessGrid grid = new_access_grid(42*4, 42*4, -21, -21, 0.25);
	World world = world_with_capacity(MAX_COUNT);

	float dt = TIMESTEP;
	int tick = 0;
	int last_realtime_count = 0;
	
	// Run simulation
	while (1) {
		int start_ms = SDL_GetTicks();
		for (int i = 0; i <3; i++) {
			world_update_positions(&world, dt);
			world_optimized_collide(&world, &grid);
			world_optimized_collide(&world, &grid);
//			world_collide(&world);
		
			for (int i = 0; i < world.size; i++) {
				constrain_bounding_box(&world, i, -20, 20, -20, 20);
			}
		
			world_apply_gravity(&world, 9.8);
		}
		int end_ms = SDL_GetTicks();
		printf("%d Objects, %d simulation ms\n", world.size, end_ms-start_ms);
		if(end_ms - start_ms > 16) {
			printf("Not realtime! Last realtime object count: %d\n", last_realtime_count);
		} else {
			last_realtime_count = world.size;
		}

		if (tick % SPAWN_DELAY == 0) {
			for (float x = -15; x < 15; x++) {
				Body object = physics_new_with_position(x, SPAWN_Y, 0.1);
				object.position.x -= 0.04;
				object.position.y -= 0.04;
				world_insert_object(&world, object);
			}
		}
		tick++;

		// Check for input
		SDL_Event event;
		while (SDL_PollEvent(&event)) {
			switch (event.type) {
				case SDL_QUIT:
					return 0;
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
	

	free_access_grid(&grid);
	world_cleanup(&world);
}
