// A simple Verlet integration based physics engine, using SDL for simple rendering and UI.
// Click on the window to add objects, objects are confined to a circle in the midle of the window.

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <SDL2/SDL.h>

#define SCREEN_WIDTH 1500
#define SCREEN_HEIGHT 1200
#define PIXELS_PER_UNIT 50

#define OBJECT_RADIUS 0.4

// Scaling factor for how much to move objects once a colision is found. 1: minium, 2: twice that.
// Higher values make objects more bouncy.
// Values above 3 violate conservation of energy and cause objects to gain speed by colliding, values under 1 prevent the solver from resolving collions.
#define COLLIDE_MOVE_SCALE 2

// Same for constriants
#define CONSTRAINT_MOVE_SCALE 2

///////////////////////////////
// low level math functions. //
///////////////////////////////

typedef struct Vector2 {
	float x;
	float y;
} Vector2;

Vector2 vector_add(Vector2 v1, Vector2 v2) {
	Vector2 output = {.x = v1.x + v2.x, .y = v1.y + v2.y};
	return output;
}

Vector2 vector_sub(Vector2 v1, Vector2 v2) {
	Vector2 output = {.x = v1.x - v2.x, .y = v1.y - v2.y};
	return output;
}

Vector2 vector_mul_scaler(Vector2 v1, float s) {
	Vector2 output = {.x = v1.x * s, .y = v1.y * s};
	return output;
}

float vector_length(Vector2 v1) {
	return sqrt(v1.x * v1.x + v1.y * v1.y);
}


////////////////////////////////////////////////////////////////////////////////////////
// Object for verlet inegration.                                                      //
// Velocity is not stored and is extrapolated based on the current and past positions //
// This means that we dont have to update the velocity when messing with the position //
// This means that the timestep must stay constant for the whole simulation           //
////////////////////////////////////////////////////////////////////////////////////////

typedef struct Body {
	float radius;
	int color;

	Vector2 position_old;
	Vector2 position;
	Vector2 acceleration; 
} Body;

// Use verlet integration to apply velocity and acceleration to the body
void update_position(Body* body, float dt) {
	// Compute velocity in terms of timestep based on position
	Vector2 velocity = vector_sub(body->position, body->position_old);
	// Save the velocity before updating
	body->position_old = body->position;
	// Move the object by the velocity, and the acceleration. This is one of the advantages of verlet integration, less latency for applied forces
	body->position = vector_add(body->position,vector_add(velocity, vector_mul_scaler(body->acceleration, dt * dt )));
	// Reset acceleration
	body->acceleration.x = 0;
	body->acceleration.y = 0;
}

Body new_with_position(float x, float y) {
	Body b = {
		.color = rand()%256,
		.radius = OBJECT_RADIUS,
		.position_old = {.x = x,  .y = y},
		.position = {.x = x,  .y = y},
		.acceleration = {.x = 0, .y = 0}
	};
	return b;
}

////////////////////////////////////////////
// A colection of objects for simulation. //
////////////////////////////////////////////

typedef struct World {
	Body* objects;
	int size; // How many objects are in the world at the given moment
	int capacity; // The total amount that can be stored at a given time
} World;

World world_with_capacity(int capacity) {
	World w = {
		.objects = malloc(capacity * sizeof(Body)),
		.size = 0,
		.capacity = capacity
	};
	return w;
}

// Returns 1 if sucessfull, zero if not
int insert_object(World* world, Body object) {
	if (world->capacity > world->size) {
		world->objects[world->size] = object;
		world->size++;
	} else {
		return 0;
	}
}

// Frees internal datastructures of a world, does not free the passed pointer. 
void cleanup_world(World* w) {
	free(w->objects);
	// Set these to make sure no one tries to access any of the freed datastructures;
	w->size = 0;
	w->capacity = 0;
	w->objects = 0;
}

void update_positions(World* w, float dt) {
	for (int i = 0; i < w->size; i++) {
		update_position(&w->objects[i], dt);
	}
}

void apply_gravity(World* w) {
	for (int i = 0; i < w->size; i++) {
		w->objects[i].acceleration.y -= 9.8;
	}
}

void collide(World* w) {
	// Bould the object to sqrt(10) units away from 0 0
	for (int i = 0; i < w->size; i++) {
		if (vector_length(w->objects[i].position) > 10) {
			float position_scale = 10 / vector_length(w->objects[i].position);
			position_scale = 1-((1-position_scale) * CONSTRAINT_MOVE_SCALE);
			w->objects[i].position = vector_mul_scaler(w->objects[i].position,position_scale);
		}
	}
	// Check object colision, this is super inefficent.
	for (int i = 0; i < w->size; i++) {
		// This iterates up to i and not w->size to avoid rendundent checks and checking an object against itself
		for (int e = 0; e < i; e++) {
			float mindistance = w->objects[i].radius + w->objects[e].radius;
			
			Vector2* object1 = &w->objects[i].position;
			Vector2* object2 = &w->objects[e].position;
			Vector2 difference = vector_sub(*object1, *object2);
			
			float distance = vector_length(difference);
			
			// Check for intersections
			if (mindistance > distance) {
				float delta = (mindistance - distance) / 2 * COLLIDE_MOVE_SCALE;
				Vector2 adjustment = vector_mul_scaler(vector_mul_scaler(difference, 1.0/distance), delta);
				*object1 = vector_add(*object1, adjustment);
				*object2 = vector_sub(*object2, adjustment);
			
			}
		}
	}
}

void spawn(World* w, float x, float y) {
	Body object = new_with_position(x, y);
	if (!insert_object(w, object)) {
		printf("Cant insert object\n");
	}
}

/////////////////////////////
// The main function       //
/////////////////////////////

int main() {
	
	// Setup window
	int rendererFlags = SDL_RENDERER_ACCELERATED;
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
		// Step physics
		apply_gravity(&world);
		update_positions(&world, dt);
		for (int i = 0; i < 16; i++) {
			collide(&world);
		}

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
					spawn(&world, x, y);
				default:
					break;
			}
		}

		// Draw to screen
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);

		for (int i = 0; i < world.size; i++) {
			SDL_SetRenderDrawColor(renderer, world.objects[i].color, 0, 255, 255);
			int x = (-world.objects[i].position.x * PIXELS_PER_UNIT) + (SCREEN_WIDTH/2);
			int y = (-world.objects[i].position.y * PIXELS_PER_UNIT) + (SCREEN_HEIGHT/2);
			int r = (world.objects[i].radius * PIXELS_PER_UNIT);
		
			SDL_Rect bounding_box = {.x = x - r, .y = y -r, .w = r*2, .h = r*2};
			SDL_RenderFillRect(renderer, &bounding_box);
			
		}

		SDL_RenderPresent(renderer);
		SDL_Delay(16);
	}
	

	cleanup_world(&world);
}
