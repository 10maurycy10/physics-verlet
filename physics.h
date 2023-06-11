// A simple, single file 2d Verlet integration based physics engine.
//
// To use it, first initalize the World struct using the world_with_capacity, add objects using world_spawn
//
// To simulate it call world_step every timestep.
// To add gravity call apply_gravity every timestep and pass the acceleration due to gravity.
// Constraints are applied by calling the constriant function every frame. The most important one is world_collide,
// which implements a simple non-intersection constraint, assuming every object has equal mass and collisions are
// inelastic.
// 
// There is nothing special about the constraint_* and world_collide functions, you can get the same effect by simple manipulating object's .position property.
//
// If constraints are not a rigid as they should be, or collisions start becoming unstable, try running the constraints multiple times per timestep, or/and reducing the timestep.


#ifndef HAS_PHYSICS
#define HAS_PHYSICS 1

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <SDL2/SDL.h>

///////////////////////////////
// low level math functions. //
///////////////////////////////

// A 2d Vector type
typedef struct Vector2 {
	float x;
	float y;
} Vector2;

// Vector addition
Vector2 vector_add(Vector2 v1, Vector2 v2) {
	Vector2 output = {.x = v1.x + v2.x, .y = v1.y + v2.y};
	return output;
}

// Vector subtraction
Vector2 vector_sub(Vector2 v1, Vector2 v2) {
	Vector2 output = {.x = v1.x - v2.x, .y = v1.y - v2.y};
	return output;
}

// Scale a vector by a value.
Vector2 vector_mul_scaler(Vector2 v1, float s) {
	Vector2 output = {.x = v1.x * s, .y = v1.y * s};
	return output;
}

// Compute the magnitude of a vector
float vector_length(Vector2 v1) {
	return sqrt(v1.x * v1.x + v1.y * v1.y);
}


////////////////////////////////////////////////////////////////////////////////////////
// Object for Verlet inegration.                                                      //
// Velocity is not stored and is extrapolated based on the current and past positions //
// This means that you dont have to update the velocity when messing with the position//
// This means that the timestep must stay constant for the whole simulation           //
////////////////////////////////////////////////////////////////////////////////////////

// A body is a circle centered on position with given radius
// The accleration vector is used to sum up the effects of forces, just add to it every frame to apply a force.
// The possition can be adjusted manualy, but this will add velocity. Adjust both the old and current possition to avoid this.
typedef struct Body {
	float radius;

	Vector2 position_old;
	Vector2 position;
	Vector2 acceleration; 
} Body;

// Use Verlet integration to apply velocity and acceleration to the body.
// This is the core of the physics simulation, and should be called every timestep.
void physics_update_position(Body* body, float dt) {
	// Compute velocity in terms of timestep based on position
	Vector2 velocity = vector_sub(body->position, body->position_old);
	// Save the velocity before updating
	body->position_old = body->position;
	// Move the object by the velocity, and the acceleration. This is one of the advantages of verlet integration, less latency for applied forces
	body->position = vector_add(body->position,vector_add(velocity, vector_mul_scaler(body->acceleration, dt * dt )));
	// Reset acceleration to 0
	body->acceleration.x = 0;
	body->acceleration.y = 0;
}

// Convenece function to create a new body at a given possition, with 0 acceleration and 0 velocity.
Body physics_new_with_position(float x, float y, float r) {
	Body b = {
		.radius = r,
		.position_old = {.x = x,  .y = y},
		.position = {.x = x,  .y = y},
		.acceleration = {.x = 0, .y = 0}
	};
	return b;
}

////////////////////////////////////////////
// A colection of objects for simulation. //
////////////////////////////////////////////

// This is a collection of objects for simulation.
// The .objects array can be allocated staticly, on the stack, or on the heap
typedef struct World {
	// This should be a pointer to .capacity body structs.
	Body* objects;
	// How many objects are in the world at the given moment, initalize to 0.
	int size;
	// The total amount that can be stored at a given time.
	int capacity;
} World;

// Allocate a empty world with a capacity to hold up to capacity objects
// This allocates .objects on the heap, so make sure to call world_cleanup before discarding the World object.
World world_with_capacity(int capacity) {
	World w = {
		.objects = malloc(capacity * sizeof(Body)),
		.size = 0,
		.capacity = capacity
	};
	return w;
}

// Frees the objects array of a world, call before discarding if it was heap allocated
void world_cleanup(World* w) {
	free(w->objects);
	// Set these to make sure no one tries to access any of the freed datastructures;
	w->size = 0;
	w->capacity = 0;
	w->objects = 0;
}

// Add an object to a world, fails if there is no space left in the world.
// Returns 1 if sucessfull, 0 if not.
int world_insert_object(World* world, Body object) {
	if (world->capacity > world->size) {
		world->objects[world->size] = object;
		world->size++;
		return 1;
	} else {
		return 0;
	}
}


// Run Verlet integration for the whole world, call this every timestep
void world_update_positions(World* w, float dt) {
	for (int i = 0; i < w->size; i++) {
		physics_update_position(&w->objects[i], dt);
	}
}

// Apply a downwards acceleration to all objects in a world, this sould also be called every timestep if you want gravity to be applied.
void world_apply_gravity(World* w, float g) {
	for (int i = 0; i < w->size; i++) {
		w->objects[i].acceleration.y -= g;
	}
}

// Apply collisions in a world, this is rather slow, see physics_optimized.h for a faster implementations for large simulations.
// This should be called every frame if you want objects to collide with each other.
void world_collide(World* w) {
	// This is fairly simple, it just finds all intersecting objects and moves them until they no longer intesect.
	// It is however, rather slow, O(n^2), this should be optimized at some point.
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
				float delta = (mindistance - distance) / 2;
				Vector2 adjustment = vector_mul_scaler(vector_mul_scaler(difference, 1.0/distance), delta);
				*object1 = vector_add(*object1, adjustment);
				*object2 = vector_sub(*object2, adjustment);
			
			}
		}
	}
}

// Create an object with given position and radius in the world, returns 1 if sucessful, 0 if object max is exeded.
// This is just shorthand for world_insert_object(world, physics_new_with_position(x, y, r)).
int world_spawn(World* w, float x, float y, float r) {
	Body object = physics_new_with_position(x, y, r);
	return world_insert_object(w, object);
}

///////////////////////////////////////////////////////////
// Constriants, these should be called once every frame  //
// There is no magic here, you can implement your own by //
// simply changing the position of objects so that they  //
// satisfy the constraint.                               //
///////////////////////////////////////////////////////////


// Keep an object('s center) within a radius of a point.
void constrain_distance_from_point(World* w, int objectidx, float x, float y, float maxd) {
	Vector2* object = &w->objects[objectidx].position;
	Vector2 origin = {.x = x, .y = y};
	
	// Edge case handling
	if (maxd == 0) {
		*object = origin;
		return;
	}

	Vector2 difference = vector_sub(*object, origin);
	if (vector_length(difference) > maxd) {
		float correction = (vector_length(difference)-maxd)/maxd;
		*object	= vector_sub(*object, vector_mul_scaler(difference, correction));
	}
}

// Keep the distance between 2 objects below a certan distance
void constrain_distance_between_objects(World* w, int idx1, int idx2, float maxd) {
	Vector2* object1 = &w->objects[idx1].position;
	Vector2* object2 = &w->objects[idx2].position;
	Vector2 difference = vector_sub(*object1, *object2);

	float distance = vector_length(difference);
	
	if ( distance > maxd ) {
		float delta = (distance-maxd) / 2;

		Vector2 adjustment = vector_mul_scaler(vector_mul_scaler(difference, 1.0/distance), delta);
		*object1 = vector_sub(*object1, adjustment);
		*object2 = vector_add(*object2, adjustment);
	}
}

// Keep object('s center) withing a bounding box
void constrain_bounding_box(World* w, int idx, float minx, float maxx, float miny, float maxy) {
	Vector2* object = &w->objects[idx].position;
	if (object->x > maxx) object->x = maxx;
	if (object->y > maxy) object->y = maxy;
	if (object->x < minx) object->x = minx;
	if (object->y < miny) object->y = miny;
}

#endif
