#include <SDL2/SDL.h>

void draw_circle(SDL_Renderer* renderer, int32_t centreX, int32_t centreY, int32_t radius) {
	const int32_t diameter = (radius * 2);

	int32_t x = (radius - 1);
	int32_t y = 0;
	int32_t tx = 1;
	int32_t ty = 1;
	int32_t error = (tx - diameter);

	while (x >= y) {
		// Each of the following renders an octant of the circle
		SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
		SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
		SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
		SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
		SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
		SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
		SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
		SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);

		if (error <= 0)  {
			++y;
			error += ty;
			ty += 2;
  		}
  		if (error > 0) {
			--x;
			tx += 2;
			error += (tx - diameter);
		}
	}
}

int round_up_to_multiple_of_8(int v) {
    return (v + (8 - 1)) & -8;
}

void draw_circle_fast( SDL_Renderer * renderer, int cx, int cy, int radius) {
	// 35 / 49 is a slightly biased approximation of 1/sqrt(2)
	int arrSize = round_up_to_multiple_of_8( radius * 8 * 35 / 49 );
	SDL_Point points[arrSize];
	int drawCount = 0;

	const int32_t diameter = (radius * 2);

	int32_t x = (radius - 1);
	int32_t y = 0;
	int32_t tx = 1;
	int32_t ty = 1;
	int32_t error = (tx - diameter);

	while( x >= y ) {
		// Each of the following renders an octant of the circle
		points[drawCount+0].x = cx + x; points[drawCount+0].y = cy - y;
		points[drawCount+1].x = cx + x; points[drawCount+1].y = cy + y;
		points[drawCount+2].x = cx - x; points[drawCount+2].y = cy - y;
		points[drawCount+3].x = cx - x; points[drawCount+3].y = cy + y;
		points[drawCount+4].x = cx + y; points[drawCount+4].y = cy - x;
		points[drawCount+5].x = cx + y; points[drawCount+5].y = cy + x;
		points[drawCount+6].x = cx - y; points[drawCount+6].y = cy - x;
		points[drawCount+7].x = cx - y; points[drawCount+7].y = cy + x;

		drawCount += 8;

		if( error <= 0 ) {
			++y;
			error += ty;
			ty += 2;
		}
		if( error > 0 ) {
			--x;
			tx += 2;
			error += (tx - diameter);
		}
	}
	SDL_RenderDrawPoints(renderer, points, drawCount);
}
