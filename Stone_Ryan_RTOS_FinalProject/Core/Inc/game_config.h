/*
 * game_config.h
 *
 *  Created on: Apr 11, 2024
 *      Author: ryenots
 */

#include "stdbool.h"

#ifndef INC_GAME_CONFIG_H_
#define INC_GAME_CONFIG_H_

#define VERSION 1

typedef enum{
	drone_at_center = 0,
	maze_at_center
}pin_at_center;

struct{
	int gravity;
	int update_frequency;
	pin_at_center center;
	int angleGain;
}physics = {980, 50, drone_at_center, 500};

struct Disruptor{
	int max_time;    		  // ms
	int power;    			  // mW
	int min_activation_energy; // mJ
}disruptor_ = {1000, 10000, 6000};

struct Energy_Store{
	int max_energy;    // mJ
	int recharge_rate; // mW
}energy_store_ = {15000, 1000};

struct{
	struct Disruptor* disruptor;
	struct Energy_Store* energy_store;
	int diameter;
}drone = {&disruptor_, &energy_store_, 8};

struct Size{
	int width;
	int height;
}size_ = {16, 21};

struct Obstacle_Probability{
	int wall;
	int hole;
}obstacle_probability_ = {100, 200};

typedef struct {
	int x;
	int y;
	bool collected; // only for waypoints, don't modify for holes.
}xy_pair;

struct Waypoints{
	int number;
	int diameter;
	bool reuse;
	xy_pair* locations_list; // dynamic alloc this list
}waypoints_ = {4, 6, 0, NULL};

struct Holes{
	int number;
	int diameter;
	xy_pair* locations_list;
}holes_ = {12, 12, NULL};

struct{
	int time_to_complete;
	int cell_size; //px
	struct Size* size;
	struct Obstacle_Probability* o_p;
	int hole_diameter;
	int hard_edged;
	struct Waypoints* waypoints;
	struct Holes* holes;
}maze_config = {30000, 15, &size_, &obstacle_probability_, 11, 1, &waypoints_, &holes_};

typedef struct Cell{
	bool top;
	bool right;
}Cell;

#endif /* INC_GAME_CONFIG_H_ */
