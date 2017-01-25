#pragma once
#include "kilolib.h"
//#include "polygon_detect.cpp"
#include <iostream>

typedef struct point_t
{
	double x;
	double y;
} point_t;

// Define a square for testing location detection
point_t shape[7] = {{0, 0}, {0, 2000}, {2000, 2000}, {2000, 1000}, {1000, 1000}, {1000, 0}, {0, 0}};

static bool point_in_polygon(point_t point, point_t polygon[], uint8_t n_vertices) {
	bool in = false;
	double px = point.x;
	double py = point.y;

	for (uint8_t i = 0, j = n_vertices - 1; i < n_vertices; j = i++) {
		double ix = polygon[i].x;
		double iy = polygon[i].y;
		double jx = polygon[j].x;
		double jy = polygon[j].y;

		if( ((iy > py) != (jy > py)) &&
				(px < ((jx - ix) * (py - iy) / (jy - iy)) + ix)) {
			in = !in;
		}
	}
	return in;
}




class mykilobot : public kilobot {
	unsigned char distance;
	message_t out_message;
	int rxed=0;

	int motion=0;
	long int motion_timer=0;

	int msrx=0;
	struct mydata {
		unsigned int data1;
		unsigned int data2;
	};

	//main loop
	void loop() {
	    // Set colors based on inside/outside box
	    //if (pos[0] >= square[0] && pos[0] <= square[2] && pos[1] >= square[1] && pos[1] <= square[3]) {
	    point_t p;
	    p.x = pos[0];
	    p.y = pos[1];
	    if (point_in_polygon(p, shape, sizeof(shape)/16)) {
	        set_color(RGB(0,1,0));
	    } else {
	        set_color(RGB(1,0,0));
	    }

		//set_motors(kilo_straight_left, kilo_straight_right);
		if (rxed) {
			rxed=0;
			if (motion==1) {
				if (out_message.data[0]<(id)) {
					motion=0;
					motion_timer=kilo_ticks;//kilo_ticks is the kilobots clock
				}
			}
		}
		if (motion==0) {
			if(kilo_ticks>motion_timer+100)
				motion=1;
		}
		if (motion==0) {
			set_motors(0, 0);//turn off motors
			//set_color(RGB(1,0,0));//set color
		} else {
			//set_color(RGB(0,1,0));

			if (rand()%100<90) {
			spinup_motors();//first start motors
			set_motors(kilo_straight_left, kilo_straight_right);//then command motion
			} else if (rand()%100<95) {
    			spinup_motors();
				set_motors(0, kilo_turn_right);
			} else {
			spinup_motors();
			set_motors(kilo_turn_left, 0);
			}
		}
		//update message
		out_message.type = NORMAL;
		out_message.data[0] = id;
		out_message.data[1] = 0;
		out_message.data[2] = 0;
		out_message.crc = message_crc(&out_message);
	}

	//executed once at start
	void setup() {
	  double x = this->pos[0];
	  //std::cout << sizeof(shape) << std::endl;
		id=id&0xff;
		out_message.type = NORMAL;
		out_message.data[0] = id;
		out_message.data[1] = 0;
		out_message.data[2] = 0;
		out_message.crc = message_crc(&out_message);
		set_color(RGB(0,0,1));
	}

	//executed on successfull message send
	void message_tx_success() {
		//set_color(RGB(1,0,0));
		msrx=1;
	}

	//sends message at fixed rate
	message_t *message_tx() {
		static int count = rand();
		count--;
		if (!(count % 50)) {
			return &out_message;
		}
		return NULL;
	}

	//receives message
	void message_rx(message_t *message, distance_measurement_t *distance_measurement) {
		distance = estimate_distance(distance_measurement);
		out_message.data[0] = message->data[0];
		out_message.data[1] = message->data[1];
		out_message.data[2] = message->data[2];
		rxed=1;
	}
};
