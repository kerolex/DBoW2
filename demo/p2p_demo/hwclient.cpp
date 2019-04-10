/*
 * hwclient.c
 *
 *  Created on: 4 Jul 2016
 *      Author: alessio_x
 */

//  Hello World client
#include <zmq.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

// OpenCV Libraries
#include "opencv2/opencv.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

int main (int argc, char **argv[]) 
{
    if(argc != 2)
        return -1;

    float T = 25; // Temperature value
    
    printf ("Connecting to hello world server…\n");
    void *context = zmq_ctx_new ();
    void *requester = zmq_socket (context, ZMQ_REQ);
    zmq_connect (requester, "tcp://localhost:5555");
    
    
    //VideoCapture cam1(fVideo);
	//if (!cam1.isOpened())
	//	return -1;

    // Number of Frames
	//int nFrames = cam1.get(CV_CAP_PROP_FRAME_COUNT);

    float T_old;
    float T_tmp;

    int request_nbr;
    for (request_nbr = 0; request_nbr != 100; request_nbr++) {
        T_old = T;
        
        char buffer [10];
        printf ("Client: Sending Hello %d…\n", request_nbr);
        zmq_send (requester, "Hello", 5, 0);
        zmq_recv (requester, buffer, 10, 0);

        t_tmp = atoi(buffer);
        
        printf ("Client: Received World %d\n", request_nbr);

        T = T_old + 0.1 * (T_old - t_tmp);
    }

    zmq_close (requester);
    zmq_ctx_destroy (context);

    return 0;
}
