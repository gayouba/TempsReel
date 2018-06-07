#include "../header/functions.h"

char mode_start;

void write_in_queue(RT_QUEUE *, MessageToMon);
void checkCompteur();

void f_server(void *arg) {
    int err;
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    err = run_nodejs("/usr/local/bin/node", "/home/pi/Interface_Robot/server.js");

    if (err < 0) {
        printf("Failed to start nodejs: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    } else {
#ifdef _WITH_TRACE_
        printf("%s: nodejs started\n", info.name);
#endif
        open_server();
        rt_sem_broadcast(&sem_serverOk);
    }
}

void f_sendToMon(void * arg) {
    int err;
    MessageToMon msg;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    while (1) {

#ifdef _WITH_TRACE_
        printf("%s : waiting for a message in queue\n", info.name);
#endif
        if (rt_queue_read(&q_messageToMon, &msg, sizeof (MessageToRobot), TM_INFINITE) >= 0) {
#ifdef _WITH_TRACE_
            printf("%s : message {%s,%s} in queue\n", info.name, msg.header, msg.data);
#endif

            err = send_message_to_monitor(msg.header, msg.data);
            free_msgToMon_data(&msg);
            rt_queue_free(&q_messageToMon, &msg);
            if (err < 0)
                rt_sem_p(&sem_comLost, TM_INFINITE);
        } else {
            printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void f_receiveFromMon(void *arg) {
    MessageFromMon msg;
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    do {
#ifdef _WITH_TRACE_
        printf("%s : waiting for a message from monitor\n", info.name);
#endif
        err = receive_message_from_monitor(msg.header, msg.data);
#ifdef _WITH_TRACE_
        printf("%s: msg {header:%s,data=%s} received from UI\n", info.name, msg.header, msg.data);
#endif
        if (strcmp(msg.header, HEADER_MTS_CAMERA) == 0) {
            if (msg.data[0] == CAM_OPEN) { // Camera related data
                rt_sem_v(&sem_startCam);
            } else if (msg.data[0] == CAM_CLOSE) {
                closeCam = 1;
            } else if (msg.data[0] == CAM_ASK_ARENA) {
                askArena = 1;
            } else if (msg.data[0] == CAM_ARENA_CONFIRM) {
                arenaValid = 1;
                rt_sem_v(&sem_arenaValid);
            } else if (msg.data[0] == CAM_ARENA_INFIRM) {
                arenaValid = 0;
                rt_sem_v(&sem_arenaValid);
            } else if (msg.data[0] == CAM_COMPUTE_POSITION) {
                computePos = 1;
            } else if (msg.data[0] == CAM_STOP_COMPUTE_POSITION) {
                computePos = 0;
            }
        } else if (strcmp(msg.header, HEADER_MTS_COM_DMB) == 0) {
            if (msg.data[0] == OPEN_COM_DMB) { // Open communication supervisor-robot
#ifdef _WITH_TRACE_
                printf("%s: message open Xbee communication\n", info.name);
#endif
                rt_sem_v(&sem_openComRobot);
            }
        } else if (strcmp(msg.header, HEADER_MTS_DMB_ORDER) == 0) {
            if (msg.data[0] == DMB_START_WITHOUT_WD) { // Start robot
#ifdef _WITH_TRACE_
                printf("%s: message start robot\n", info.name);
#endif 
                rt_sem_v(&sem_startRobot);

            } else if ((msg.data[0] == DMB_GO_BACK)
                    || (msg.data[0] == DMB_GO_FORWARD)
                    || (msg.data[0] == DMB_GO_LEFT)
                    || (msg.data[0] == DMB_GO_RIGHT)
                    || (msg.data[0] == DMB_STOP_MOVE)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msg.data[0];
                rt_mutex_release(&mutex_move);
#ifdef _WITH_TRACE_
                printf("%s: message update movement with %c\n", info.name, move);
#endif

            }
        }
    } while (err >= 0);

}

void f_openComRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openComRobot arrived => open communication robot\n", info.name);
#endif
        err = open_communication_robot();
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the communication is opened\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_startRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startRobot\n", info.name);
#endif
        rt_sem_p(&sem_startRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_startRobot arrived => Start robot\n", info.name);
#endif
        err = send_command_to_robot(DMB_START_WITHOUT_WD);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the robot is started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_move(void *arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
        printf("%s: move equals %c\n", info.name, move);
#endif
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
#ifdef _WITH_TRACE_
            printf("%s : error send command to robot\n", info.name);
#endif
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            err = send_command_to_robot(move);
            if (err < 0) {
                rt_mutex_acquire(&mutex_compteur, TM_INFINITE);
                compteur++;
                rt_mutex_release(&mutex_compteur);
            } else {
                rt_mutex_acquire(&mutex_compteur, TM_INFINITE);
                compteur = 0;
                rt_mutex_release(&mutex_compteur);
            }
            checkCompteur();
            rt_mutex_release(&mutex_move);
#ifdef _WITH_TRACE_
            printf("%s: the movement %c was sent\n", info.name, move);
#endif            
        }
        rt_mutex_release(&mutex_robotStarted);
    }
}

void f_cam(void *arg) {

    /*DECLARATIONS*/
    Camera rpiCam;
    Image imgVideo;
    Position positionRobots[20];
    Arene * monArene;
    Jpg compress;
    int err;
    MessageToMon msgPos;
    MessageToMon msgImg;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while(1){

#ifdef _WITH_TRACE_
        printf("%s : Wait sem_cam\n", info.name);
#endif
        rt_sem_p(&sem_startCam, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_cam arrived => open camera\n", info.name);
#endif

        err= open_camera(&rpiCam);

        if (err==0) {
#ifdef _WITH_TRACE_
            printf("%s : the camera is started\n", info.name);
#endif
            send_message_to_monitor(HEADER_STM_ACK,(void*)"");
            /* PERIODIC START */
#ifdef _WITH_TRACE_
            printf("%s: start period\n", info.name);
#endif
            rt_task_set_periodic(NULL, TM_NOW, 100000000);
            while (1) {
#ifdef _WITH_TRACE_
                printf("%s: Wait period \n", info.name);
#endif
                if (!closeCam){
                    printf("%s: cam not close \n", info.name);
                    get_image(&rpiCam, &imgVideo);
                    if (computePos==1){
                        detect_position(&imgVideo,positionRobots,monArene);
                        draw_position(&imgVideo, &imgVideo, &positionRobots[0]);
                        send_message_to_monitor(HEADER_STM_POS,&(positionRobots[0]));
                    }
                    if (askArena==1){
                        if (!monArene){
                            monArene=new Arene();
                        }
                        if(detect_arena(&imgVideo, monArene)==0){
                            draw_arena(&imgVideo,&imgVideo,monArene);
                            compress_image(&imgVideo,&compress);
                            send_message_to_monitor(HEADER_STM_IMAGE,&compress);
                            send_message_to_monitor(HEADER_STM_POS,&(positionRobots[0]));
                            rt_sem_p(&sem_arenaValid, TM_INFINITE);
                            if (!arenaValid){
                                delete monArene;
                                monArene=NULL;						
                            }

                        }else{
#ifdef _WITH_TRACE_
                            printf("%s: Arena detection failed \n", info.name);
#endif
                        }
                        askArena=0;
                    } else {
                        if(monArene){
                            draw_arena(&imgVideo,&imgVideo,monArene);
                        }
                        compress_image(&imgVideo,&compress);
                        send_message_to_monitor(HEADER_STM_IMAGE,&compress);
                        send_message_to_monitor(HEADER_STM_POS,&(positionRobots[0]));
                    }
                    rt_task_wait_period(NULL);
                }
                else {
                    send_message_to_monitor(HEADER_STM_ACK,(void*)"");
                    close_camera(&rpiCam);
                    closeCam=0;
                    break;
                }
            }
        } else {
#ifdef _WITH_TRACE_
            printf("%s: failed to open camera \n", info.name);
#endif
            send_message_to_monitor(HEADER_STM_NO_ACK,(void*)"");
        }
    }
}

void f_battery(void *arg) {

    int bat;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif
    rt_task_set_periodic(NULL, TM_NOW, 500000000);
    while (1) {
#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
#endif
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
            bat = send_command_to_robot(DMB_GET_VBAT);
            if (bat < 0) {
#ifdef _WITH_TRACE_
                printf("%s : error send command to robot\n", info.name);
#endif
                rt_mutex_acquire(&mutex_compteur, TM_INFINITE);
                compteur++;
                rt_mutex_release(&mutex_compteur);
            } else {
                rt_mutex_acquire(&mutex_compteur, TM_INFINITE);
                compteur = 0;
                rt_mutex_release(&mutex_compteur);
                MessageToMon msg;
                bat+=48;
                send_message_to_monitor(HEADER_STM_BAT,&bat);
            }
            checkCompteur();
#ifdef _WITH_TRACE_
            printf("%s: the battery level was asked\n", info.name);
#endif
        }
        rt_mutex_release(&mutex_robotStarted);
    }
}

void f_close(void *arg) {

    int err; 

    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_comLost\n", info.name);
#endif
        rt_sem_p(&sem_comLost, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_comLost arrived => Closing\n", info.name);
#endif

        err = send_command_to_robot(CAM_CLOSE);
        err = send_command_to_robot(DMB_STOP_MOVE);

        err = close_communication_robot();
        err = kill_nodejs();
        err = close_server();

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);
    }
}

void write_in_queue(RT_QUEUE *queue, MessageToMon msg) {
    void *buff;
    buff = rt_queue_alloc(&q_messageToMon, sizeof (MessageToMon));
    memcpy(buff, &msg, sizeof (MessageToMon));
    rt_queue_send(&q_messageToMon, buff, sizeof (MessageToMon), Q_NORMAL);
}

void checkCompteur() {
    rt_mutex_acquire(&mutex_compteur, TM_INFINITE);
    if(compteur >= 3) {
        MessageToMon msg;

        printf("Perte de communication robot-superviseur\n");
        send_message_to_monitor(HEADER_STM_ACK,(void*) "");

        close_communication_robot();
    }
    rt_mutex_release(&mutex_compteur);
}

