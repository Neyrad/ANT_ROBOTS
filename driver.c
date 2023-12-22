//The C driver file for a ROSS model
//This file includes:
// - an initialization function for each LP type
// - a forward event function for each LP type
// - a reverse event function for each LP type
// - a finalization function for each LP type

//Includes
#include <stdio.h>

#include "ross.h"
#include "model.h"

//Helper Functions
void SWAP(double* a, double* b)
{
    double tmp = *a;
    *a = *b;
    *b = tmp;
}

unsigned int setting_1 = 0;

const tw_optdef model_opts[] = {
    TWOPT_GROUP("ROSS Model"),
    TWOPT_UINT("setting_1", setting_1, "first setting for this model"),
    TWOPT_END(),
};

tw_lptype model_lps[] = {
  {
    (init_f) model_init,
    (pre_run_f) NULL,
    (event_f) model_event,
    (revent_f) model_event_reverse,
    (commit_f) NULL,
    (final_f) model_final,
    (map_f) model_map,
    sizeof(state)
  },
  { 0 },
};

extern const char* path_to_log_folder;

int glb_time = 0;
int RobotResponded[MAX_ROBOTS];
bool NewWeights = FALSE;
unsigned long long EventCounter = 0;

//Init function
// - called once for each LP
// ! LP can only send messages to itself during init !
void model_init(state* s, tw_lp* lp)
{
    if (lp->gid == 0)
    {
        s->type = COMMAND_CENTER;
		
		for (int i = 0; i < Robots.N; ++i)
			RobotResponded[i] = FALSE;
		
        printf("COMMAND_CENTER is initialized\n");
    }
    else
    {
        s->type = ROBOT;
        assert(lp->gid <= Robots.N);
		
		for (int y = 0; y < storage.height; ++y)
			for (int x = 0; x < storage.length; ++x)
				Robots.data[lp->gid - 1].visited[y][x] = FALSE;
	
		Robots.data[lp->gid - 1].state          	 		 = STOP;
		Robots.data[lp->gid - 1].time_in_action  			 = 0; //no commands received, no actions performed
		Robots.data[lp->gid - 1].boxes_delivered 			 = 0;
		
		AssignDest(&Robots.data[lp->gid - 1], CELL_BOX);
    
        printf("ROBOT #%ld is initialized\n", lp->gid);
    }

    int self = lp->gid;

    // init state data
    s->value = -1;

    s->got_msgs_MOVE_U   = 0;
	s->got_msgs_MOVE_D   = 0;
	s->got_msgs_MOVE_L   = 0;
	s->got_msgs_MOVE_R   = 0;
    s->got_msgs_BOX_GRAB = 0;
    s->got_msgs_BOX_DROP = 0;
    s->got_msgs_RECEIVED = 0;
    s->got_msgs_INIT     = 0;
	s->got_msgs_NOP      = 0;
	
    if (lp->gid == 0)
		for (int i = 1; i <= Robots.N; ++i)
			SendMessage(i, lp, glb_time, INIT);
}

//Forward event handler
void model_event(state* s, tw_bf* bf, message* in_msg, tw_lp* lp)
{
    int self = lp->gid;
    bool is_executed = FALSE;
    // initialize the bit field
    *(int*)bf = (int)0;

    // update the current state
    // however, save the old value in the 'reverse' message
    SWAP(&(s->value), &(in_msg->contents));

    // handle the message
    switch(s->type)
    {
        case COMMAND_CENTER:
            switch (in_msg->type)
            {
                case RECEIVED:
                    ++s->got_msgs_RECEIVED; 
                    break;
                default:
                    printf("COMMAND CENTER: Unhandled forward message type %d\n", in_msg->type);
            }

            if (pairs.eof || glb_time >= GLOBAL_TIME_END)
				return;

			RobotResponded[in_msg->sender-1] = TRUE;
			if (EveryoneResponded(RobotResponded, Robots.N))
			{	
				PrintMap(path_to_log_folder);
				PrintWeights(path_to_log_folder);
				glb_time += 1;
				
				for (int i = 0; i < Robots.N; ++i)
					RobotResponded[i] = FALSE;
					
				
				if (NewWeights)
				{
					for (int i = 0; i < Robots.N; ++i)
						for (int y = 0; y < storage.height; ++y)
							for (int x = 0; x < storage.length; ++x)
								Robots.data[i].visited[y][x] = FALSE;
					
					printf("model_event(): NewWeights => All Clean\n");
					NewWeights = FALSE;
				}
			
				
				for (int i = 1; i <= Robots.N; ++i)
				{
					message_type cmd = CalcNextMove(&Robots.data[i-1]);
					SendMessage(i, lp, glb_time, cmd);
				}
			}
            break;

        case ROBOT:
            {
            struct _robot* This = &Robots.data[self-1];
			switch (in_msg->type)
            {
                case MOVE_U:
                    ++s->got_msgs_MOVE_U;
					if (This->time_in_action > 1)
						break;
					This->time_in_action = MOVE_TIME;
					
					if (This->y == 0)
					{
						This->state = STOP;
						break;
					}
					
                    switch(storage.room[This->y - 1][This->x])
                    {
                        case CELL_EMPTY:
						case CELL_BOX:
						case CELL_CONTAINER:
							if (storage.robots[This->y - 1][This->x] == CELL_EMPTY)
							{
								storage.robots[This->y - 1][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX : CELL_ROBOT;
								storage.robots[This->y    ][This->x] = CELL_EMPTY;
								This->y = This->y - 1;
								
								This->visited[This->y][This->x] = TRUE;
								This->state = MOTION;
								
								LOG(self, MOVE_U);
							}
							else
								for (int y = 0; y < storage.height; ++y)
									for (int x = 0; x < storage.length; ++x)
										This->visited[y][x] = FALSE;
                            break;
                        case CELL_WALL:
							This->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_D:
                    ++s->got_msgs_MOVE_D;
					if (This->time_in_action > 1)
						break;
					This->time_in_action = MOVE_TIME;
					
					if (This->y == 8)
					{
						This->state = STOP;
						break;
					}
					
                    switch(storage.room[This->y + 1][This->x])
                    {
                        case CELL_EMPTY:
						case CELL_BOX:
						case CELL_CONTAINER:
							if (storage.robots[This->y + 1][This->x] == CELL_EMPTY)
							{
								storage.robots[This->y + 1][This->x] = This->carries_box ? CELL_ROBOT_WITH_BOX : CELL_ROBOT;
								storage.robots[This->y    ][This->x] = CELL_EMPTY;
								This->y = This->y + 1;
								
								This->visited[This->y][This->x] = TRUE;
								This->state = MOTION;
								
								LOG(self, MOVE_D);
							}
							else
								for (int y = 0; y < storage.height; ++y)
									for (int x = 0; x < storage.length; ++x)
										This->visited[y][x] = FALSE;
                            break;
                        case CELL_WALL:
							This->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_L:
                    ++s->got_msgs_MOVE_L;
					if (This->time_in_action > 1)
						break;
					This->time_in_action = MOVE_TIME;
					
					if (This->x == 0)
					{
						This->state = STOP;
						break;
					}
					
                    switch(storage.room[This->y][This->x - 1])
                    {
                        case CELL_EMPTY:
						case CELL_BOX:
						case CELL_CONTAINER:
							if (storage.robots[This->y][This->x - 1] == CELL_EMPTY)
							{
								storage.robots[This->y][This->x - 1] = This->carries_box ? CELL_ROBOT_WITH_BOX : CELL_ROBOT;
								storage.robots[This->y][This->x    ] = CELL_EMPTY;
								This->x = This->x - 1;
								
								This->visited[This->y][This->x] = TRUE;
								This->state = MOTION;
								
								LOG(self, MOVE_L);
							}
							else
								for (int y = 0; y < storage.height; ++y)
									for (int x = 0; x < storage.length; ++x)
										This->visited[y][x] = FALSE;
                            break;
                        case CELL_WALL:
							This->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
				case MOVE_R:
                    ++s->got_msgs_MOVE_R;
					if (This->time_in_action > 1)
						break;
					This->time_in_action = MOVE_TIME;
					
					if (This->x == 8)
					{
						This->state = STOP;
						break;
					}
					
                    switch(storage.room[This->y][This->x + 1])
                    {
                        case CELL_EMPTY:
						case CELL_BOX:
						case CELL_CONTAINER:
							if (storage.robots[This->y][This->x + 1] == CELL_EMPTY)
							{
								storage.robots[This->y][This->x + 1] = This->carries_box ? CELL_ROBOT_WITH_BOX : CELL_ROBOT;
								storage.robots[This->y][This->x]     = CELL_EMPTY;
								This->x = This->x + 1;
								
								This->visited[This->y][This->x] = TRUE;
								This->state = MOTION;
								
								LOG(self, MOVE_R);
							}
							else
								for (int y = 0; y < storage.height; ++y)
									for (int x = 0; x < storage.length; ++x)
										This->visited[y][x] = FALSE;
                            break;
                        case CELL_WALL:
							This->state = STOP;
							break;
                        default:
                            break;
                    }
                    break;
                case BOX_GRAB:
                    ++s->got_msgs_BOX_GRAB;
					if (This->time_in_action > 1)
						break;
					This->time_in_action = BOX_GRAB_TIME;
					This->state = STOP;
					
                    if (storage.room[This->y][This->x] == CELL_BOX && This->carries_box == FALSE)
                    {
                        This->carries_box = TRUE;
						storage.robots[This->y][This->x] = CELL_ROBOT_WITH_BOX;
                        //printf("ROBOT #%d: grab the box.\n", self);
						AssignDest(&Robots.data[self-1], CELL_CONTAINER);
						
						LOG(self, BOX_GRAB);
                    }
                    break;
                case BOX_DROP:
                    ++s->got_msgs_BOX_DROP;
					if (This->time_in_action > 1)
						break;
					This->time_in_action = BOX_DROP_TIME;
					This->state = STOP;
					
                    if (storage.room[This->y][This->x] == CELL_CONTAINER && This->carries_box == TRUE)
                    {
                        This->carries_box = FALSE;
						
						storage.robots[This->y][This->x] = CELL_ROBOT;
						//printf("ROBOT #%d: drop the box.\n", self);
						++This->boxes_delivered;						
						AssignDest(&Robots.data[self-1], CELL_BOX);
						
						LOG(self, BOX_DROP);
                    }
                    break;
				case INIT:
					++s->got_msgs_INIT;
					LOG(self, INIT);
					break;
				case NOP:
					++s->got_msgs_NOP;
					This->state = STOP;
					LOG(self, NOP);
					break;
                default:
                    printf("ROBOT #%d: Unhandled forward message of type %d\n", self, in_msg->type);
            }

            if (in_msg->sender == 0) //the message came from the command center
                SendMessage(0, lp, glb_time, RECEIVED);

            break;
        }
    } 
}

//Reverse Event Handler
void model_event_reverse(state* s, tw_bf* bf, message* in_msg, tw_lp* lp)
{
	return;
}

//report any final statistics for this LP
void model_final(state* s, tw_lp* lp)
{
    int self = lp->gid;
    if      (s->type == COMMAND_CENTER)
	{
        /*
		printf("COMMAND_CENTER:\n");
		printf("                got %4d messages of type RECEIVED\n",   	s->got_msgs_RECEIVED);
		*/
	}
	else if (s->type == ROBOT)
	{
		printf("\nROBOT #%d\n\n", self);

		/*
		printf("                got %8d messages of type ROTATE\n", 		s->got_msgs_ROTATE);
		printf("                got %8d messages of type MOVE_U\n",         s->got_msgs_MOVE_U);
		printf("                got %8d messages of type MOVE_D\n",         s->got_msgs_MOVE_D);
		printf("                got %8d messages of type MOVE_L\n",         s->got_msgs_MOVE_L);
		printf("                got %8d messages of type MOVE_R\n",         s->got_msgs_MOVE_R);
		printf("                got %8d messages of type BOX_GRAB\n",       s->got_msgs_BOX_GRAB);
		printf("                got %8d messages of type BOX_DROP\n",       s->got_msgs_BOX_DROP);
		printf("                got %8d messages of type INIT\n",           s->got_msgs_INIT);
		printf("                got %8d messages of type NOP\n",            s->got_msgs_NOP);
		*/
		
		printf("                delivered %8d boxes\n",			            Robots.data[self-1].boxes_delivered);
	}
}
