#include "model.h"

//The C extras file for a ROSS model
//This file includes:
// - all functions that are not from ROSS

#define MAX_PATH_TO_LOG_FOLDER 256

size_t step_number = 0;
extern int glb_time;
extern bool NewWeights;

extern const char* path_to_log_folder;
extern const char* path_to_room_file;
extern const char* path_to_robots_file;
extern const char* path_to_pairs;
extern const char* path_to_log_file;
extern FILE* f;

void PrintMap(const char* log_folder_path)
{   
    ++step_number;
    char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/STEP_%lu.csv", log_folder_path, step_number);
    //printf("path = %s\n", path);

    FILE* f = fopen(path, "w");
    for (int y = 0; y < storage.height; ++y) {
        for (int x = 0; x < storage.length; ++x) {
            fprintf(f, "%d", storage.robots[y][x]);
            if (x < storage.length - 1)
				fprintf(f, ","); //no comma at the end of a line
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

void PrintPairs()
{
	for (int y = 0; y < pairs.length; ++y)
	{
        for (int x = 0; x < 2; ++x)
		{
            printf("%d", pairs.data[y][x]);
            if (x == 0)
				printf(","); //no comma at the end of a line
        }
        printf("\n");
    }
}

void PrintNSteps(const char* log_folder_path)
{
    char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/_N_STEPS.txt", log_folder_path);  
    FILE* f = fopen(path, "w");
    fprintf(f, "%lu", step_number);
    fclose(f);
}

//Converts a CSV file to a 2D int array
void Parse(const char* path, int arr[MAX_ROOM_HEIGHT][MAX_ROOM_LENGTH])
{
    struct stat stats;
    stat(path, &stats);
    FILE* f = fopen(path, "r");
    assert(f); //to check correctness of the path
    char buf[MAX_ROOM_HEIGHT * MAX_ROOM_LENGTH * 2 + MAX_COMMENT_LENGTH]; // * 2 for commas
    fread(buf, sizeof(buf[0]), stats.st_size, f);
    fclose(f);

    int start_index = 0;
    if (buf[0] == '#') {
        for (start_index = 0; buf[start_index] != '\n' ; ++start_index)
            ;
        ++start_index; //next symbol after #bla-bla-bla\n
    }
    
    storage.height = 0;
    int x = 0;
    for (int i = start_index; i < stats.st_size; ++i) {
        if (buf[i] == '\n') {
            ++storage.height;
            storage.length = x;
            x = 0;
            continue;
        }
        if (isdigit(buf[i])) {
            if ((i < stats.st_size) && isdigit(buf[i+1])) {  //processing 2-digit numbers
                arr[storage.height][x] = (buf[i] - '0') * 10 + (buf[i+1] - '0');
                ++i;
            }
            else {
                arr[storage.height][x] = buf[i] - '0';
            }
            ++x;
        }
    }
}

void PairsInit(const char* path)
{
	pairs.data = calloc(MAX_INPUT_LENGTH, sizeof(int*));
	for (size_t i = 0; i < MAX_INPUT_LENGTH; ++i)
		pairs.data[i] = calloc(2, sizeof(int));

    struct stat stats;
    stat(path, &stats);
    FILE* f = fopen(path, "r");
    assert(f); //to check correctness of the path
	assert(stats.st_size < MAX_INPUT_LENGTH * 4);
    char buf[MAX_INPUT_LENGTH * 4]; //'1' ',' '2' '\n' -> 4 columns
    fread(buf, sizeof(buf[0]), stats.st_size, f);
    fclose(f);

	pairs.length = 0;
    int x = 0;
    for (int i = 0; i < stats.st_size; ++i)
	{
        if (buf[i] == '\n')
		{
            ++pairs.length;
            x = 0;
            continue;
        }
        if (isdigit(buf[i]))
		{
            pairs.data[pairs.length][x] = buf[i] - '0';
            ++x;
        }
    }
	pairs.cur = 0;
	pairs.eof = FALSE;
}

void FreePairs()
{	
	for (size_t i = 0; i < MAX_INPUT_LENGTH; ++i)
		free(pairs.data[i]);
	free(pairs.data);
}

void RobotsInit()
{
    Robots.N = 0;
    for (int y = 0; y < storage.height; ++y)
        for (int x = 0; x < storage.length; ++x)
            if (storage.robots[y][x] == CELL_ROBOT   		|| \
                storage.robots[y][x] == CELL_ROBOT_WITH_BOX) 
			{
				if (storage.robots[y][x] == CELL_ROBOT)
					Robots.data[Robots.N].carries_box = FALSE;
				else
					Robots.data[Robots.N].carries_box = TRUE;
				
                Robots.data[Robots.N].x = x;
                Robots.data[Robots.N].y = y;
                
                ++Robots.N;
                assert(Robots.N <= MAX_ROBOTS);
            }
	assert(Robots.N);
}

void RobotsPrint()
{
    for (int i = 0; i < Robots.N; ++i)
        printf("Robot #%d is located at (%d, %d)\n", \
                      i+1, Robots.data[i].x, Robots.data[i].y);
}

void SendMessageContents(tw_lpid receiver, tw_lp* lp, double ts, lp_type type, double contents)
{
    tw_event* Event   = tw_event_new(receiver, ts, lp);
    message* Message  = tw_event_data(Event);
    Message->type     = type;
    Message->contents = contents;
    Message->sender   = lp->gid;
    tw_event_send(Event);
}

void SendMessage(tw_lpid receiver, tw_lp* lp, double ts, lp_type type)
{
    SendMessageContents(receiver, lp, ts, type, 0);
}

bool EveryoneResponded(int* arr, int N)
{
	int cnt = 0;
	for (int i = 0; i < N; ++i)
		if(arr[i])
			++cnt;
	return cnt == N;
}

void AssignDest(struct _robot* robot, int goal)
{
	if 		(goal == CELL_BOX)
	{
		GetPair(robot);
		struct cell dest_cell = {4, 7, CELL_BOX};
		robot->dest = dest_cell;
		return;
	}
	
	else if (goal == CELL_CONTAINER)
	{
		switch(robot->pair[1])
		{
			case 1:
				{
				struct cell dest_cell = {1, 0, CELL_CONTAINER};
				robot->dest = dest_cell;
				}
				return;
			case 2:
				{
				struct cell dest_cell = {4, 0, CELL_CONTAINER};
				robot->dest = dest_cell;
				}
				return;
			case 3:
				{
				struct cell dest_cell = {7, 0, CELL_CONTAINER};
				robot->dest = dest_cell;
				}
				return;
		}
	}	
}

message_type CalcNextMove(struct _robot* robot)
{
	
	if (robot->time_in_action > 1)
	{
		robot->time_in_action -= 1;
		return NOP;
	}
	
	int x = robot->x;
	int y = robot->y;
	
	if (storage.room[robot->y][robot->x] == robot->dest.value && \
						  robot->dest.value == CELL_BOX       && !robot->carries_box)
	{
		UpdateWeights(robot);
		return BOX_GRAB;
	}
	
	if (storage.room[robot->y][robot->x] == robot->dest.value && \
						  robot->dest.value == CELL_CONTAINER && robot->carries_box)
	{
		UpdateWeights(robot);
		return BOX_DROP;
	}
	
	bool u_blocked = (robot->y == 0                || storage.room[y-1][x] == CELL_WALL);
	bool d_blocked = (robot->y == storage.height-1 || storage.room[y+1][x] == CELL_WALL);
	bool l_blocked = (robot->x == 0                || storage.room[y][x-1] == CELL_WALL);
	bool r_blocked = (robot->x == storage.length-1 || storage.room[y][x+1] == CELL_WALL);
	
	int u_weight = u_blocked? 0: (storage.robots[y-1][x] != CELL_EMPTY || robot->visited[y-1][x])? (AntWeights[y-1][x] / 10): AntWeights[y-1][x];
	int d_weight = d_blocked? 0: (storage.robots[y+1][x] != CELL_EMPTY || robot->visited[y+1][x])? (AntWeights[y+1][x] / 10): AntWeights[y+1][x];
	int l_weight = l_blocked? 0: (storage.robots[y][x-1] != CELL_EMPTY || robot->visited[y][x-1])? (AntWeights[y][x-1] / 10): AntWeights[y][x-1];
	int r_weight = r_blocked? 0: (storage.robots[y][x+1] != CELL_EMPTY || robot->visited[y][x+1])? (AntWeights[y][x+1] / 10): AntWeights[y][x+1];
	
	srand(time(NULL));
	int rng = rand() % 3;
	if (rng < 0)
		return ChooseDir(u_weight, d_weight, l_weight, r_weight, robot);
	return RandChooseDir(u_weight, d_weight, l_weight, r_weight, robot);
}

void UpdateWeights(struct _robot* robot)
{
	for (int y = 0; y < storage.height; ++y)
		for (int x = 0; x < storage.length; ++x)
			if (robot->visited[y][x])
				AntWeights[y][x] += 100;
			
	NewWeights = TRUE;
}

message_type ChooseDir(int u, int d, int l, int r, struct _robot* robot)
{
	int x = robot->x;
	int y = robot->y;
	
	if (NeighborCellsBlocked(robot))
		for (int y = 0; y < storage.height; ++y)
			for (int x = 0; x < storage.length; ++x)
				robot->visited[y][x] = FALSE;
	
	if (u >= d && u >= l && u >= r)
	{
		//printf("TIME = %d: ROBOT(%d,%d): moving UP\n", glb_time, x, y);
		return MOVE_U;
	}
	if (d >= l && d >= r)
	{
		//printf("TIME = %d: ROBOT(%d,%d): moving DOWN\n", glb_time, x, y);
		return MOVE_D;
	}
	if (l >= r)
	{
		//printf("TIME = %d: ROBOT(%d,%d): moving LEFT\n", glb_time, x, y);
		return MOVE_L;
	}
	
	//printf("TIME = %d: ROBOT(%d,%d): moving RIGHT\n", glb_time, x, y);
	return MOVE_R;
}

message_type RandChooseDir(int u, int d, int l, int r, struct _robot* robot)
{
	int x = robot->x;
	int y = robot->y;
	
	if (NeighborCellsBlocked(robot))
		for (int y = 0; y < storage.height; ++y)
			for (int x = 0; x < storage.length; ++x)
				robot->visited[y][x] = FALSE;
	
	int sum = u + d + l + r;
	//printf("RandChooseDir(%d, %d, %d, %d) - [%d, %d]\n", u, d, l, r, x, y);
	srand(time(NULL));
	int rng = rand() % sum;
	if (rng < u)
		return MOVE_U;
	if (rng < u + d)
		return MOVE_D;
	if (rng < u + d + l)
		return MOVE_L;
	return MOVE_R;
}

bool NeighborCellsBlocked(struct _robot* robot)
{
	int x = robot->x;
	int y = robot->y;
	
	if (x == 0 && y == 0)
		return (robot->visited[y+1][x] || storage.room[y+1][x] == CELL_WALL || storage.robots[y+1][x] != CELL_EMPTY) && \
			   (robot->visited[y][x+1] || storage.room[y][x+1] == CELL_WALL || storage.robots[y][x+1] != CELL_EMPTY);
			   
	if (x == 0 && y == 8)
		return (robot->visited[y-1][x] || storage.room[y-1][x] == CELL_WALL || storage.robots[y-1][x] != CELL_EMPTY) && \
			   (robot->visited[y][x+1] || storage.room[y][x+1] == CELL_WALL || storage.robots[y][x+1] != CELL_EMPTY);
			   
	if (x == 8 && y == 0)
		return (robot->visited[y+1][x] || storage.room[y+1][x] == CELL_WALL || storage.robots[y+1][x] != CELL_EMPTY) && \
			   (robot->visited[y][x-1] || storage.room[y][x-1] == CELL_WALL || storage.robots[y][x-1] != CELL_EMPTY);
			   
	if (x == 8 && y == 8)
		return (robot->visited[y-1][x] || storage.room[y-1][x] == CELL_WALL || storage.robots[y-1][x] != CELL_EMPTY) && \
			   (robot->visited[y][x-1] || storage.room[y][x-1] == CELL_WALL || storage.robots[y][x-1] != CELL_EMPTY);
	
	if (x == 0) 
		return (robot->visited[y+1][x] || storage.room[y+1][x] == CELL_WALL || storage.robots[y+1][x] != CELL_EMPTY) && \
			   (robot->visited[y-1][x] || storage.room[y-1][x] == CELL_WALL || storage.robots[y-1][x] != CELL_EMPTY) && \
			   (robot->visited[y][x+1] || storage.room[y][x+1] == CELL_WALL || storage.robots[y][x+1] != CELL_EMPTY);
	
	if (x == 8)
		return (robot->visited[y+1][x] || storage.room[y+1][x] == CELL_WALL || storage.robots[y+1][x] != CELL_EMPTY) && \
			   (robot->visited[y-1][x] || storage.room[y-1][x] == CELL_WALL || storage.robots[y-1][x] != CELL_EMPTY) && \
			   (robot->visited[y][x-1] || storage.room[y][x-1] == CELL_WALL || storage.robots[y][x-1] != CELL_EMPTY);
			   
	if (y == 0)
		return (robot->visited[y+1][x] || storage.room[y+1][x] == CELL_WALL || storage.robots[y+1][x] != CELL_EMPTY) && \
			   (robot->visited[y][x-1] || storage.room[y][x-1] == CELL_WALL || storage.robots[y][x-1] != CELL_EMPTY) && \
			   (robot->visited[y][x+1] || storage.room[y][x+1] == CELL_WALL || storage.robots[y][x+1] != CELL_EMPTY);
			   
	if (y == 8)
		return (robot->visited[y-1][x] || storage.room[y-1][x] == CELL_WALL || storage.robots[y-1][x] != CELL_EMPTY) && \
			   (robot->visited[y][x-1] || storage.room[y][x-1] == CELL_WALL || storage.robots[y][x-1] != CELL_EMPTY) && \
			   (robot->visited[y][x+1] || storage.room[y][x+1] == CELL_WALL || storage.robots[y][x+1] != CELL_EMPTY);
	
	return (robot->visited[y-1][x] || storage.room[y-1][x] == CELL_WALL || storage.robots[y-1][x] != CELL_EMPTY) && \
		   (robot->visited[y+1][x] || storage.room[y+1][x] == CELL_WALL || storage.robots[y+1][x] != CELL_EMPTY) && \
		   (robot->visited[y][x-1] || storage.room[y][x-1] == CELL_WALL || storage.robots[y][x-1] != CELL_EMPTY) && \
		   (robot->visited[y][x+1] || storage.room[y][x+1] == CELL_WALL || storage.robots[y][x+1] != CELL_EMPTY);
}

void GetPair(struct _robot* robot)
{	
	if (pairs.cur == pairs.length)
	{
		pairs.eof = TRUE;
		printf("GetPair(): End of LOG file...\n");
		return;
	}
	robot->pair[0] = pairs.data[pairs.cur][0];
	robot->pair[1] = pairs.data[pairs.cur][1];
	++pairs.cur;
}

void SimulateROSS(int argc, char* argv[])
{
	int i, num_lps_per_pe;
    tw_opt_add(model_opts);
    tw_init(&argc, &argv);
    num_lps_per_pe = Robots.N + 1; //n robots + command center
    tw_define_lps(num_lps_per_pe, sizeof(message));
    g_tw_lp_typemap = &model_typemap;
    for (int i = 0; i < g_tw_nlp; ++i)
        tw_lp_settype(i, &model_lps[0]);
    tw_run();
    tw_end();
	printf("\nFinal global time is %d days\n", glb_time / (9000 * 24));
}

void FilesInit()
{
	Parse(path_to_room_file, storage.room);
	Parse(path_to_robots_file, storage.robots);
	f = fopen(path_to_log_file, "a");
}

void InitROSS()
{
	FilesInit();
	PairsInit(path_to_pairs);
    RobotsInit();
	AntWeightsInit();
	
	RobotsPrint();
    PrintMap(path_to_log_folder);
	PrintWeights(path_to_log_folder);
}

void AntWeightsInit()
{
	srand(time(NULL));
	for (int y = 0; y < storage.height; ++y)
        for (int x = 0; x < storage.length; ++x)
			AntWeights[y][x] = rand() % 1000;
}

void Free()
{
	FreePairs();
	fclose(f);
}

void FinalizeROSS()
{
	PrintNSteps(path_to_log_folder);
	PrintWeights(path_to_log_folder);
	Free();
}

void PrintWeights(const char* log_folder_path)
{   
    char path[MAX_PATH_TO_LOG_FOLDER];
    sprintf(path, "%s/WEIGHTS_STEP_%lu.csv", log_folder_path, step_number);

    FILE* f = fopen(path, "w");

    for (int y = 0; y < storage.height; ++y) {
        for (int x = 0; x < storage.length; ++x) {
            fprintf(f, "%d", AntWeights[y][x]);
            if (x < storage.length - 1)
				fprintf(f, ","); //no comma at the end of a line
        }
        fprintf(f, "\n");
    }
  
	fclose(f);

}

void LOG(int id, message_type Event)
{
	/*
	fprintf(f, "TIME: %8d | ROBOT: %d | EVENT: ", glb_time, id);
	switch(Event)
	{
		case MOVE_U:
			fprintf(f, "MOVE_U   | ");
			break;
		case MOVE_D:
			fprintf(f, "MOVE_D   | ");
			break;
		case MOVE_L:
			fprintf(f, "MOVE_L   | ");
			break;
		case MOVE_R:
			fprintf(f, "MOVE_R   | ");
			break;
		case BOX_GRAB:
			fprintf(f, "BOX_GRAB | ");
			break;
		case BOX_DROP:
			fprintf(f, "BOX_DROP | ");
			break;
		case INIT:
			fprintf(f, "INIT     | ");
			break;
		case NOP:
			fprintf(f, "NOP      | ");
			break;
		default:
			fprintf(f, "UNKNOWN  | ");
			break;
	}
	fprintf(f, "COORDS: (%d, %d)\n", Robots.data[id-1].x, Robots.data[id-1].y);
	*/
	fprintf(f, "%d,%d,%d,%d,%d\n", glb_time, id, Event, Robots.data[id-1].x, Robots.data[id-1].y);
}