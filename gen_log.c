//													  //
//	Generates a .csv file with BOX - CONTAINER pairs  //
//  which is used by the ROSS model as input		  //
//													  //

#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#define LOG_LENGTH 10000

#define N_BOXES        1
#define N_CONTAINERS   3

const char* log_path = "log.csv";
int weight[N_CONTAINERS - 1];

int main()
{	
	weight[0] = 80;
	weight[1] = 95;

	FILE* f = fopen(log_path, "w");
	srand(time(NULL));
	for (int y = 0; y < LOG_LENGTH; ++y)
	{
		int DEST = -1;
		int RNG = rand() % 100;

		if 		(RNG < weight[0])
			DEST = 1;
		else if (RNG < weight[1])
			DEST = 2;
		else
			DEST = 3;

        fprintf(f, "%d,%d\n", (rand() % N_BOXES) + 1, (rand() % N_CONTAINERS) + 1);
	}
	fclose(f);
}

