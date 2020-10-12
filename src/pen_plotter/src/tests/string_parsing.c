#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#define pi 3.14159

void parse_gcode_line(char gcode_line[],char out[][30],int *num_words){
    int init_size = strlen(gcode_line);
	char delim[] = " ";

	char *ptr = strtok(gcode_line, delim);
    int idx = 0;    

	while(ptr != NULL)
	{
		// printf("'%s'\n", ptr);
        strcpy(out[idx],ptr);
		ptr = strtok(NULL, delim);
        idx++;
	}
    *num_words = idx;
}

int main()
{
	char gcode_line[] = "G1 X82.472 Y74.787 D0 ;comment with muliple lines";
    int num_words;
    char commands[10][30];
    printf("\nGCode Line: %s\n",gcode_line);
    printf("Parsed commands:\n");
    parse_gcode_line(gcode_line,commands,&num_words); 
    
    int comment = 0;
    printf("\t");
    for (int i=0; i < num_words; i++){
        if(commands[i][0]==';'){
            comment = 1;
        }
        if (comment == 1){
            printf("%s ",commands[i]);
        }
        else{
            printf("%s\n\t",commands[i]);
        }
    }
    if (comment==1){
        printf("\n");
    }
    printf("\n");
	return 0;
}