#include<iostream>
using namespace std;

typedef struct layerConfig {
	int layerId;
	int inputs;
	int modelType;
	char *modelFile;
} layerConfig_t;

extern void remove_white_space_comment(char* str);
extern char* get_filename_from_str(char* str);
extern int check_config(layerConfig_t* layerConf, int layerNum);
extern void read_from_file(const char* file, const char* file2, layerConfig_t* &layerConf, int &layerNum);
