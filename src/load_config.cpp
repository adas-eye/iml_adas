#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h"

#include "load_config.h"

using namespace std;

void remove_white_space_comment(char* str)
{
	char temp[2048];
	char* p = str;
	int i = 0;
	while (*p) {
		if (!isspace(*p)) {
			temp[i++] = *p;
		}
		++p;
	}
	temp[i] = '\0';

	if ((p = strchr(temp, '#')) != NULL) {
		*p = '\0';
	}
	strcpy(str, temp);
}

char* get_filename_from_str(char* str)
{
    char* p = NULL;
    if ((p = strchr(str, '=')) != NULL) {
        char* filename = (char*)malloc(sizeof(char) * strlen(p));
        if (filename == NULL) {
            perror("get_filename_from_str:");
            exit(1);
        }
        strcpy(filename, strchr(str, '=') + 1);
        return filename;
    }
    return NULL;
}

void read_from_file(const char* file, const char* file2, layerConfig_t* &layerConf, int &layerNum)
{
	int num = 0;
	char str[1024];
	FILE *fp = fopen(file, "r");
	if (fp == NULL) {
		fp = fopen(file2, "r");
		if (fp == NULL) {
			printf("can't read layer config file: %s\n", file);
			return;
		}
	}
	while (fgets(str, 1024, fp)) {
		remove_white_space_comment(str);

		if ('\0' == str[0]) {
			continue;
		}

		if (strncmp(str, "layerNum", strlen("layerNum")) == 0) {
			if (strchr(str, '=') != NULL) {
				sscanf(strchr(str, '=') + 1, "%d", &num);
			}
			if (num > 0 && num < 100) {
				layerConf = (layerConfig_t*)malloc(num * sizeof(layerConfig_t));
			} else {
				printf("layer num not support!\n");
				exit(0);
			}
			continue;
		}

		if (strncmp(str, "[baselayer]", strlen("[baselayer]")) == 0) {
			layerNum++;
			continue;
		}

		if (layerNum == 0) {
			continue;
		}

		if (strncmp(str, "layerId", strlen("layerId")) == 0) {
			if (strchr(str, '=') != NULL) {
				sscanf(strchr(str, '=') + 1, "%d", &num);
				layerConf[layerNum-1].layerId = num;
			}
			continue;
		}

		if (strncmp(str, "inputs", strlen("inputs")) == 0) {
			if (strchr(str, '=') != NULL) {
				sscanf(strchr(str, '=') + 1, "%d", &num);
				layerConf[layerNum-1].inputs = num;
			}
			continue;
		}

		if (strncmp(str, "modelType", strlen("modelType")) == 0) {
			if (strchr(str, '=') != NULL) {
				sscanf(strchr(str, '=') + 1, "%d", &num);
				layerConf[layerNum-1].modelType = num;
			}
			continue;
		}

		if (strncmp(str, "modelFile", strlen("modelFile")) == 0) {
			layerConf[layerNum-1].modelFile = get_filename_from_str(str);
			continue;
		}
	}
}

int check_config(layerConfig_t* layerConf, int layerNum)
{
	return 0;
}

