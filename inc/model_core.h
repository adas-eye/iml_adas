#ifndef MODEL_CORE_H_
#define MODEL_CORE_H_
#include <vector>

namespace score_namespace {
extern void* model_init(void *model_path);
extern int model_score(void *model_net, std::vector<float> data_vec, std::vector<float> &result_vec);
extern int model_free(void *model_net);
}

#endif  // MODEL_CORE_H_

