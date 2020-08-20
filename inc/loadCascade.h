#ifndef _FACECASCADE_H_
#define _FACECASCADE_H_

const int cascade_num = 3;
extern int *HaarClassifierCascade_face_vec[];
extern int HaarClassifierCascade_face[];
extern int HaarClassifierCascade_face1[];
extern int HaarClassifierCascade_face2[];

extern int *StageClassifier_face_vec[];
extern int StageClassifier_face[];
extern int StageClassifier_face1[];
extern int StageClassifier_face2[];

extern int *Classifier_face1_vec[];
extern int Classifier_face[];
extern int Classifier_face1[];
extern int Classifier_face2[];

extern int *class_info_vec[];
extern int class_info[];
extern int class_info1[];
extern int class_info2[];

#endif