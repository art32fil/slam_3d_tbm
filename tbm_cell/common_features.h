#ifndef COMMON_FEATURES_H_
#define COMMON_FEATURES_H_
#include <cmath>

extern bool print_all_info;

bool approximatelyEqual(float a, float b, float epsilon);
bool essentiallyEqual(float a, float b, float epsilon);
bool definitelyGreaterThan(float a, float b, float epsilon);
bool definitelyLessThan(float a, float b, float epsilon);


#endif /* COMMON_FEATURES_H_ */
