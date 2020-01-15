#include <cmath>
bool print_all_info = 0;

bool approximatelyEqual(float a, float b, float epsilon)
{
	float greater = fabs(a) < fabs(b) ? fabs(b) : fabs(a);
    return fabs(a - b) <=   greater * epsilon;
}

bool essentiallyEqual(float a, float b, float epsilon)
{
	float lower = fabs(a) > fabs(b) ? fabs(b) : fabs(a);
    return fabs(a - b) <=  lower * epsilon;
}

bool definitelyGreaterThan(float a, float b, float epsilon)
{
	float greater = fabs(a) < fabs(b) ? fabs(b) : fabs(a);
    return (a - b) > greater * epsilon;
}

bool definitelyLessThan(float a, float b, float epsilon)
{
	float greater = fabs(a) < fabs(b) ? fabs(b) : fabs(a);
    return (b - a) > greater * epsilon;
}
