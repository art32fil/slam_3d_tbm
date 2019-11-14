#include "tbm3.h"
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "common_features.h"

using namespace std;

TBM3::mass_t get_rand() {
	return TBM3::mass_t(double(rand()%1021)/1021.0);
}

TBM3 build_correct_cell() {
	TBM3::mass_t sum = TBM3::ZERO;
	TBM3::mass_t a   = get_rand();
	TBM3::mass_t b   = get_rand();
	TBM3::mass_t c   = get_rand();
	TBM3::mass_t ab  = get_rand();
	TBM3::mass_t ac  = get_rand();
	TBM3::mass_t bc  = get_rand();
	TBM3::mass_t abc = get_rand();
	do {
		a   = get_rand();
		b   = get_rand();
		c   = get_rand();
		ab  = get_rand();
		ac  = get_rand();
		bc  = get_rand();
		abc = get_rand();
		sum = a+b+c+ab+ac+bc+abc;
	} while (sum >= 1);
	return TBM3(a,b,c,ab,ac,bc,abc);
}

int main(int argc, char** argv) {
	srand(time(0));
	cout << "**************" << endl
	     << "*TBM BUILDING*" << endl
	     << "**************" << endl;
	cout << "1) building correct cell:\n" ;
	for (int i = 0; i < 5; i++) {
		cout << "---------------------------------" << endl;
		cout << "cell " << i << endl << build_correct_cell() << endl;
	}
}
