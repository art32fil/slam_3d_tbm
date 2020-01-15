#include "tbm3.h"
#include <iostream>
#include <time.h>
#include "common_features.h"
#include <random>
#include <iomanip>
#include <string.h>
#include <string>
#include <cstring>

using namespace std;

TBM3::mass_t get_rand() {
	static random_device r;
	static mt19937 mt(r());
	static uniform_real_distribution<double> urd(0,1);
	return urd(mt);
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
	if (argc < 2) {
		cout << "Usage:\n\t" << argv[0] << " <print everything mode (0 or 1)>\n";
		return 0;
	}
	else {
		if (strcmp(argv[1],"true") == 0 or
		    strcmp(argv[1],"1") == 0 or
		    strcmp(argv[1],"True") == 0) {
			print_all_info = true;
		}
		else
			print_all_info = false;
	}
	cout << "**************" << endl
	     << "*TBM BUILDING*" << endl
	     << "**************" << endl;
	cout << "1) building correct cell:\n" ;
	int N = 5;
	TBM3 arr[N];
	for (int i = 0; i < N; i++) {
		cout << "---------------------------------" << endl;
		cout << "cell " << i << endl;
		cout << (arr[i] = build_correct_cell()) << endl;
	}
	cout << "2) merging cells:" << endl;
	for (int i = 0; i < N; i++) {
		for (int j = i+1; j < N; j++) {
			cout << " ---------------------------------" << endl;
			cout << "  cell " << i << " " << arr[i] << endl
			     << "+ cell " << j << " " << arr[j] << endl
			     << "=        " << arr[i] + arr[j] << endl
			     ;
		}
	}
}
