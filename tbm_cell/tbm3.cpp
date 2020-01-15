#include <iostream>
#include "tbm3.h"
#include "common_features.h"

const TBM3::mass_t TBM3::ZERO = TBM3::mass_t(0);
const TBM3::mass_t TBM3::ONE = TBM3::mass_t(1);

TBM3::mass_t epsilon = TBM3::mass_t(0.00001);

TBM3::TBM3(double a, double b, double c,
           double ab, double ac, double bc, double abc):
             a(a), b(b), c(c), ab(ab), ac(ac), bc(bc), abc(abc), status(false) {
	mass_t summ = a+b+c+ab+ac+bc+abc;

	// if (summ == 1)
	if (essentiallyEqual(summ, TBM3::ONE, epsilon)) {
		status = true;
	}
	// else if (summ < 1)
	else if (definitelyLessThan(summ, TBM3::ONE, epsilon)) {
		if (print_all_info)
			std::cout << "cell is not normalized!" << std::endl
			          << "summ of masses is less than 1" << std::endl
			          << *this << std::endl
			          << "normalized coefficient: " << summ<< std::endl;
		normalize();
		status = true;
	}
	//else if (summ > 1)
	else if (definitelyGreaterThan(summ, TBM3::ONE, epsilon)) {
		if (print_all_info)
			std::cout << "cell is not normalized!" << std::endl
			          << "summ of masses is greater than 1" << std::endl
			          << *this << std::endl
			          << "I don't know what to do!"<< std::endl;
		status = false;
	}
	else {
		if (print_all_info)
			std::cout << "undefined behaviour total sum is !=, !>, !< than 1"
			          << std::endl;
		status = false;
	}
}

void TBM3::normalize() {
	mass_t summ = a+b+c+ab+ac+bc+abc;
	// if (summ > 1)
	if (definitelyGreaterThan(summ, TBM3::ONE,epsilon)) {
		if (print_all_info)
			std::cout << "all masses are grater than one! for cell:\n"
			          << *this << std::endl;
		return;
	}
	a   /= summ; b  /= summ; c  /= summ;
	ab  /= summ; ac /= summ; bc /= summ;
	abc /= summ;
	return;
}

TBM3& TBM3::operator =(const TBM3& a) {
	this->a = a.a; b = a.b; c = a.c;
	ab = a.ab; ac = a.ac; bc = a.bc;
	abc = a.abc;

	status = a.status;
	return *this;
}

TBM3 operator +(const TBM3& a, const TBM3& b) {
	TBM3::mass_t mass_a = a.a*b.a +
	                      a.ab*b.a + a.ac*b.a + a.abc*b.a +
	                      a.a*b.ab + a.a*b.ac + a.a*b.abc +
	                      a.ab*b.ac + a.ac*b.ab;
	TBM3::mass_t mass_b = a.b*b.b +
	                      a.ab*b.b + a.bc*b.b + a.abc*b.b +
	                      a.b*b.ab + a.b*b.bc + a.b*b.abc +
	                      a.bc*b.ab + a.ab*b.bc;
	TBM3::mass_t mass_c = a.c*b.c +
	                      a.ac*b.c + a.bc*b.c + a.abc*b.c +
	                      a.c*b.ac + a.c*b.bc + a.c*b.abc +
	                      a.ac*b.bc + a.bc*b.ac;
	TBM3::mass_t mass_ab = a.ab*b.ab +
	                       a.abc*b.ab + a.ab*b.abc;
	TBM3::mass_t mass_ac = a.ac*b.ac +
	                       a.abc*b.ac + a.ac*b.abc;
	TBM3::mass_t mass_bc = a.bc*b.bc +
	                       a.abc*b.bc + a.bc*b.abc;
	TBM3::mass_t mass_abc = a.abc*b.abc;
	TBM3::mass_t mass_zero = a.a*b.b + a.a*b.c + a.a*b.bc +
	                         a.b*b.a + a.b*b.c + a.b*b.ac +
	                         a.c*b.a + a.c*b.b + a.c*b.ab +
	                         a.ab*b.c + a.ac*b.b + a.bc*b.a;
	TBM3::mass_t summ_without_zero = mass_a + mass_b + mass_c +
                                     mass_ab + mass_ac + mass_bc +
                                     mass_abc;
	TBM3::mass_t summ = mass_a + mass_b + mass_c +
	                    mass_ab + mass_ac + mass_bc +
	                    mass_abc + mass_zero;
	// summ < 1
	if (definitelyLessThan(summ, TBM3::ONE, epsilon)) {
		std::cout << "something is wrong in computation when two cells are merged" << std::endl
		          << "cell a: " << std::endl << a << std::endl
		          << "cell b: " << std::endl << b << std::endl
		          << "calculated masses: " << std::endl
		          << "mass a: "   << mass_a   << std::endl
		          << "mass b: "   << mass_b   << std::endl
		          << "mass c: "   << mass_c   << std::endl
		          << "mass ab: "  << mass_ab  << std::endl
		          << "mass ac: "  << mass_ac  << std::endl
		          << "mass bc: "  << mass_bc  << std::endl
		          << "mass abc: " << mass_abc << std::endl
		          << "mass zero: " << mass_zero << std::endl
		          << "mass summ (without zero): " << summ_without_zero << std::endl
		          << "mass summ: " << summ << std::endl;
	}
	return TBM3(mass_a, mass_b, mass_c, mass_ab, mass_ac, mass_bc, mass_abc);
}

std::ostream& operator << (std::ostream& ostr, const TBM3& a) {
	if (print_all_info)
		ostr << "a: " << a.a << "\t" << "b: " << a.b << "\t" << "c: " << a.c << std::endl
		     << "ab: " << a.ab << "\t" << "ac: " << a.ac << "\t" << "bc: " << a.bc << std::endl
		     << "abc: " << a.abc << std::endl
		     << "-------" << std::endl
		     << "sum: " << a.a+a.b+a.c+a.ab+a.ac+a.bc+a.abc << std::endl
		     << "zero: " << TBM3::ONE - (a.a+a.b+a.c+a.ab+a.ac+a.bc+a.abc);
	else
		ostr << "(" << a.a << "; " << a.b << "; " << a.c << "; "
		     << a.ab << "; " << a.ac  << "; " << a.bc  << "; "
		     << a.abc  << "; " << ")";
	return ostr;
}
