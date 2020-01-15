#ifndef TBM3_H_
#define TBM3_H_

#include <ostream>

class TBM3 {
public:
	using mass_t = double;
	static const mass_t ZERO;
	static const mass_t ONE;
private:
	mass_t a;
	mass_t b;
	mass_t c;
	mass_t ab;
	mass_t ac;
	mass_t bc;
	mass_t abc;

	bool status;
public:
	TBM3(double a = TBM3::ZERO, double b = TBM3::ZERO, double c = TBM3::ZERO,
	     double ab = TBM3::ZERO, double ac = TBM3::ZERO, double bc = TBM3::ZERO,
	     double abc = TBM3::ONE);
	void normalize();

	friend TBM3 operator + (const TBM3& a, const TBM3& b);
	TBM3& operator = (const TBM3& a);
	friend std::ostream& operator << (std::ostream& ostr, const TBM3& a);
};



#endif /* TBM3_H_ */
