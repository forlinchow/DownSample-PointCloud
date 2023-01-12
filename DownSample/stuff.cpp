#include "stuff.h"

bool forlin::icompare_pred(unsigned char a, unsigned char b) {
	return std::tolower(a) == std::tolower(b);
}
bool forlin::icompare(string const & a, string const & b)
{
	if (a.length() == b.length()) {
		return std::equal(b.begin(), b.end(), a.begin(), icompare_pred);
	}
	else {
		return false;
	}
}
bool forlin::iEndsWith(const string & str, const string & suffix)
{
	if (str.size() < suffix.size()) {
		return false;
	}

	auto tstr = str.substr(str.size() - suffix.size());

	return icompare(tstr, suffix);
}
;

