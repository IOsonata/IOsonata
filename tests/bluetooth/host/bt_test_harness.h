#ifndef __BT_TEST_HARNESS_H__
#define __BT_TEST_HARNESS_H__

#include <cstdio>

namespace bttest {

class Context {
public:
	explicit Context(const char *pName) : vpName(pName), vChecks(0), vFailures(0) {}

	void Check(bool bResult, const char *pExpr, const char *pFile, int Line)
	{
		vChecks++;
		if (!bResult)
		{
			std::printf("FAIL %s:%d: %s\n", pFile, Line, pExpr);
			vFailures++;
		}
	}

	template <typename F>
	void Run(const char *pName, F Fn)
	{
		int before = vFailures;
		Fn();
		std::printf("%s %s\n", vFailures == before ? "PASS" : "FAIL", pName);
	}

	int Finish() const
	{
		if (vFailures != 0)
		{
			std::printf("%s: %d failure(s), %d checks\n",
						vpName, vFailures, vChecks);
			return 1;
		}

		std::printf("%s: PASS (%d checks)\n", vpName, vChecks);
		return 0;
	}

private:
	const char *vpName;
	int vChecks;
	int vFailures;
};

} // namespace bttest

#define BT_CHECK(ctx, expr) \
	do { (ctx).Check((expr), #expr, __FILE__, __LINE__); } while (0)

#endif
