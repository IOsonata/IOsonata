#ifndef __BT_COMPLIANCE_H__
#define __BT_COMPLIANCE_H__

#include <cstddef>

namespace btcompliance {

struct Requirement {
	const char *Id;
	const char *Specification;
	const char *Clause;
	const char *Description;
};

class Context {
public:
	explicit Context(const char *pSuiteName);

	void Begin(const Requirement &Req);
	void Check(bool Result, const char *pExpr, const char *pFile, int Line);
	void Skip(const char *pReason);
	void End();
	int Finish(bool StrictCoverage = false);

	int RequirementCount() const { return vRequirements; }
	int PassedCount() const { return vPassed; }
	int FailedCount() const { return vFailed; }
	int SkippedCount() const { return vSkipped; }
	int CheckCount() const { return vChecks; }

private:
	const char *vpSuiteName;
	const Requirement *vpCurrent;
	const char *vpSkipReason;
	int vChecks;
	int vCheckFailures;
	int vRequirements;
	int vPassed;
	int vFailed;
	int vSkipped;
	int vRequirementFailureStart;
};

} // namespace btcompliance

#define BT_CHECK(ctx, expr) \
	do { (ctx).Check((expr), #expr, __FILE__, __LINE__); } while (0)

#endif // __BT_COMPLIANCE_H__
