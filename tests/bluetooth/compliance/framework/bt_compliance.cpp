#include "bt_compliance.h"

#include <cstdio>

namespace btcompliance {

Context::Context(const char *pSuiteName) :
	vpSuiteName(pSuiteName),
	vpCurrent(nullptr),
	vpSkipReason(nullptr),
	vChecks(0),
	vCheckFailures(0),
	vRequirements(0),
	vPassed(0),
	vFailed(0),
	vSkipped(0),
	vRequirementFailureStart(0)
{
}

void Context::Begin(const Requirement &Req)
{
	if (vpCurrent != nullptr)
	{
		End();
	}

	vpCurrent = &Req;
	vpSkipReason = nullptr;
	vRequirementFailureStart = vCheckFailures;
	vRequirements++;
}

void Context::Check(bool Result, const char *pExpr, const char *pFile, int Line)
{
	vChecks++;
	if (!Result)
	{
		std::printf("  CHECK FAIL %s:%d: %s\n", pFile, Line, pExpr);
		vCheckFailures++;
	}
}

void Context::Skip(const char *pReason)
{
	vpSkipReason = pReason;
}

void Context::End()
{
	if (vpCurrent == nullptr)
	{
		return;
	}

	if (vpSkipReason != nullptr)
	{
		std::printf("SKIP %-28s %-18s %s -- %s\n",
			vpCurrent->Id, vpCurrent->Clause, vpCurrent->Description, vpSkipReason);
		vSkipped++;
	}
	else if (vCheckFailures != vRequirementFailureStart)
	{
		std::printf("FAIL %-28s %-18s %s\n",
			vpCurrent->Id, vpCurrent->Clause, vpCurrent->Description);
		vFailed++;
	}
	else
	{
		std::printf("PASS %-28s %-18s %s\n",
			vpCurrent->Id, vpCurrent->Clause, vpCurrent->Description);
		vPassed++;
	}

	vpCurrent = nullptr;
	vpSkipReason = nullptr;
}

int Context::Finish(bool StrictCoverage)
{
	End();

	std::printf("%s: %d requirement(s), %d passed, %d failed, %d skipped, %d checks\n",
		vpSuiteName, vRequirements, vPassed, vFailed, vSkipped, vChecks);

	if (vFailed != 0)
	{
		return 1;
	}

	if (StrictCoverage && vSkipped != 0)
	{
		std::printf("%s: strict coverage failed because %d requirement(s) are skipped\n",
			vpSuiteName, vSkipped);
		return 2;
	}

	return 0;
}

} // namespace btcompliance
