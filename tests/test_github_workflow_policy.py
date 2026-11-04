"""IOsonata keeps only the RAG maintenance action, not GitHub Actions CI."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
WORKFLOWS = ROOT / ".github/workflows"
RAG = WORKFLOWS / "build-rag-index.yml"


class GithubWorkflowPolicyTest(unittest.TestCase):
    def test_only_rag_workflow_is_present(self):
        self.assertTrue(RAG.is_file(), "RAG index updater must be preserved")
        files = {p.name for p in WORKFLOWS.iterdir() if p.is_file()}
        self.assertEqual(files, {"build-rag-index.yml"})

    def test_rag_workflow_runs_by_schedule_or_manual_request(self):
        text = RAG.read_text(encoding="utf-8")
        self.assertIn("workflow_dispatch:", text)
        self.assertIn("schedule:", text)
        self.assertNotIn("\npush:", text)
        self.assertNotIn("\npull_request:", text)


if __name__ == "__main__":
    unittest.main()
