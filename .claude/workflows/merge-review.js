export const meta = {
  name: 'merge-review',
  description: 'Review every file changed against a base branch, adversarially verify each finding, return one ranked list',
  whenToUse: 'Before merging a branch to main: one reviewer per changed file, every finding refuted by two independent verifiers, then ranked and deduplicated.',
  phases: [
    { title: 'Scope', detail: 'list the files changed against the base' },
    { title: 'Review', detail: 'one reviewer per changed file' },
    { title: 'Verify', detail: 'two independent refuters per finding' },
    { title: 'Rank', detail: 'deduplicate and rank the confirmed findings' },
  ],
}

// args: { base?: string } -- the base ref; defaults to "main".
const base = (args && args.base) || 'main'

const FILES = {
  type: 'object',
  required: ['files'],
  properties: {
    files: { type: 'array', items: { type: 'string' } },
    excluded: { type: 'array', items: { type: 'string' } },
  },
}
const FINDINGS = {
  type: 'object',
  required: ['findings'],
  properties: {
    findings: {
      type: 'array',
      items: {
        type: 'object',
        required: ['file', 'line', 'category', 'summary', 'failure_scenario'],
        properties: {
          file: { type: 'string' },
          line: { type: 'integer' },
          category: { type: 'string' },
          summary: { type: 'string' },
          failure_scenario: { type: 'string' },
        },
      },
    },
  },
}
const VERDICT = {
  type: 'object',
  required: ['refuted', 'reason'],
  properties: { refuted: { type: 'boolean' }, reason: { type: 'string' } },
}
const REPORT = {
  type: 'object',
  required: ['findings'],
  properties: {
    findings: {
      type: 'array',
      items: {
        type: 'object',
        required: ['file', 'line', 'severity', 'summary', 'failure_scenario'],
        properties: {
          file: { type: 'string' },
          line: { type: 'integer' },
          severity: { type: 'string' },
          summary: { type: 'string' },
          failure_scenario: { type: 'string' },
        },
      },
    },
  },
}

phase('Scope')
const scope = await agent(
  `Run \`git diff --name-only ${base}...HEAD\` in the repository root and return the changed files that still exist, excluding files under .agent/ (living plans and the state file), build/, results*/ and *.json baselines (pinned measurements), and also return the excluded ones. Return only the two lists.`,
  { schema: FILES, label: 'scope', phase: 'Scope', effort: 'low' },
)
const files = scope ? scope.files : []
const excluded = (scope && scope.excluded) || []
log(`${files.length} changed files against ${base} reviewed; ${excluded.length} excluded by scope (.agent/, build/, results*/, *.json baselines): ${excluded.join(', ') || 'none'}`)
if (files.length === 0) return { findings: [] }

const reviewed = await pipeline(
  files,
  file =>
    agent(
      `Review \`${file}\` as changed against \`${base}\` (\`git diff ${base}...HEAD -- ${file}\`, then read the surrounding code). Report only defects a careful colleague would block a merge on: correctness bugs with a concrete failing input, regressions against the behaviour the tests or docs promise, missing tests for a new rule, and code that contradicts the repository's stated conventions (AGENTS.md, docs/ARCHITECTURE.md). Skip style. For each finding give file, 1-based line, a category slug, a one-sentence summary and the concrete failure scenario. Return an empty list when nothing blocks.`,
      { schema: FINDINGS, label: `review:${file}`, phase: 'Review' },
    ),
  (result, file) =>
    parallel(
      ((result && result.findings) || []).map(f => () =>
        parallel(
          ['correctness', 'reproduce'].map(lens => () =>
            agent(
              `A reviewer claims a defect in \`${f.file}\` line ${f.line}: "${f.summary}". Failure scenario: "${f.failure_scenario}". Through the ${lens} lens, try to REFUTE it: read the code (and run the smallest command that settles it, if one exists and takes under two minutes). If the claim is uncertain, refuted=true. Return refuted and a one-sentence reason.`,
              { schema: VERDICT, label: `verify:${lens}:${f.file}:${f.line}`, phase: 'Verify' },
            ),
          ),
        ).then(votes => {
          // A verifier that returned nothing (skipped, or died after retries)
          // is not a refutation: the finding is kept, marked unverified, so a
          // missing vote never silently drops a defect.
          const answered = votes.filter(Boolean)
          const refuted = answered.filter(v => v.refuted).length
          if (refuted > 0) return null
          if (answered.length < 2) {
            log(`finding ${f.file}:${f.line} kept with ${answered.length} of 2 verifier votes`)
            return { ...f, verification: `unverified (${answered.length} of 2 votes)` }
          }
          return f
        }),
      ),
    ),
)
const confirmed = reviewed.filter(Boolean).flat().filter(Boolean)
log(`${confirmed.length} findings survived verification`)
if (confirmed.length === 0) return { findings: [] }

phase('Rank')
const report = await agent(
  `Deduplicate and rank these confirmed review findings, most severe first (severity: blocking, should-fix, nit). Merge findings that describe the same defect. Keep file, line, summary and failure scenario verbatim from the input where possible. Findings: ${JSON.stringify(confirmed)}`,
  { schema: REPORT, label: 'rank', phase: 'Rank' },
)
return report || { findings: confirmed }
