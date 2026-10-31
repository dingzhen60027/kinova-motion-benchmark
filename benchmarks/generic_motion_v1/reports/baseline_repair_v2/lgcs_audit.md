# Independent LGCS repair audit

Passed: **True**.
New runtime `e6b7e372e091ce5b01417073e0880e7c3068910c`; old runtime `72984027030175b7bca78e2dcf592b38f0c1864b`.

## Separate new-runtime outcomes

```json
{
  "smoke": {
    "jobs": 10,
    "legal_jobs": 8,
    "statuses": {
      "frontend_failed": 5,
      "invalid_endpoint": 2,
      "success_validated_numerical": 3
    },
    "solver_statuses": {
      "optimal": 4,
      "gaplimit": 2
    }
  },
  "dev": {
    "jobs": 180,
    "legal_jobs": 180,
    "statuses": {
      "frontend_failed": 70,
      "success_validated_numerical": 110
    },
    "solver_statuses": {
      "optimal": 60,
      "gaplimit": 79
    }
  }
}
```

## Paired development outcomes

```json
{
  "n": 180,
  "legal_n": 180,
  "transitions": {
    "success_validated_numerical -> success_validated_numerical": 94,
    "unsupported -> success_validated_numerical": 16,
    "frontend_failed -> frontend_failed": 69,
    "infrastructure_error -> frontend_failed": 1
  },
  "old_success": 94,
  "new_success": 110,
  "gained": 16,
  "lost": 0,
  "cost_difference_common_success": {
    "n": 94,
    "mean": -4.7069949491298877e-07,
    "median": -6.621743153800708e-10,
    "p90": 0.0,
    "p95": 8.647862870247964e-10
  },
  "cold_difference_all_legal": {
    "n": 180,
    "mean": 2.1518048570905295,
    "median": 0.04830906563438475,
    "p90": 5.844711871305479,
    "p95": 14.196814618748588
  },
  "old_cold_all_legal": {
    "n": 180,
    "mean": 2.1830311164622094,
    "median": 1.79153373837471,
    "p90": 4.573421950591728,
    "p95": 5.323736267886123
  },
  "new_cold_all_legal": {
    "n": 180,
    "mean": 4.334835973552739,
    "median": 1.841924583306536,
    "p90": 10.174359135562552,
    "p95": 18.263364795711816
  }
}
```

## Scope

- No GPU, solver, planner or collision checker was invoked.
- Saved collision labels and clearances are checked for consistency, not recomputed; no mathematical continuous collision proof.
- All source/goal memberships and selected edges are independently checked; completeness of region-region intersection graph is not re-solved.
- SCIP optimal/gap are numerical solver claims for the supplied local region graph, not full configuration-space global optimality.
- SCIP fractional gap uses min absolute primal/dual denominator; textual log gap is rounded percent.
- Old/new runtimes and backends are kept separate. Paired pipeline changes cannot identify a solver-only causal effect.
- Costs use only common validated successes; observed process latency includes all legal failure outcomes. No timeout censoring or offline construction time is hidden.
- Three map seeds repeat each task; differences are descriptive, no IID or confidence-interval claim.
