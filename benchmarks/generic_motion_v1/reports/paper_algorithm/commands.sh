# Actual successful CLI invocations; outputs are append-only.
# To repeat, choose new output directories or use resume on an unfinished run.
# Historical failed commands also remain in actual_cli_commands.json.
bash local/python.sh -m benchmarks.generic_motion_v1 build-roadmap --nodes 6000 --seed 0 --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json --output benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000
bash local/python.sh -m benchmarks.generic_motion_v1 build-roadmap --nodes 6000 --seed 1 --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json --output benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s001
bash local/python.sh -m benchmarks.generic_motion_v1 build-roadmap --nodes 6000 --seed 2 --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json --output benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s002
bash local/python.sh -m benchmarks.generic_motion_v1 batch --manifest benchmarks/generic_motion_v1/manifests/smoke.jsonl --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json --methods EI_ZO --output benchmarks/generic_motion_v1/results/rss2025_smoke_v1
bash local/python.sh -m benchmarks.generic_motion_v1 verify --run benchmarks/generic_motion_v1/results/rss2025_smoke_v1
bash local/python.sh -m benchmarks.generic_motion_v1 summarize --run benchmarks/generic_motion_v1/results/rss2025_smoke_v1
bash local/python.sh -m benchmarks.generic_motion_v1 batch --manifest benchmarks/generic_motion_v1/manifests/dev.jsonl --roadmaps benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s000 benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s001 benchmarks/generic_motion_v1/roadmaps/rss2025_n6000_s002 --config benchmarks/generic_motion_v1/configs/rss2025_eizo.json --methods EI_ZO --output benchmarks/generic_motion_v1/results/rss2025_dev_v1
bash local/python.sh -m benchmarks.generic_motion_v1 verify --run benchmarks/generic_motion_v1/results/rss2025_dev_v1
bash local/python.sh -m benchmarks.generic_motion_v1 replay --attempt benchmarks/generic_motion_v1/results/rss2025_smoke_v1/jobs/smoke-bottleneck-000-p00-EI_ZO-minimum_distance-0fe69a89/attempts/0001 --output benchmarks/generic_motion_v1/reports/paper_algorithm/replays/LSCS
bash local/python.sh -m benchmarks.generic_motion_v1 replay --attempt benchmarks/generic_motion_v1/results/rss2025_smoke_v1/jobs/smoke-bottleneck-000-p00-EI_ZO-minimum_time-5130e1f4/attempts/0001 --output benchmarks/generic_motion_v1/reports/paper_algorithm/replays/SCSTO
bash local/python.sh -m pytest benchmarks/generic_motion_v1/tests -q
bash local/python.sh -m benchmarks.generic_motion_v1.reports.paper_algorithm.collect_results
bash local/python.sh -m benchmarks.generic_motion_v1.reports.qualification.freeze_integrity --output benchmarks/generic_motion_v1/reports/paper_algorithm/frozen_inputs_check.json
