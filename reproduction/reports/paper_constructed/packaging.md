# V5 evidence packaging

`reproduction/package_constructed.py` reuses the streaming hash, archive writer and member-by-member read-back verification helpers from `package_completion.py`. It does not rewrite the v4 package or its raw runs. Files are selected from the filesystem, so new untracked v5 scripts are included before a Git commit.

Run only after the final main-run index exists and all computation, analysis, figure writes and report edits have stopped:

```bash
bash local/python.sh -m reproduction.package_constructed
```

Run this command directly, outside `record_constructed`: the packager records its own running/completed/failed status beside the archive. A running command-ledger entry, active run manifest or live computational process causes rejection before bulk input hashing. Package logs are omitted from their own archive to avoid archiving a file that is still being written.

The command generates two member-verified archives:

- `reproduction/reports/paper_constructed/execution_evidence.tar.gz`: every model-fitting attempt (including the two failed attempts), geometry inputs, candidate parameters and labels, independent geometry samples, row records, pair-distance matrices, failure lists, metadata and relevant command logs. `execution_evidence_manifest.json` records its hash and verification outcome. This compact evidence archive can be committed while individual raw files remain ignored.
- `reproduction/artifacts/csdecomp_paper_constructed_20261026.tar.gz`: v5 package, drivers, shared source dependencies, reports, figures, logs, environment locks, third-party source/licenses and every discovered main/pilot/supplemental raw run. It includes the compact evidence archive too. The native core source/build rules and direct old asset/config/source dependencies are retained without copying the old historical result bundle.

The portable archive preserves repository-relative paths. It discovers the indexed Forest/FR3 runs, run pointers in top-level v5 indexes and ledgers, and v5-named local attempts, including failed or unindexed attempts. Source snapshots, scenes, IK attempts, roadmap samples, seeds, corridors, trajectory coefficients, replay arrays, validation inputs and exceptions are included.

Only native `.map` files and complete throughput `configurations.npy`, `warmup_flags.npz` and `repeat_*_flags.npz` / `batch_*_flags.npz` files are excluded. Each excluded file receives an individual SHA256, byte count, repository/local path and reason in `BUNDLE_EXCLUDED_FILES.json` and the external exclusion manifest. `flags.npz` is deliberately retained because it also contains small independent-check indices/flags; all independent verification inputs and partial records remain included. No file is removed by packaging.

`BUNDLE_FILES.json` records member hashes and byte counts; `BUNDLE_SOURCE_SCOPES.json` explains source selection. The archive writer rereads every member and checks its expected identity before replacing an existing v5 archive. Failures retain partial archives and explicit diagnostics. Existing artifacts without a v5 manifest cannot be overwritten.

Acceptance is determined by the actual terminal archive manifests: `status=completed` in the packaging status record, `all_contents_verified=true` in both archive manifests, and matching SHA256 identities for the produced files. An implemented script or an earlier syntax check is not evidence of a completed archive. Failed or interrupted packaging remains identified by its status and retained diagnostics; rerun only after all source, experiment and report writes have stopped.
