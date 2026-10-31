# Before-fix evidence retention limitation

The requested original directory `/tmp/pytest-of-eric/pytest-197` was already absent when archival began. Only pytest-198, pytest-199 and pytest-200 remained. No original subprocess logs were copied or reconstructed. Earlier actual test output reported one baseline pass and three lifecycle failures: a SIGTERM-ignoring grandchild survived, a normally exited parent left a child running, and a process metadata write error leaked the launched process. Each test cleaned up its child processes in `finally`.

`archive_unavailable.json` records the observed absence. After-fix logs and subprocess artifacts are separate fresh evidence, not replacements for the missing originals.

## Frozen-source replay

`old_snapshot_replay.log` is a new actual replay using `runs/platform_acceptance_forest/source_snapshot/research_platform/runner.py`, not the deleted pytest-197 log. It reproduced exactly three failures and one baseline pass. `old_snapshot_replay_receipt.json` records the command and unchanged old runner SHA-256; raw child-process files are in `../process_before_replay_raw/`. Automatic Python bytecode caches created by importing the old snapshot were removed after checking their timestamps and absence from the frozen input manifest; the original snapshot identity passed verification. See `replay_cache_cleanup.json`.

## Final verification

After process-group cleanup and SIGTERM handling were repaired, all 40 selected checks passed in 6.54 seconds. The final command, log, source hashes, and preserved pytest artifacts are in `../process_final_verification_20261026T102851.476018Z/`. This includes four lifecycle tests, four signal tests, five worker fault tests, and 27 integrity tests. All checks used CPU subprocesses or isolated fake backends; they did not run numerical planning or use the GPU.

A separate real SIGTERM audit before the signal repair is retained in `../process_signal_audit/`; it observed one surviving grandchild and then cleaned it up. The final real signal test confirmed that an intentionally SIGTERM-ignoring grandchild is stopped, the caller's signal handler is restored, and scheduler receipts preserve an interrupted status and signal name. No active member of the recorded final-test process groups remained at the post-test check.
