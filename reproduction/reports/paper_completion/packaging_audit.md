# Streaming evidence archive

`reproduction/package_completion.py` packages the two main runs named by
`run_index.json`. It now includes every file in those runs except native `.map`
files: all case inputs, candidate and trajectory arrays, replay samples,
pre-solve regions and seeds, validation records, failure artifacts, frozen
source snapshots, scene records, roadmap metadata and sampled nodes. No
success-only selection is applied. Historical tracked reports remain separate;
the bundle does not pool historical experiments with the current runs.

The packager retains third-party source/licenses and execution logs. The
paper-date source is still read from the 179 immutable Git objects listed in
`paperdate_source_inventory.json`, checked against their recorded sizes and
SHA-256 values, and placed under `paperdate_source/`. A locally patched build
directory cannot replace those original objects. Final current reports and
figures are included even when generated after Git staging. Raw third-party
source downloads and connector responses retain the previous inclusion policy:
tracked files plus the compact JSON/log execution-evidence archive.

## Large files kept locally

Native `.map` files are excluded. Throughput runs named in
`throughput_index.json`, plus local attempts with the current Forest, FR3 or
public-iiwa throughput directory names, are inventoried individually. Their
`configurations.npy`, `flags.npz` and `*_flags.npz` full-batch files remain
local. JSON/log records and smaller independent-verification arrays are
included. Discovery does not select a result by performance; each run keeps its
own directory, status and provenance.

`BUNDLE_EXCLUDED_FILES.json` records each excluded file's absolute local path,
repository-relative path when applicable, run, intended archive member names,
reason, byte count and SHA-256. A copy is written beside the final archive as
`*.tar.excluded.json`. No excluded file is deleted or modified. These entries
make omissions inspectable; the portable bundle cannot execute native-map
queries or replay a full two-million-configuration throughput batch without
those local files or a rebuild.

## Memory use and integrity

The archive index holds paths and metadata rather than file contents. All file
hashes and archive reads use 1 MiB chunks; `tarfile.addfile` streams disk files.
Original Git objects are spooled to temporary disk files instead of retained in
RAM. The generated metadata JSON itself remains in memory, so index memory
scales with the number of files, not their total data size.

The compact execution-evidence archive and final archive are written to
separate partial paths, then read sequentially to verify every member's size
and SHA-256. Duplicate, unexpected or missing members fail verification.
`BUNDLE_FILES.json` excludes itself to avoid a recursive hash, but its own hash
is independently computed and checked during that same verification; it is
also recorded in the external archive manifest. The final archive's SHA-256 is
computed in chunks. Existing output is replaced only after verification;
archive-write/verification errors retain the partial file and a failure JSON.
An absent optional diagnostics directory is accepted. Runs whose manifests
still say running/preparing/pending are rejected before compression.

## Execution and validation status

Execute only after timed experiments and throughput measurements have stopped,
and after final reports/figures and their indices have been generated:

```bash
bash local/python.sh -m reproduction.package_completion
```

This edit was checked with Python AST parsing and
`git diff --check -- reproduction/package_completion.py`. Both passed. No
archive was created, compressed, read back or hashed during this review, to
avoid interference with measured work. Streaming runtime behavior and final
member counts therefore remain pending the real packaging command; the final
`*.tar.manifest.json` must report `all_contents_verified: true` before delivery.

## Actual archive verification

The first real run rejected the Forest layout because the preflight required a shared `scenes/` directory. Forest instead stores the exact circle input in each case. The requirement was corrected for Forest without altering any run data; the failure is retained in `packaging_attempts.json`.

The next actual run exited 0 and verified every one of 41,785 archive members, including all primary candidates, regions, replay arrays, failure records, 179 historical source files and the supplemental LGCS/core diagnostic arrays. The precommit archive was 306,839,538 bytes. Its 421 excluded native maps/large throughput arrays (2,080,261,865 bytes) remain local with individual hashes. The final archive is generated after the source commit; use its adjacent `.manifest.json` for the final commit, exact size and hash. No original paper TeX or figure source is bundled.
