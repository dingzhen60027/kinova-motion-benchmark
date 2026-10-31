"""CPU-only durable records shared by the standalone throughput drivers."""

from contextlib import contextmanager
import hashlib
from pathlib import Path
import signal
import time
import traceback

import numpy as np


def file_hash(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@contextmanager
def catch_termination():
    """Allow SIGTERM to reach the drivers' failure-record handlers."""
    def terminated(number, frame):
        raise InterruptedError(f"Received signal {number}")

    previous = signal.signal(signal.SIGTERM, terminated)
    try:
        yield
    finally:
        signal.signal(signal.SIGTERM, previous)


def save_input(directory, q):
    path = Path(directory) / "configurations.npy"
    np.save(path, q)
    return dict(configuration_file=path.name, configuration_file_sha256=file_hash(path),
                configuration_shape=list(q.shape), configuration_dtype=str(q.dtype),
                configuration_values_sha256=hashlib.sha256(q.tobytes(order="C")).hexdigest())


def measure_batches(check, q, directory, record, persist, repeats=5):
    """Persist each batch attempt, returned flags and timings before proceeding.

    ``check`` returns a host array after device work completes. Timing includes
    the callback and bool conversion; hashing and disk writes are excluded.
    The callback boundary makes fault handling testable without GPU imports.
    """
    directory = Path(directory)
    warm = None
    actual = None
    record["batches"] = []
    record["measurements"] = []
    for index in range(repeats + 1):
        name = "warmup" if index == 0 else f"repeat_{index - 1:02d}"
        row = dict(name=name, warmup=index == 0,
                   repeat=None if index == 0 else index - 1, status="running")
        record["batches"].append(row)
        record["stage"] = name
        persist()
        started = time.perf_counter()
        try:
            actual = np.asarray(check(q), dtype=bool)
        except BaseException as error:
            row.update(status="failed", seconds=time.perf_counter() - started,
                       error=repr(error), traceback=traceback.format_exc(), returned_flags=False)
            persist()
            raise
        seconds = time.perf_counter() - started
        row.update(status="recording_flags", seconds=seconds,
                   configurations_per_second=len(q) / seconds, returned_flags=True)
        persist()
        flags_path = directory / (name + "_flags.npz")
        np.savez_compressed(flags_path, collision_free=actual)
        valid_shape = actual.shape == (len(q),)
        disagreements = None if warm is None or not valid_shape else int(np.count_nonzero(actual != warm))
        row.update(
            status="completed" if valid_shape and disagreements in (None, 0) else "failed",
            collision_free=int(actual.sum()), valid_shape=valid_shape,
            flags_shape=list(actual.shape), flags_file=flags_path.name,
            flags_file_sha256=file_hash(flags_path),
            flag_values_sha256=hashlib.sha256(actual.tobytes(order="C")).hexdigest(),
            disagreements_with_warmup=disagreements,
            matches_warmup=None if warm is None else disagreements == 0,
        )
        if index == 0:
            record["warmup"] = row
            record["warmup_seconds"] = seconds
        else:
            record["measurements"].append(row)
        persist()
        if not valid_shape or disagreements not in (None, 0):
            raise ValueError("Collision batch flags differ or have wrong shape; returned flags and timings retained")
        if warm is None:
            warm = actual.copy()
    return actual
