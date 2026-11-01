"""Source guards; native behavior is exercised by the saved GPU diagnostic."""
import pytest
from benchmarks.generic_motion_v1.runners.native_drm_corrections import CORRECTIONS, correct_source
from benchmarks.generic_motion_v1.runners.native_drm_query import SOURCE, patched_source


def test_correctness_repairs_are_exact_and_do_not_edit_upstream():
    original = SOURCE.read_text()
    exits = patched_source(original)
    corrected = correct_source(exits)
    assert 'collision_set_.insert(start_id)' not in corrected
    assert 'collision_set_.insert(target_id)' not in corrected
    assert 'forbidden_edge_set_.insert({-1, start_id})' in corrected
    assert 'forbidden_edge_set_.insert({-2, target_id})' in corrected
    for before, after in reversed(CORRECTIONS):
        assert corrected.count(after) == 1
        corrected = corrected.replace(after, before, 1)
    assert corrected == exits
    assert SOURCE.read_text() == original


@pytest.mark.parametrize('before,after',CORRECTIONS)
def test_corrections_fail_closed_on_missing_or_duplicate_source(before,after):
    original=patched_source(SOURCE.read_text())
    with pytest.raises(ValueError,match='exact reviewed'):
        correct_source(original.replace(before,'',1))
    with pytest.raises(ValueError,match='exact reviewed'):
        correct_source(original + '\n' + before)
