# Copyright 2026 xu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Test the stationary F1 pose-update gate."""

from nav_slam.orb_map_matcher import _is_stationary, _stationary_f1_gate


def test_stationary_speed_thresholds_include_the_boundary():
    assert _is_stationary(0.02, 0.0, 0.01, 0.02, 0.01)
    assert not _is_stationary(0.021, 0.0, 0.0, 0.02, 0.01)
    assert not _is_stationary(0.0, 0.0, -0.011, 0.02, 0.01)


def test_first_stationary_match_sets_reference():
    allowed, reference = _stationary_f1_gate(True, True, None, 60.0)

    assert allowed
    assert reference == 60.0


def test_stationary_match_requires_strictly_higher_f1():
    assert _stationary_f1_gate(True, True, 60.0, 59.9) == (False, 60.0)
    assert _stationary_f1_gate(True, True, 60.0, 60.0) == (False, 60.0)
    assert _stationary_f1_gate(True, True, 60.0, 60.1) == (True, 60.1)


def test_motion_or_disabled_gate_clears_reference():
    assert _stationary_f1_gate(True, False, 70.0, 50.0) == (True, None)
    assert _stationary_f1_gate(False, True, 70.0, 50.0) == (True, None)
