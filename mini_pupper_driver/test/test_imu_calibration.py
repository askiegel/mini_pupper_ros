# Copyright 2026 Tony Kiegel
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

"""Tests for stationary IMU sample calibration."""

import pytest

from mini_pupper_driver.imu_calibration import (
    DEFAULT_CALIBRATION_SAMPLES,
    DEFAULT_CALIBRATION_WARMUP_SAMPLES,
    StationarySampleCalibrator,
)


def test_default_window_is_longer_than_original_calibration():
    assert DEFAULT_CALIBRATION_WARMUP_SAMPLES == 500
    assert DEFAULT_CALIBRATION_SAMPLES == 3000
    assert DEFAULT_CALIBRATION_SAMPLES > 200


def test_warmup_samples_are_ignored():
    calibrator = StationarySampleCalibrator(
        dimensions=2,
        warmup_samples=2,
        calibration_samples=2,
    )

    assert calibrator.add_sample((100.0, 200.0)) is None
    assert calibrator.add_sample((-100.0, -200.0)) is None
    assert calibrator.warmup_remaining == 0

    assert calibrator.add_sample((2.0, 4.0)) is None
    offsets = calibrator.add_sample((4.0, 8.0))

    assert offsets == pytest.approx((3.0, 6.0))


def test_offsets_are_returned_only_after_full_window():
    calibrator = StationarySampleCalibrator(
        dimensions=3,
        warmup_samples=0,
        calibration_samples=3,
    )

    assert calibrator.add_sample((1.0, 2.0, 3.0)) is None
    assert calibrator.add_sample((3.0, 4.0, 5.0)) is None

    offsets = calibrator.add_sample((5.0, 6.0, 7.0))

    assert offsets == pytest.approx((3.0, 4.0, 5.0))
    assert calibrator.complete
    assert calibrator.samples_collected == 3
    assert calibrator.offsets == offsets


def test_completed_offsets_remain_stable():
    calibrator = StationarySampleCalibrator(
        dimensions=1,
        warmup_samples=0,
        calibration_samples=1,
    )

    assert calibrator.add_sample((2.5,)) == pytest.approx((2.5,))
    assert calibrator.add_sample((99.0,)) == pytest.approx((2.5,))


@pytest.mark.parametrize(
    "arguments",
    [
        (0, 0, 1),
        (1, -1, 1),
        (1, 0, 0),
    ],
)
def test_invalid_configuration_is_rejected(arguments):
    with pytest.raises(ValueError):
        StationarySampleCalibrator(
            dimensions=arguments[0],
            warmup_samples=arguments[1],
            calibration_samples=arguments[2],
        )


def test_wrong_sample_dimension_is_rejected():
    calibrator = StationarySampleCalibrator(
        dimensions=2,
        warmup_samples=0,
        calibration_samples=1,
    )

    with pytest.raises(ValueError):
        calibrator.add_sample((1.0,))
