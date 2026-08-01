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

"""Stationary sample-mean calibration for the Mini Pupper IMU."""


DEFAULT_CALIBRATION_WARMUP_SAMPLES = 500
DEFAULT_CALIBRATION_SAMPLES = 3000


class StationarySampleCalibrator:
    """Ignore warm-up samples and average a fixed calibration window."""

    def __init__(
        self,
        dimensions,
        warmup_samples,
        calibration_samples,
    ):
        """Create a calibrator for fixed-length numeric samples."""
        if dimensions <= 0:
            raise ValueError("dimensions must be greater than zero")

        if warmup_samples < 0:
            raise ValueError(
                "warmup_samples must be greater than or equal to zero"
            )

        if calibration_samples <= 0:
            raise ValueError(
                "calibration_samples must be greater than zero"
            )

        self._dimensions = dimensions
        self._warmup_remaining = warmup_samples
        self._calibration_samples = calibration_samples
        self._samples_collected = 0
        self._sums = [0.0] * dimensions
        self._offsets = None

    @property
    def warmup_remaining(self):
        """Return the number of warm-up samples still to ignore."""
        return self._warmup_remaining

    @property
    def samples_collected(self):
        """Return the number of calibration samples accumulated."""
        return self._samples_collected

    @property
    def complete(self):
        """Return whether the calibration offset is available."""
        return self._offsets is not None

    @property
    def offsets(self):
        """Return the completed offsets, or None while calibrating."""
        return self._offsets

    def add_sample(self, sample):
        """Process one sample and return offsets when calibration ends."""
        if self.complete:
            return self._offsets

        if len(sample) != self._dimensions:
            raise ValueError(
                f"expected {self._dimensions} values, got {len(sample)}"
            )

        if self._warmup_remaining > 0:
            self._warmup_remaining -= 1
            return None

        for index, value in enumerate(sample):
            self._sums[index] += value

        self._samples_collected += 1

        if self._samples_collected < self._calibration_samples:
            return None

        self._offsets = tuple(
            total / self._calibration_samples
            for total in self._sums
        )

        return self._offsets
