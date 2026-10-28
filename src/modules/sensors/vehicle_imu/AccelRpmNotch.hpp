// add by jayjie
#pragma once

#include <mathlib/math/filter/NotchFilter.hpp>
#include <uORB/topics/esc_status.h>

namespace sensors
{

// One bank per VehicleIMU. All mutations run on that instance's WorkQueue;
// neither filter history nor ACC sample timing is shared between IMUs.
class AccelRpmNotch
{
public:
	static constexpr unsigned MaxEscs = esc_status_s::CONNECTED_ESC_MAX;
	static constexpr unsigned MaxHarmonics = 7;
	static constexpr uint64_t TimeoutUs = 3000000;

	AccelRpmNotch() = default;
	~AccelRpmNotch() { delete[] _filters; }
	AccelRpmNotch(const AccelRpmNotch &) = delete;
	AccelRpmNotch &operator=(const AccelRpmNotch &) = delete;

	// Configure once at module start. Disabled instances allocate no filter bank.
	bool configure(unsigned harmonics, float bandwidth_hz, float minimum_hz)
	{
		if (_filters || harmonics < 1 || harmonics > MaxHarmonics
		    || !PX4_ISFINITE(bandwidth_hz) || bandwidth_hz < 5.f || bandwidth_hz > 30.f
		    || !PX4_ISFINITE(minimum_hz) || minimum_hz < 1.f || minimum_hz > 1000.f) {
			return false;
		}

		_filters = new math::NotchFilter<matrix::Vector3f>[MaxEscs * harmonics];

		if (!_filters) {
			return false;
		}

		_harmonics = harmonics;
		_bandwidth_hz = bandwidth_hz;
		_minimum_hz = math::max(minimum_hz, bandwidth_hz);
		return true;
	}

	bool enabled() const { return _filters != nullptr; }
	float sampleRateHz() const { return _sample_rate_hz; }
	unsigned harmonics() const { return _harmonics; }
	float frequency(unsigned esc, unsigned harmonic) const
	{
		return (_filters && esc < MaxEscs && harmonic < _harmonics)
		       ? _filters[esc * _harmonics + harmonic].getNotchFreq() : 0.f;
	}

	// Must be called on every Run, even when the publisher stops completely.
	void update(uint64_t now, const esc_status_s *status = nullptr)
	{
		if (!enabled()) {
			return;
		}

		_now = now;

		if (status) {
			for (unsigned esc = 0; esc < MaxEscs; ++esc) {
				_rpm_timestamp[esc] = 0;
				_rpm_hz[esc] = 0.f;

				if (esc < status->esc_count && status->timestamp > 0 && status->timestamp <= now) {
					const esc_report_s &report = status->esc[esc];

					if (report.timestamp > 0 && report.timestamp <= now) {
						// esc_rpm is mechanical RPM, including signed reverse rotation.
						_rpm_hz[esc] = fabsf(static_cast<float>(report.esc_rpm)) / 60.f;
						_rpm_timestamp[esc] = math::min(status->timestamp, report.timestamp);
					}
				}
			}
		}

		updateFilters();
	}

	matrix::Vector3f apply(const matrix::Vector3f &sample, uint64_t timestamp, uint32_t device_id, bool gap)
	{
		if (!enabled()) {
			return sample;
		}

		if (gap || device_id != _device_id || timestamp <= _last_sample || !sample.isAllFinite()) {
			resetSampleHistory();
		}

		_device_id = device_id;

		if (!sample.isAllFinite()) {
			// Preserve the invalid sample for existing sensor error handling, but
			// do not poison the recursive filter state for subsequent valid samples.
			return sample;
		}

		if (_last_sample > 0) {
			const float interval_us = timestamp - _last_sample;

			if (_sample_rate_hz > 0.f && fabsf(interval_us * _sample_rate_hz * 1e-6f - 1.f) > 0.25f) {
				// Relearn timing after a rate change or unreported sample gap.
				resetSampleHistory();

			} else {
				_interval_sum_us += interval_us;

				if (++_interval_count == 32) {
					const float rate_hz = 32e6f / _interval_sum_us;

					// This is the sensor_accel message rate, NOT the FIFO/raw ODR
					// and NOT the vehicle_imu integration/publication rate.
					if (_sample_rate_hz <= 0.f || fabsf(rate_hz - _sample_rate_hz) > 0.01f * _sample_rate_hz) {
						_sample_rate_hz = rate_hz;
						updateFilters();
					}

					_interval_sum_us = 0.f;
					_interval_count = 0;
				}
			}
		}

		_last_sample = timestamp;
		matrix::Vector3f filtered = sample;

		for (unsigned i = 0; i < MaxEscs * _harmonics; ++i) {
			if (_filters[i].getNotchFreq() > 0.f) {
				filtered = _filters[i].apply(filtered);
			}
		}

		if (!filtered.isAllFinite()) {
			resetSampleHistory();
			return sample;
		}

		return filtered;
	}

private:
	void resetSampleHistory()
	{
		_last_sample = 0;
		_sample_rate_hz = 0.f;
		_interval_sum_us = 0.f;
		_interval_count = 0;

		for (unsigned i = 0; i < MaxEscs * _harmonics; ++i) {
			_filters[i].disable();
		}
	}

	void updateFilters()
	{
		for (unsigned esc = 0; esc < MaxEscs; ++esc) {
			const bool fresh = _rpm_timestamp[esc] > 0 && _now >= _rpm_timestamp[esc]
					   && _now - _rpm_timestamp[esc] < TimeoutUs;

			for (unsigned harmonic = 0; harmonic < _harmonics; ++harmonic) {
				auto &filter = _filters[esc * _harmonics + harmonic];
				const float frequency_hz = math::max(_rpm_hz[esc] * (harmonic + 1),
								     _minimum_hz + harmonic * 0.5f * _bandwidth_hz);

				// Park low nonzero RPM at the minimum as in the gyro bank. A
				// stopped motor, stale report or unrepresentable harmonic bypasses.
				if (!fresh || _rpm_hz[esc] <= 0.f || _sample_rate_hz <= 0.f
				    || frequency_hz + 0.5f * _bandwidth_hz >= 0.5f * _sample_rate_hz) {
					filter.disable();

				} else if (fabsf(filter.getNotchFreq() - frequency_hz) > 0.1f
					   || fabsf(_coefficient_sample_rate_hz - _sample_rate_hz) > FLT_EPSILON) {
					filter.setParameters(_sample_rate_hz, frequency_hz, _bandwidth_hz);
				}
			}
		}

		_coefficient_sample_rate_hz = _sample_rate_hz;
	}

	math::NotchFilter<matrix::Vector3f> *_filters{nullptr};
	float _rpm_hz[MaxEscs] {};
	uint64_t _rpm_timestamp[MaxEscs] {};
	uint64_t _now{0};
	uint64_t _last_sample{0};
	uint32_t _device_id{0};
	unsigned _harmonics{0};
	unsigned _interval_count{0};
	float _interval_sum_us{0.f};
	float _sample_rate_hz{0.f};
	float _coefficient_sample_rate_hz{0.f};
	float _bandwidth_hz{0.f};
	float _minimum_hz{0.f};
};

} // namespace sensors
// end
