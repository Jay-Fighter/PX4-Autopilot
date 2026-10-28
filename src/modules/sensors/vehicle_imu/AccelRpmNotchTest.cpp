// add by jayjie
#include "AccelRpmNotch.hpp"
#include "../Integrator.hpp"
#include <gtest/gtest.h>

using matrix::Vector3f;
using sensors::AccelRpmNotch;

namespace
{

esc_status_s telemetry(uint64_t now, int32_t rpm = 6000)
{
	esc_status_s status{};
	status.timestamp = now;
	status.esc_count = 1;
	status.esc_online_flags = 1;
	status.esc[0].timestamp = now;
	status.esc[0].esc_rpm = rpm;
	return status;
}

void learnRate(AccelRpmNotch &bank, uint64_t &time, unsigned dt = 1000, uint32_t device = 1)
{
	for (unsigned i = 0; i < 34; ++i) {
		time += dt;
		bank.update(time);
		bank.apply(Vector3f{0.f, 0.f, 9.81f}, time, device, false);
	}
}

TEST(AccelRpmNotch, DisabledIsExactBypassAndConfigurationIsValidated)
{
	AccelRpmNotch bank;
	const Vector3f sample{1.f, -2.f, 9.81f};
	EXPECT_EQ((bank.apply(sample, 1000, 1, false) - sample).norm(), 0.f);
	EXPECT_FALSE(bank.configure(0, 15.f, 25.f));
	EXPECT_FALSE(bank.configure(8, 15.f, 25.f));
	EXPECT_FALSE(bank.configure(3, NAN, 25.f));
	EXPECT_FALSE(bank.configure(3, 15.f, NAN));
	EXPECT_TRUE(bank.configure(3, 15.f, 25.f));
	EXPECT_FALSE(bank.configure(3, 15.f, 25.f));
}

TEST(AccelRpmNotch, AttenuatesRotorHarmonicsButPreservesGravityAndLowFrequency)
{
	AccelRpmNotch bank;
	ASSERT_TRUE(bank.configure(3, 15.f, 25.f));
	uint64_t time = 1000000;
	double rotor_energy = 0;
	double low_frequency_energy = 0;

	for (unsigned n = 0; n < 6000; ++n) {
		time += 1000;

		if (n % 20 == 0) {
			const auto status = telemetry(time);
			bank.update(time, &status);

		} else {
			bank.update(time);
		}

		const float phase = 2.f * M_PI_F * n / 1000.f;
		const Vector3f input{sinf(100.f * phase) + 0.5f * sinf(200.f * phase) + 0.25f * sinf(300.f * phase),
				     sinf(5.f * phase), 9.81f};
		const Vector3f output = bank.apply(input, time, 1, false);
		ASSERT_TRUE(output.isAllFinite());
		EXPECT_NEAR(output(2), 9.81f, 0.002f);

		if (n >= 2000) {
			rotor_energy += output(0) * output(0);
			low_frequency_energy += output(1) * output(1);
		}
	}

	EXPECT_LT(sqrt(rotor_energy / 4000), 0.01);
	EXPECT_NEAR(sqrt(low_frequency_energy / 4000), sqrt(0.5), 0.02);
	EXPECT_NEAR(bank.sampleRateHz(), 1000.f, 0.1f);
}

TEST(AccelRpmNotch, CompleteTelemetryLossExpiresWithoutAnyNewMessage)
{
	AccelRpmNotch bank;
	ASSERT_TRUE(bank.configure(3, 15.f, 25.f));
	uint64_t time = 1000000;
	auto status = telemetry(time);
	bank.update(time, &status);
	learnRate(bank, time);
	ASSERT_GT(bank.frequency(0, 0), 0.f);
	bank.update(status.timestamp + AccelRpmNotch::TimeoutUs);
	EXPECT_EQ(bank.frequency(0, 0), 0.f);
	EXPECT_EQ(bank.frequency(0, 2), 0.f);
	const Vector3f sample{2.f, 3.f, 4.f};
	EXPECT_EQ((bank.apply(sample, time + 1000, 1, false) - sample).norm(), 0.f);
}

TEST(AccelRpmNotch, TracksIndependentMotorsReverseRotationAndNyquist)
{
	AccelRpmNotch bank;
	ASSERT_TRUE(bank.configure(3, 15.f, 25.f));
	uint64_t time = 1000000;
	auto status = telemetry(time, -6000);
	status.esc_count = 2;
	status.esc[1].timestamp = time;
	status.esc[1].esc_rpm = 12000;
	bank.update(time, &status);
	learnRate(bank, time);
	EXPECT_FLOAT_EQ(bank.frequency(0, 0), 100.f);
	EXPECT_FLOAT_EQ(bank.frequency(0, 2), 300.f);
	EXPECT_FLOAT_EQ(bank.frequency(1, 0), 200.f);
	EXPECT_FLOAT_EQ(bank.frequency(1, 1), 400.f);
	EXPECT_FLOAT_EQ(bank.frequency(1, 2), 0.f);

	status = telemetry(time, 9000);
	bank.update(time, &status);
	EXPECT_FLOAT_EQ(bank.frequency(0, 0), 150.f);
	EXPECT_FLOAT_EQ(bank.frequency(1, 0), 0.f); // Removed ESC must not retain its bank.
	status = telemetry(time, 0);
	bank.update(time, &status);
	EXPECT_FLOAT_EQ(bank.frequency(0, 0), 0.f);
	status = telemetry(time, 60);
	bank.update(time, &status);
	EXPECT_FLOAT_EQ(bank.frequency(0, 0), 25.f);
	EXPECT_FLOAT_EQ(bank.frequency(0, 1), 32.5f);
}

TEST(AccelRpmNotch, RejectsStaleIndividualReportsAndFutureTimestamps)
{
	AccelRpmNotch bank;
	ASSERT_TRUE(bank.configure(1, 15.f, 25.f));
	uint64_t time = 10000000;
	learnRate(bank, time);
	auto status = telemetry(time);
	status.esc[0].timestamp = time - AccelRpmNotch::TimeoutUs;
	bank.update(time, &status);
	EXPECT_EQ(bank.frequency(0, 0), 0.f);
	status = telemetry(time + 1);
	bank.update(time, &status);
	EXPECT_EQ(bank.frequency(0, 0), 0.f);
	status = telemetry(time);
	status.esc[0].timestamp = time + 1;
	bank.update(time, &status);
	EXPECT_EQ(bank.frequency(0, 0), 0.f);
	status = telemetry(time);
	bank.update(time, &status);
	EXPECT_EQ(bank.frequency(0, 0), 100.f);
}

TEST(AccelRpmNotch, RelearnsAfterRateDeviceGapAndTimestampChanges)
{
	AccelRpmNotch bank;
	ASSERT_TRUE(bank.configure(3, 15.f, 25.f));
	uint64_t time = 1000000;
	const auto status = telemetry(time);
	bank.update(time, &status);
	learnRate(bank, time);
	learnRate(bank, time, 2000);
	EXPECT_FLOAT_EQ(bank.sampleRateHz(), 500.f);
	EXPECT_FLOAT_EQ(bank.frequency(0, 2), 0.f);
	bank.apply(Vector3f{1.f, 2.f, 3.f}, time + 2000, 2, false);
	EXPECT_FLOAT_EQ(bank.sampleRateHz(), 0.f);
	time += 2000;
	learnRate(bank, time, 2000, 2);
	EXPECT_FLOAT_EQ(bank.sampleRateHz(), 500.f);
	bank.apply(Vector3f{1.f, 2.f, 3.f}, time + 2000, 2, true);
	EXPECT_FLOAT_EQ(bank.frequency(0, 0), 0.f);
	time += 2000;
	learnRate(bank, time, 2000, 2);
	bank.apply(Vector3f{1.f, 2.f, 3.f}, time - 2000, 2, false);
	EXPECT_FLOAT_EQ(bank.sampleRateHz(), 0.f);
}

TEST(AccelRpmNotch, NonfiniteInputDoesNotPoisonRecoveryOrOtherInstances)
{
	AccelRpmNotch first;
	AccelRpmNotch second;
	ASSERT_TRUE(first.configure(1, 15.f, 25.f));
	ASSERT_TRUE(second.configure(1, 15.f, 25.f));
	uint64_t first_time = 1000000;
	uint64_t second_time = first_time;
	const auto status = telemetry(first_time);
	first.update(first_time, &status);
	second.update(second_time, &status);
	learnRate(first, first_time);
	learnRate(second, second_time, 2000);
	EXPECT_FALSE(first.apply(Vector3f{NAN, 0.f, 9.81f}, first_time + 1000, 1, false).isAllFinite());
	EXPECT_EQ(first.frequency(0, 0), 0.f);
	EXPECT_EQ(second.frequency(0, 0), 100.f);
	first_time += 1000;
	learnRate(first, first_time);
	EXPECT_TRUE(first.apply(Vector3f{0.f, 0.f, 9.81f}, first_time + 1000, 1, false).isAllFinite());
	EXPECT_EQ(first.sampleRateHz(), 1000.f);
	EXPECT_EQ(second.sampleRateHz(), 500.f);
}

TEST(AccelRpmNotch, ParallelRawIntegrationPreservesDiagnosticInput)
{
	AccelRpmNotch bank;
	ASSERT_TRUE(bank.configure(1, 15.f, 25.f));
	sensors::Integrator raw;
	sensors::Integrator filtered;
	raw.set_reset_samples(5);
	filtered.set_reset_samples(5);
	raw.set_reset_interval(5000);
	filtered.set_reset_interval(5000);
	uint64_t time = 1000000;
	double raw_energy = 0;
	double filtered_energy = 0;

	for (unsigned n = 0; n < 2000; ++n) {
		time += 1000;
		const auto status = telemetry(time);
		bank.update(time, &status);
		const Vector3f sample{sinf(2.f *M_PI_F * 100.f *n / 1000.f), 0.f, 9.81f};
		raw.put(sample, 0.001f);
		filtered.put(bank.apply(sample, time, 1, false), 0.001f);
		ASSERT_EQ(raw.integral_ready(), filtered.integral_ready());

		if (filtered.integral_ready()) {
			Vector3f raw_delta;
			Vector3f filtered_delta;
			uint32_t raw_dt = 0;
			uint32_t filtered_dt = 0;
			ASSERT_TRUE(raw.reset(raw_delta, raw_dt));
			ASSERT_TRUE(filtered.reset(filtered_delta, filtered_dt));
			EXPECT_EQ(raw_dt, filtered_dt);
			EXPECT_NEAR(raw_delta(2), filtered_delta(2), 1e-5f);

			if (n > 1000) {
				raw_energy += raw_delta(0) * raw_delta(0);
				filtered_energy += filtered_delta(0) * filtered_delta(0);
			}
		}
	}

	EXPECT_GT(raw_energy, 1e-5);
	EXPECT_LT(filtered_energy, raw_energy * 0.001);
}

} // namespace
// end
