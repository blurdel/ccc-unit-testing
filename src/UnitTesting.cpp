#include <iostream>
#include <functional>
#include <stdexcept>
using namespace std;


constexpr void assert_that(bool stmt, const char* msg) {
	if (!stmt) throw std::runtime_error{ msg };
}

void run_test(void(*unit_test)(), const char* name) {
	try {
		unit_test();
		printf("[+] Test %s successful.\n", name);
	} catch (const std::exception& e) {
		printf("[-] Test failure in %s. %s.\n", name, e.what());
	}
}

struct SpeedUpdate {
	double velocity_mps;
};

struct CarDetected {
	double distance_m;
	double velocity_mps;
};

struct BrakeCommand {
	double time2collision_s;
};

struct ServiceBus {
	void publish(const BrakeCommand&);
};

template <typename T>
struct AutoBrake {
	AutoBrake(const T& publish) :
		collision_thresh_s{ 5 },
		speed_mps{},
		publish{ publish } {}

	void observe(const SpeedUpdate& x) {
		speed_mps = x.velocity_mps;
	}
	void observe(const CarDetected& cd) {
		const auto relative_velocity_mps = speed_mps - cd.velocity_mps;
		const auto time_to_collision_s = cd.distance_m / relative_velocity_mps;
//		if (time_to_collision_s > 0 &&
//				time_to_collision_s <= collision_thresh_s) {
//			publish(BrakeCommand{ time_to_collision_s });
//		}
	}

	void setCollisionThresh(double x) {
		if (x < 1) {
			throw std::runtime_error{ "Collision less than 1." };
		}
		collision_thresh_s = x;
	}
	double getCollisionThresh() const {
		return collision_thresh_s;
	}
	double getSpeedMps() const {
		return speed_mps;
	}

private:
	double collision_thresh_s;
	double speed_mps;
	const T& publish;
};

void initialSpeedIsZero() {
	AutoBrake auto_brake{ [](const BrakeCommand&){} };
	assert_that(auto_brake.getSpeedMps() == 0L, "speed not equal to 0");
}

void initialSensitivityIs5() {
	AutoBrake auto_brake{ [](const BrakeCommand&){} };
	assert_that(auto_brake.getCollisionThresh() == 5L, "sensitivity is not 5");
}

void sensitivityGreaterThan1() {
	AutoBrake auto_brake{ [](const BrakeCommand&){} };
	try {
		auto_brake.setCollisionThresh(0.5L);
	} catch (std::exception& e) {
		return;
	}
	assert_that(false, "no exception thrown");
}

void speedIsSaved() {
	AutoBrake auto_brake{ [](const BrakeCommand&){} };
	auto_brake.observe(SpeedUpdate{ 100L });
	assert_that(100L == auto_brake.getSpeedMps(), "speed not saved to 100");
	auto_brake.observe(SpeedUpdate{ 50L });
	assert_that(50L == auto_brake.getSpeedMps(), "speed not saved to 50");
	auto_brake.observe(SpeedUpdate{ 0L });
	assert_that(0L == auto_brake.getSpeedMps(), "speed not saved to 0");
}

void alertWhenImminent() {
	int brake_commands_published{};
	AutoBrake auto_brake{
		[&brake_commands_published](const BrakeCommand&) {
			brake_commands_published++;
	} };
	auto_brake.setCollisionThresh(10L);
	auto_brake.observe(SpeedUpdate{ 100L });
	auto_brake.observe(CarDetected{ 100L, 0L });
	assert_that(brake_commands_published == 1, "brake commands published not one");
}


int main() {
	cout << "" << endl;

	assert_that(1 + 2 > 2, "Something is wrong in the universe!");
//	assert_that(24 == 42, "This assertion will generate an exception");

	run_test(initialSpeedIsZero, "initial speed is 0");
	run_test(initialSensitivityIs5, "initial sensitivity is 5");
	run_test(sensitivityGreaterThan1, "sensitivity greater than 1");
	run_test(speedIsSaved, "speed is saved");
	run_test(alertWhenImminent, "alert when imminent");

//	ServiceBus bus;
//	AutoBrake auto_brake { [&bus] (const auto& cmd) {
//		bus.publish(cmd);
//		}
//	};
//
//	while (true) {
//		auto_brake.observe( SpeedUpdate{ 10L } );
//		auto_brake.observe( CarDetected{ 250L, 25l } );
//	}

	return 0;
}
