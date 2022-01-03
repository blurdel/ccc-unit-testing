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

typedef function<void(const SpeedUpdate&)> SpeedUpdateCallback;
typedef function<void(const CarDetected&)> CarDetectedCallback;

struct IServiceBus {
	virtual ~IServiceBus() = default;
	virtual void publish(const BrakeCommand&) = 0;
	virtual void subscribe(SpeedUpdateCallback) = 0;
	virtual void subscribe(CarDetectedCallback) = 0;
};

struct MockServiceBus : IServiceBus {
	void publish(const BrakeCommand& cmd) override {
		commands_published++;
		last_command = cmd;
	}
	void subscribe(SpeedUpdateCallback callback) override {
		speed_update_callback = callback;
	}
	void subscribe(CarDetectedCallback callback) override {
		car_detected_callback = callback;
	}

	BrakeCommand last_command{};
	int commands_published{};
	SpeedUpdateCallback speed_update_callback{};
	CarDetectedCallback car_detected_callback{};
};


struct AutoBrake {
	AutoBrake(IServiceBus& bus) :
		collision_thresh_s{ 5 },
		speed_mps{} {
			bus.subscribe([this](const SpeedUpdate& update) {
				speed_mps = update.velocity_mps;
			});
			bus.subscribe([this, &bus](const CarDetected& cd) {
				auto relative_velocity_mps = speed_mps - cd.velocity_mps;
				auto time_to_collision_s = cd.distance_m / relative_velocity_mps;
				if(time_to_collision_s > 0 && time_to_collision_s <= collision_thresh_s) {
					bus.publish(BrakeCommand{ time_to_collision_s });
				}
			});
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
};

void initialSpeedIsZero() {
	MockServiceBus bus{};
	AutoBrake auto_brake{ bus };
	assert_that(auto_brake.getSpeedMps() == 0L, "speed not equal to 0");
}

void initialSensitivityIs5() {
	MockServiceBus bus{};
	AutoBrake auto_brake{ bus };
	assert_that(auto_brake.getCollisionThresh() == 5L, "sensitivity is not 5");
}

void sensitivityGreaterThan1() {
	MockServiceBus bus{};
	AutoBrake auto_brake{ bus };
	try {
		auto_brake.setCollisionThresh(0.5L);
	} catch (std::exception& e) {
		return;
	}
	assert_that(false, "no exception thrown");
}

void speedIsSaved() {
	MockServiceBus bus{};
	AutoBrake auto_brake{ bus };

	bus.speed_update_callback(SpeedUpdate{ 100L });
	assert_that(100L == auto_brake.getSpeedMps(), "speed not saved to 100");
	bus.speed_update_callback(SpeedUpdate{ 50L });
	assert_that(50L == auto_brake.getSpeedMps(), "speed not saved to 50");
	bus.speed_update_callback(SpeedUpdate{ 0L });
	assert_that(0L == auto_brake.getSpeedMps(), "speed not saved to 0");
}

void alertWhenImminent() {
	MockServiceBus bus{};
	AutoBrake auto_brake{ bus };
	auto_brake.setCollisionThresh(10L);

	bus.speed_update_callback(SpeedUpdate{ 100L });
	bus.car_detected_callback(CarDetected{ 100L, 0L });
	assert_that(bus.commands_published == 1, "1 brake command was not published");
	assert_that(bus.last_command.time2collision_s == 1L, "time to collision not computed correctly");
}

void no_alert_when_not_imminent() {
	MockServiceBus bus{};
	AutoBrake auto_brake{ bus };
	auto_brake.setCollisionThresh(2L);
	bus.speed_update_callback(SpeedUpdate{ 100L });
	bus.car_detected_callback(CarDetected{ 1000L, 50L });
	assert_that(bus.commands_published == 0, "brake commands were published");
}

int main() {
	cout << "" << endl;

	assert_that(1 + 2 > 2, "Something is wrong in the universe!");
//	assert_that(24 == 42, "This assertion will generate an exception");

	run_test(initialSpeedIsZero, "initial speed is 0");
	run_test(initialSensitivityIs5, "initial sensitivity is 5");
	run_test(sensitivityGreaterThan1, "sensitivity greater than 1");
	run_test(speedIsSaved, "speed is saved");
	run_test(no_alert_when_not_imminent, "no alert when not imminent");
	run_test(alertWhenImminent, "alert when imminent");

	return 0;
}
