#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <functional>
#include <stdexcept>
using namespace std;


//constexpr void assert_that(bool stmt, const char* msg) {
//	if (!stmt) throw std::runtime_error{ msg };
//}

//void run_test(void(*unit_test)(), const char* name) {
//	try {
//		unit_test();
//		printf("[+] Test %s successful.\n", name);
//	} catch (const std::exception& e) {
//		printf("[-] Test failure in %s. %s.\n", name, e.what());
//	}
//}

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

// Setup call backs
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

			bus.subscribe([this](const SpeedUpdate& su) {
				speed_mps = su.velocity_mps;
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


TEST_CASE("AutoBrake") {
	MockServiceBus bus{};
	AutoBrake auto_brake{ bus };

	SECTION("initial car speed is 0") {
		REQUIRE(auto_brake.getSpeedMps() == Approx(0));
	}

	SECTION("initial sensitivity is 5") {
		REQUIRE(auto_brake.getCollisionThresh() == Approx(5L));
	}

	SECTION("sensitivity less than 1") {
		REQUIRE_THROWS(auto_brake.setCollisionThresh(0.5L));
	}

	SECTION("speed is saved after update") {
		bus.speed_update_callback(SpeedUpdate{ 100L });
		REQUIRE(100L == auto_brake.getSpeedMps());
		bus.speed_update_callback(SpeedUpdate{ 50L });
		REQUIRE(50L == auto_brake.getSpeedMps());
		bus.speed_update_callback(SpeedUpdate{ 0L });
		REQUIRE(0L == auto_brake.getSpeedMps());
	}

	SECTION("no alert when not imminent") {
		auto_brake.setCollisionThresh(2L);

		bus.speed_update_callback(SpeedUpdate{ 100L });
		bus.car_detected_callback(CarDetected{ 1000L, 50L });
		REQUIRE(bus.commands_published == 0);
	}

	SECTION("alert when imminent") {
		auto_brake.setCollisionThresh(10L);

		bus.speed_update_callback(SpeedUpdate{ 100L });
		bus.car_detected_callback(CarDetected{ 100L, 0L });
		REQUIRE(bus.commands_published == 1);
		REQUIRE(bus.last_command.time2collision_s == Approx(1L));
	}
}
