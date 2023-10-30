#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	double maxSpeed = 600 * 0.5 / 60 * 4 * M_PI;

	const double delta_d = 0.1;
	const int sample_points = 1000;
	const int benchmark_samples = 100;

	// Test Motion Profile
	auto constraints = new Constraints(maxSpeed, maxSpeed * 3, 0.1, maxSpeed * 3, maxSpeed * 100, 11.0);

	auto profileGenerator = new ProfileGenerator(constraints, delta_d);
	// benchmark profile gen
	CubicBezier *testPath;
	int startTime = pros::micros();

	for (int i = 0; i < benchmark_samples; i++)
	{
		testPath = new CubicBezier({-12, -36}, {-12, -60}, {-36, -36}, {-36, -60}, sample_points);
		profileGenerator->generateProfile(testPath);
		delete testPath;
	}
	// testPath = new CubicBezier({-12, -36}, {-12, -60}, {-36, -36}, {-36, -60}, 10000);
	int endTime = pros::micros();
	std::cout << "Length: " << profileGenerator->getProfile().size() * delta_d << "in" << std::endl;
	std::cout << "Time: " << (endTime - startTime) / benchmark_samples / 1000.0 << "ms" << std::endl;

	auto profile = profileGenerator->getProfile();

	// print profile
	// for (int i = 0; i < profile.size(); i++)
	// {
	// 	std::cout << i << " " << profile[i].dist << "," << profile[i].vel << std::endl;
	// }
	// std::cout << testPath->getLength() << std::endl;
	std::cout << "t,dist,vel,curvature" << std::endl;

	while (1)
	{
		pros::delay(10);
	}
}