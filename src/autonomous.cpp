#include "okapi/api.hpp"
#include "autonomous.hpp"
#include "initialize.hpp"
#include "position_PID.hpp"
#include "turn_PID.hpp"
#include "strafe_PID.hpp"
#include "utils.hpp"
#include "drive_to_point_PID.hpp"
#include "async_drive_to_point_PID.hpp"
using namespace okapi;


void modified_autonomous()
{

	top_mtr.setBrakeMode(AbstractMotor::brakeMode::hold);
	right_intake_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	left_intake_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	right_rear_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	left_rear_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	right_front_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);
	left_front_mtr.setBrakeMode(AbstractMotor::brakeMode::brake);

	//first tower

	top_mtr.moveVelocity(600);
	bottom_mtr.moveVelocity(200);
	pros::delay(300);
	top_mtr.moveVelocity(0);
	bottom_mtr.moveVelocity(0);
	pros::delay(200);
	// top_mtr.moveVelocity(600);
	// pros::delay(300);
	// top_mtr.moveVelocity(0);
	// pros::delay(200);
	// top_mtr.moveVelocity(600);
	// pros::delay(300);
	// top_mtr.moveVelocity(0);
	// pros::delay(200);
	// top_mtr.moveVelocity(600);
	// pros::delay(300);
	// top_mtr.moveVelocity(0);

	//void drive_to_point(std::shared_ptr<ChassisController> chassis, QLength y_distance, double y_max_speed, double y_min_speed, QLength x_distance, double x_max_speed, double x_min_speed);

	//drive_to_point(chassis, -23.5_in, 100.0, 15.0, -23.5_in, 100.0, 40.0);
	//drive_to_point(chassis, 0_in, 100.0, 15.0, 72.0_in, 100.0, 30.0);
  //drive_to_point(chassis, 72_in, 100.0, 15.0, 0.0_in, 100.0, 20.0);
	printf("****************************************************n");
	//async_drive_to_point(chassis, 23.5_in, 100.0, 15.0, 23.5_in, 100.0, 40.0, 90, 0.009);
	//drive_to_point(chassis, 0.0_in, 100.0, 15.0, 0.0_in, 100.0, 40.0, 90);




	// async_drive_to_point(chassis, 52_in, 100.0, 15.0, 0_in, 100.0, 15.0, 0, 0.009, 0.03, 0.05);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_on(0);
	// async_drive_to_point(chassis, 35_in, 100.0, 15.0, 0_in, 100.0, 50.0, -68, 0.012, 0.03, 0.09);
	// wait_for_drive_complete_2();
	// async_drive_to_point(chassis, 35_in, 100.0, 15.0, -20_in, 100.0, 25.0, -68, 0.012, 0.03, 0.05);
	// wait_for_drive_complete_2();
	// top_and_bottom_spin();
	// pros::delay(750);
	// top_and_bottom_off();

	async_drive_to_point(chassis, 18_in, 100.0, 30.0, 0_in, 100.0, 15.0, 0, 0.009, 0.02, 0.05);
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_on(0);
	//async_drive_to_point(chassis, 25_in, 100.0, 15.0, 0_in, 100.0, 15.0, -130, 0.014, 0.03, 0.05);
	async_drive_to_point(chassis, 5_in, 100.0, 15.0, -22_in, 100.0, 20.0, -130, 0.012, 0.03, 0.05);
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(750);
	top_and_bottom_off();
	intake_off();
	intake_on(-150);
	top_and_bottom_eject();
	async_drive_to_point(chassis, 14_in, 100.0, 15.0, -8_in, 100.0, 25.0, -170, 0.012, 0.03, 0.05);
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 25_in, 100.0, 15.0, -22_in, 100.0, 15.0, -70, 0.012, 0.03, 0.05);
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 63.5_in, 100.0, 15.0, -7_in, 100.0, 33.0, 10, 0.02, 0.03, 0.05); //jerk drive
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 63.5_in, 100.0, 15.0, -7_in, 100.0, 33.0, -90, 0.015, 0.03, 0.05);
	wait_for_drive_complete_2();

	async_drive_to_point(chassis, 63.5_in, 100.0, 15.0, -30_in, 100.0, 15.0, -90, 0.02, 0.07, 0.07);
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(750);
	top_and_bottom_off();
	intake_off();

	intake_on(-150);
	top_and_bottom_eject();
	async_drive_to_point(chassis, 63.5_in, 100.0, 15.0, -9_in, 100.0, 25.0, -110, 0.02, 0.03, 0.05);
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 100.5_in, 100.0, 15.0, -22_in, 100.0, 25.0, -60, 0.012, 0.03, 0.05);
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 127_in, 100.0, 15.0, -18_in, 100.0, 25.0, -48, 0.02, 0.03, 0.05);
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(750);
	top_and_bottom_off();
	intake_off();


	//printf("****************************************************n");
	//drive_to_point(chassis, 0.0_in, 100.0, 15.0, 0.0_in, 100.0, 40.0);

	// async_gyro_drive(chassis, 37_in, 70);
	// wait_for_drive_complete();
	// gyro_turn_to(chassis, -69_deg, 100, 17.5, 0.017, 0.0, 0.0001, 2);
	// async_gyro_drive(chassis, 16_in, 142);
	//
	// intake_on(200);
	// pros::delay(1000);
	// intake_off();
	// intake_ball();
	//
  // wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	// async_gyro_drive(chassis, -16_in, 142);
	// wait_for_drive_complete();
	// gyro_turn_to(chassis, -203_deg);
	// async_gyro_drive(chassis, 92_in, 150);
	// wait_for_drive_complete();
	// gyro_turn_to(chassis, -150_deg, 100, 17.5, 0.017, 0.0, 0.0001, 2);
	// async_gyro_drive(chassis, 16_in, 142);
	//
	// intake_on(200);
	// pros::delay(1000);
	// intake_off();
	// intake_ball();
	//
  // wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	// async_gyro_drive(chassis, -8_in, 142);

	// async_gyro_drive(chassis, 24_in, 142);
	// wait_for_drive_complete();
	// gyro_turn_to(chassis, -90_deg);
	// gyro_turn_to(chassis, 0_deg);
		//
		// // I Tower
		// async_gyro_drive(chassis, 24_in, 120);
		// intake_on(200);
		// intake_ball();
	  // wait_for_drive_complete();
	  // intake_off();
		// gyro_turn_to(chassis, -131_deg);
		// async_gyro_drive(chassis, 26.75_in, 65);
		// intake_ball();
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		//
		// //H tower
		// async_gyro_drive(chassis, -11_in, 142);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, 0_deg);
		// async_gyro_drive(chassis, 48_in, 71); // I to H
		// intake_on(200);
		// intake_ball();
		// wait_for_drive_complete();
		// intake_off();
		// intake_on(200);
		// gyro_turn_to(chassis, -90_deg);
		// intake_off();
		// async_gyro_drive(chassis, 8_in, 106.5);
		// intake_ball();
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		//
		// //G tower
		// async_gyro_drive(chassis, -11_in, 71);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, -11_deg);
		// async_gyro_drive(chassis, 51_in, 60); // H to G
		// intake_on(200);
		// intake_ball();
		// wait_for_drive_complete();
		// intake_off();
		// gyro_turn_to(chassis, -41_deg, 100, 17.5, 0.017, 0.0, 0.0001, 2);
		// async_gyro_drive(chassis, 10_in, 124.25);
		// intake_ball();
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		//
		// //D tower
		// async_gyro_drive(chassis, -8.5_in, 142);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, 90_deg, 100, 17.5, 0.0125, 0.0, 0.0, 2);
		// async_gyro_drive(chassis, 50_in, 60); // G to D
		// intake_on(200);
		// intake_ball();
		// wait_for_drive_complete();
		// intake_off();
		// gyro_turn_to(chassis, 0_deg);
		// async_gyro_drive(chassis, 8_in, 124.25);
		// intake_ball();
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		//
		// //A tower
		// async_gyro_drive(chassis, -7.5_in, 142);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, 90_deg, 100, 17.5, 0.014, 0.0, 0.0, 2);
		// async_gyro_drive(chassis, 47_in, 60); // D to A
		// intake_on(200);
		// intake_ball();
		// wait_for_drive_complete();
		// intake_off();
		// gyro_turn_to(chassis, 45_deg, 100, 17.5, 0.015, 0.0, 0.0, 2);
		// async_gyro_drive(chassis, 18.25_in, 124.25);
		// intake_ball();
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		//
		// //B tower
		// async_gyro_drive(chassis, -11_in, 142);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, 178_deg);
		// async_gyro_drive(chassis, 48_in, 60); // A to B
		// intake_on(200);
		// intake_ball();
		// wait_for_drive_complete();
		// intake_off();
		// gyro_turn_to(chassis, 90_deg);
		// async_gyro_drive(chassis, 10_in, 124.25);
		// intake_ball();
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		//
		// //E tower
		// async_gyro_drive(chassis, -8_in, 142);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, -97_deg);
		// async_gyro_drive(chassis, 40_in, 60);
		// intake_on(200);
		// intake_ball();
		// wait_for_drive_complete();
		// intake_off();
		// async_gyro_drive(chassis, -5.7_in, 142);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, -62_deg, 100, 17.5, 0.017, 0.0, 0.0001, 2);
		// strafe_drive(chassis, -3_in, 200, true);
		// async_gyro_drive(chassis, 8_in, 124.25);
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		//
		// //C tower
		// async_gyro_drive(chassis, -27_in, 142);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, -180_deg);
		// async_gyro_drive(chassis, 24_in, 50);
		// intake_on(200);
		// intake_ball();
		// wait_for_drive_complete();
		// intake_off();
		// gyro_turn_to(chassis, -230_deg, 100, 18, 0.017, 0.0, 0.0001, 2);
		// async_gyro_drive(chassis, 26_in, 142);
		// wait_for_drive_complete();
		// intake_ball();
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		//
		// //F tower
		// async_gyro_drive(chassis, -48_in, 142);
		// wait_for_drive_complete();
		// gyro_turn_to(chassis, -76_deg);
		// async_gyro_drive(chassis, 18_in, 50);
		// intake_on(200);
		// intake_ball();
		// wait_for_drive_complete();
		// intake_off();
		// gyro_turn_to(chassis, -180_deg, 100, 18, 0.017, 0.0, 0.0001, 2);
		// async_gyro_drive(chassis, 30_in, 142);
		// wait_for_drive_complete();
		// intake_ball();
		// wait_for_drive_complete();
		// top_and_bottom_spin();
		// pros::delay(500);
		// top_and_bottom_off();
		// async_gyro_drive(chassis, -10_in, 142);
		// intake_on(200);
		// wait_for_drive_complete();
		// intake_off();







	// //async_gyro_drive(chassis, 24_in, 200, 0.00165,  0.00008, 0.00005);
	// async_gyro_drive(chassis, 24_in, 200);
	// intake_on(200);
	// intake_ball();
  // wait_for_drive_complete();
  // intake_off();
	// //gyro_turn(chassis, -132_deg, 100, 26, 0.0115, 0.0, 0.0, 1.25);
	// gyro_turn(chassis, -132_deg, 100, 22, 0.012, 0.0, -0.009, 1.25);
	// //gyro_turn(chassis, -132_deg, 200, 0, 0.01, 0.0005, -0.5, 1.0);
	// async_gyro_drive(chassis, 29_in, 200);
	// intake_ball();
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	//
	// //second tower
	// async_gyro_drive(chassis, -12_in, 200);
	// wait_for_drive_complete();
	// gyro_turn(chassis, 130.5_deg, 100, 20, 0.0125, 0.0, 0.0, 1); //Cancer Turn
	// async_gyro_drive(chassis, 48.5_in, 100);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	// //gyro_turn(chassis, -87.5_deg, 100, 27, 0.0125, 0.0, 0.0, 2);
	// gyro_turn(chassis, -88.5_deg, 100, 22, 0.012, 0.0, -0.009, 1.25);
	// async_gyro_drive(chassis, 8.5_in, 150);
	// intake_ball();
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	//
	// //third tower
	// async_gyro_drive(chassis, -10_in, 100);
	// wait_for_drive_complete();
	// //gyro_turn(chassis, 79_deg, 100, 27, 0.013, 0.0, 0.0, 2);
	// gyro_turn(chassis, 79_deg, 100, 20, 0.0125, 0.0, 0.0, 1);
	// async_gyro_drive(chassis, 51_in, 150);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	// //gyro_turn(chassis, -33_deg, 100, 27, 0.013, 0.0, 0.0, 2);
	// gyro_turn(chassis, -32_deg, 100, 22, 0.012, 0.0, -0.009, 1.25);
	// async_gyro_drive(chassis, 10_in, 175);
	// intake_ball();
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	//
	// //fourth tower
	// async_gyro_drive(chassis, -9.5_in, 200);
	// wait_for_drive_complete();
	// //gyro_turn(chassis, 132.5_deg, 100, 27, 0.013, 0.0, 0.0, 2);
	// gyro_turn(chassis, 133.5_deg, 100, 22, 0.012, 0.0, -0.009, 1.25);
	// async_gyro_drive(chassis, 51.5_in, 200);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	// //gyro_turn(chassis, -89_deg, 100, 27, 0.013, 0.0, 0.0, 2);
	// gyro_turn(chassis, -91_deg, 100, 22, 0.012, 0.0, -0.009, 1.25);
	// async_gyro_drive(chassis, 11.25_in, 175);
	// intake_ball();
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	//
	// //fifth tower
	// async_gyro_drive(chassis, -8_in, 200);
	// wait_for_drive_complete();
	// //gyro_turn(chassis, 86_deg, 100, 27, 0.013, 0.0, 0.0, 2);
	// gyro_turn(chassis, 85.79_deg, 100, 22, 0.012, 0.0, -0.009, 1.25);
	// async_gyro_drive(chassis, 45_in, 200);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	// //gyro_turn(chassis, -40_deg, 100, 27, 0.013, 0.0, 0.0, 2);
	// gyro_turn(chassis, -41.5_deg, 100, 22, 0.012, 0.0, -0.009, 1.25);
	// async_gyro_drive(chassis, 19_in, 175);
	// intake_ball();
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	//
	// //sixth tower
	// async_gyro_drive(chassis, -12_in, 200);
	// wait_for_drive_complete();
	// gyro_turn(chassis, 130_deg, 100, 20, 0.013, 0.0, 0.0, 1); //Cancer Turn
	// async_gyro_drive(chassis, 49_in, 200);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	// //gyro_turn(chassis, -86_deg, 100, 27, 0.013, 0.0, 0.0, 2);
	// gyro_turn(chassis, -86_deg, 100, 22, 0.012, 0.0, -0.009, 1.25);
	// async_gyro_drive(chassis, 10_in, 175);
	// intake_ball();
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	//
	// //seventh tower
	// async_gyro_drive(chassis, -8_in, 200);
	// wait_for_drive_complete();
	// gyro_turn(chassis, 169.6_deg, 100, 20, 0.013, 0.0, 0.0, 1); //Cancer Turn
	// async_gyro_drive(chassis, 40_in, 200);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	// async_gyro_drive(chassis, -2_in, 200);
	// wait_for_drive_complete();
	// strafe_drive(chassis, -4_in, 200, true);
	// gyro_turn(chassis, 38_deg, 100, 20, 0.013, 0.0, 0.0, 1);
	// async_gyro_drive(chassis, 6_in, 175);
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// pros::delay(500);
	// top_and_bottom_off();
	// strafe_drive(chassis, -3_in, 200, true);










	// //First Tower
	// async_gyro_drive(chassis, 24_in, 160);
  // intake_on(200);
	// intake_ball();
  // wait_for_drive_complete();
  // intake_off();
	// gyro_turn(chassis, -131_deg, 100, 20, 0.013, 0.0, 0.0, 2);
	// async_gyro_drive(chassis, 31_in, 160);
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(750);
	// top_and_bottom_off();
	// intake_off();
	//
	// //Second Tower / First Ball
	// async_gyro_drive(chassis, -10_in, 160);
	// top_mtr.moveVelocity(-200);
	// pros::delay(250);
	// top_mtr.moveVelocity(0);
	// wait_for_drive_complete();
	// intake_on(-120);
	// top_and_bottom_eject();
	// gyro_turn(chassis, 42_deg, 100, 20, 0.013, 0.0, 0.0, 2);
	// top_and_bottom_off();
	// intake_off();
	// strafe_drive(chassis, 14_in, 200, true);
	// async_gyro_drive(chassis, 14_in, 160);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	//
	// //Second Tower / Second Ball
	// async_gyro_drive(chassis, -13_in, 160);
	// wait_for_drive_complete();
	// gyro_turn(chassis, 88.5_deg, 100, 20, 0.013, 0.0, 0.0, 2);
	// async_gyro_drive(chassis, 35_in, 160);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	// gyro_turn(chassis, -90_deg, 100, 20, 0.013, 0.0, 0.0, 2);
	// async_gyro_drive(chassis, 12_in, 160);
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(750);
	// top_and_bottom_off();
	// intake_off();
	//
	// //Third Tower / First Ball
	// async_gyro_drive(chassis, -10_in, 160);
	// intake_on(-40);
	// wait_for_drive_complete();
	// intake_off();
	// intake_on(-120);
	// top_and_bottom_spin_backwards();
	// strafe_drive(chassis, 33_in, 200, true);
	// intake_off();
	// top_and_bottom_off();
	// async_gyro_drive(chassis, 15_in, 160);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete();
	// intake_off();
	//
	// //Third Tower
	// async_gyro_drive(chassis, -9.5_in, 160);
	// wait_for_drive_complete();
	// gyro_turn(chassis, 88.5_deg, 100, 20, 0.013, 0.0, 0.0, 2);
	// async_gyro_drive(chassis, 13_in, 160);
	// wait_for_drive_complete();
	// gyro_turn(chassis, -43_deg, 100, 20, 0.013, 0.0, 0.0, 2);
	// async_gyro_drive(chassis, 15_in, 160);
	// wait_for_drive_complete();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(750);
	// top_and_bottom_off();
	// intake_off();
	//
	// //Fourth Tower
	// async_gyro_drive(chassis, -8_in, 160);
	// wait_for_drive_complete();
	// intake_on(-120);
	// top_and_bottom_eject();
	// gyro_turn(chassis, 130_deg, 100, 20, 0.013, 0.0, 0.0, 2);
	// top_and_bottom_off();
	// intake_off();

}
