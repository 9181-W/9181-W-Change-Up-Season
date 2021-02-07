#include "okapi/api.hpp"
#include "autonomous.hpp"
#include "initialize.hpp"
#include "position_PID.hpp"
#include "turn_PID.hpp"
#include "strafe_PID.hpp"
#include "utils.hpp"
#include "drive_to_point_PID.hpp"
#include "async_drive_to_point_PID.hpp"
#include "position_tracker.hpp"
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

	top_mtr.moveVelocity(600);
	bottom_mtr.moveVelocity(200);
	pros::delay(300);
	top_mtr.moveVelocity(0);
	bottom_mtr.moveVelocity(0);
	pros::delay(200);

	//void drive_to_point(std::shared_ptr<ChassisController> chassis, QLength y_distance, double y_max_speed, double y_min_speed, QLength x_distance, double x_max_speed, double x_min_speed);

	// top_mtr.moveVelocity(600);
	// bottom_mtr.moveVelocity(200);
	// pros::delay(300);
	//
	// async_drive_to_point(chassis, 36_in, 100.0, 20.0, 0_in, 100.0, 15.0, -25, 0.012, 0.05, 0.07, 0.12);//first ball
	// top_mtr.moveVelocity(0);
	// bottom_mtr.moveVelocity(0);
	// pros::delay(200);
	// wait_for_drive_complete_2();
	//
	// reset_pos_generic(0.0, 0.0);
	//
	// //pros::delay(4000);
	//
	// async_drive_to_point(chassis, 16.5_in, 100.0, 20.0, -10.9_in, 100.0, 15.0, -41.1, 0.02, 0.05, 0.07, 0.12);//first ball
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_on(0);
	//
	// top_and_bottom_spin();
	// pros::delay(750);
	// top_and_bottom_off();
	//
	// async_drive_to_point(chassis, -12.7_in, 100.0, 20.0, 39.7_in, 100.0, 25.0, 90, 0.012, 0.05, 0.07, 0.12);//first ball
	// intake_on(-200);
	// pros::delay(900);
	// intake_on(0);
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_on(0);
	//
	// allowable_errors_up_1();
	// maximum_vel_adj_up();
	//
	// async_drive_to_point(chassis, -28.2_in, 100.0, 20.0, 39_in, 100.0, 60.0, 90, 0.012, 0.1, 1.0, 0.12);//first ball
	// wait_for_drive_complete_2();
	//
	// allowable_errors_back_1();
	//
	// async_drive_to_point(chassis, -28.2_in, 100.0, 20.0, 23.6_in, 100.0, 40.0, 90, 0.012, 0.1, 0.3, 0.12);//first ball
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_on(0);
	//
	// async_drive_to_point(chassis, -85.3_in, 100.0, 20.0, 2.6_in, 100.0, 40.0, 230, 0.012, 0.1, 0.3, 0.12);//first ball
	// intake_ball();
	// wait_for_drive_complete_2();
	//
	// allowable_errors_up_1();
	//
	// async_drive_to_point(chassis, -99_in, 100.0, 20.0, -12.9_in, 100.0, 40.0, 230, 0.012, 0.1, 0.3, 0.12);//first ball
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_on(0);
	//
	// allowable_errors_back_1();
	//
	// intake_on(200);
	// top_and_bottom_spin();
	// pros::delay(1000);
	// top_and_bottom_off();
	// intake_on(0);

	//pros::delay(250);

	// async_drive_to_point(chassis, -90.5_in, 100.0, 20.0, -5.9_in, 100.0, 40.0, 230, 0.012, 0.1, 0.3, 0.12);//first ball
	// intake_on(-200);
	// wait_for_drive_complete_2();
	// intake_on(0);
	//
	//
	//
	//
	//
	//
	// async_drive_to_point(chassis, -39.2_in, 100.0, 20.0, 28.1_in, 100.0, 50.0, 106, 0.01, 0.1, 0.6, 0.12);//first ball
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_on(0);
	//
	// allowable_errors_up();
	//
	// async_drive_to_point(chassis, -50.1_in, 100.0, 20.0, 40_in, 100.0, 40.0, 45, 0.009, 0.1, 1.0, 0.12);//first ball
	// intake_on(200);
	// pros::delay(650);
	// top_and_bottom_spin();
	// wait_for_drive_complete_2();
	// intake_on(0);
	// pros::delay(250);
	// top_and_bottom_off();
	//
	// allowable_errors_back();
	// maximum_vel_adj_back();
	//
	// async_drive_to_point(chassis, -96.6_in, 100.0, 20.0, -12.2_in, 100.0, 15.0, -130, 0.012, 0.1, 0.09, 0.12);//first ball
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_on(0);

	// top_and_bottom_spin();
	// pros::delay(750);
	// top_and_bottom_off();


	//POG PROG

	async_drive_to_point(chassis, 18_in, 100.0, 20.0, 0_in, 100.0, 15.0, 0, 0.009, 0.05, 0.07, 0.12);//first ball
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_on(0);
	//Driver 1 //X - (-21.9) //Y - (5.1)
	//Driver 2 //X - (-21.4) //Y - (6.8)
	//Auto //X - (21.4) 		 //Y - (5.9)

	allowable_errors_up();

	async_drive_to_point(chassis, 6.8_in, 100.0, 20.0, -21.5_in, 100.0, 15.0, -130, 0.018, 0.05, 0.04, 0.12);//first tower //former//X - (-21.5)//Y (6)
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(750);
	top_and_bottom_off();
	intake_off();

	allowable_errors_back();

	intake_on(-150);
	top_and_bottom_eject();
	async_drive_to_point(chassis, 16_in, 100.0, 15.0, -10_in, 100.0, 40.0, -170, 0.012, 0.06, 0.1, 0.12);//eject balls 1
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 25_in, 100.0, 20.0, -21_in, 100.0, 15.0, -70, 0.012, 0.04, 0.06, 0.12);//second ball
	intake_on(200);
	// intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 62.5_in, 100.0, 10.0, -6_in, 100.0, 25.0, 14, 0.03, 0.06, 0.08, 0.12); //third ball
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	//Driver 1 //X - (-20.6) //Y - (63.2)
	//Driver 2 //X - (-19.6) //Y - (64.1)
	//Auto //X - (-20.2) //Y - (63.7)

	async_drive_to_point(chassis, 64.1_in, 100.0, 15.0, -20_in, 100.0, 15.0, -90, 0.017, 0.06, 0.05, 0.12);//second tower //former//X - (-20)//Y (64.75)
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(750);
	top_and_bottom_off();
	intake_off();

	//reset_pos_x_first();

	intake_on(-150);
	top_and_bottom_eject();
	async_drive_to_point(chassis, 63.5_in, 100.0, 20.0, -7_in, 100.0, 25.0, -110, 0.03, 0.04, 0.03, 0.12);//eject balls 2
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 100_in, 100.0, 15.0, -23_in, 100.0, 30.0, -57, 0.04, 0.05, 0.1, 0.2);//fourth ball
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	// async_drive_to_point(chassis, 15_in, 100.0, 20.0, 91.2_in, 100.0, 25.0, 123, 0.012, 0.05, 0.175, 0.12);//align to seventh tower
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 9_in, 100.0, 20.0, 96.6_in, 101.0, 25.0, 135, 0.012, 0.04, 0.1, 0.12);//seventh tower
	// wait_for_drive_complete_2();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(750);
	// top_and_bottom_off();
	// intake_off();

	//Driver 1 //X - (-17.5) //Y - (114.1)
	//Driver 2 //X - (-17.4) //Y - (115.7)
	//Auto //X - () //Y -

	async_drive_to_point(chassis, 113.7_in, 100.0, 17.5, -17.5_in, 100.0, 30.0, -42, 0.012, 0.04, 0.1, 0.12);//align to third tower //former//X - (-18)//Y (119)
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	//Driver 1 //X - (-22.8) //Y - (120.1)
	//Driver 2 //X - (-22.4) //Y - (120.8)
	//Driver 3 //X - (-23.0) //Y - (120.9)
	//Driver 4 //X - (-22.7) //Y - (121.2)
	//Auto //X - (-22.2) //Y - (123.4)

	async_drive_to_point(chassis, 123.5_in, 100.0, 15.0, -22_in, 100.0, 20.0, -42.5, 0.012, 0.05, 0.06, 0.12);//third tower //former//X - (-22.5)//Y (123.5)
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(750);
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 112_in, 100.0, 15.0, -16_in, 100.0, 35.0, 90, 0.012, 0.035, 0.04, 0.12);//descore third tower // X -16 Y 112
	intake_on(-150);
	top_and_bottom_eject();
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 112.5_in, 100.0, 15.0, 0_in, 100.0, 20.0, 90, 0.012, 0.04, 0.05, 0.12);//fifth ball
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 86_in, 100.0, 20.0, 33.5_in, 100.0, 20.0, 137, 0.012, 0.04, 0.05, 0.12);//sixth ball
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	//Driver 1 //X - (36.5) //Y - (118.8)
	//Driver 2 //X - (33.6) //Y - (119.3)
	//Driver 3 //X - (34.8) //Y - (118.6)
	//Driver 4 //X - (34.0) //Y - (118.6)
	//Auto //X - (37.2) //Y - (120.1)

	allowable_errors_up();

	async_drive_to_point(chassis, 119.5_in, 100.0, 20.0, 35_in, 100.0, 30.0, 0, 0.018, 0.06, 0.07, 0.12);//fourth tower //former//X - (36)//Y (122.5)
	intake_ball();
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(750);
	top_and_bottom_off();
	intake_off();

	allowable_errors_back();

	//reset_pos_y_first();

	async_drive_to_point(chassis, 104_in, 100.0, 15.0, 42_in, 100.0, 25.0, -40, 0.012, 0.04, 0.08, 0.12);
	intake_on(-100);
	top_and_bottom_eject();
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 113.5_in, 100.0, 15.0, 54_in, 100.0, 25.0, 90, 0.012, 0.05, 0.06, 0.12);//seventh ball
	intake_on(200);
	//intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 98_in, 100.0, 20.0, 94_in, 100.0, 30.0, 90, 0.008, 0.05, 0.08, 0.12);//eighth ball
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	//reset_pos_x_second();

	//Driver Alignment 1 //X - (91.4) //Y - (119.5)
	//Driver Alignment 2 //X - (85.3) //Y - (117.8)

	//Driver 1 //X - (93.9) //Y - (122.3)
	//Driver 2 //X - (91.6) //Y - (123.8)
	//Auto //X - (94.6) //Y - (124.9)

	allowable_errors_up();

	async_drive_to_point(chassis, 113.8_in, 100.0, 20.0, 86.3_in, 100.0, 25.0, 45, 0.012, 0.15, 0.2, 0.12);//align to fifth tower
	wait_for_drive_complete_2();

	async_drive_to_point(chassis, 122.95_in, 100.0, 10.0, 91.85_in, 100.0, 25.0, 45, 0.012, 0.09, 0.07, 0.12); //fifth tower
	intake_ball();
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(800);
	top_and_bottom_off();
	intake_off();

	allowable_errors_back();

	async_drive_to_point(chassis, 62.5_in, 100.0, 20.0, 54.5_in, 100.0, 25.0, 199, 0.012, 0.06, 0.1, 0.12);//ninth ball//0.03
	intake_on(-125);//200
	top_and_bottom_eject();
	pros::delay(800);
	intake_off();
	top_and_bottom_off();
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 62.8_in, 100.0, 15.0, 89.25_in, 100.0, 10.0, 90, 0.04, 0.03, 0.03, 0.12);//tenth ball and sixth tower
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(700);
	top_and_bottom_off();
	intake_off();

	//reset_pos_x_third();

	intake_on(-150);
	top_and_bottom_eject();
	async_drive_to_point(chassis, 63_in, 100.0, 20.0, 78_in, 100.0, 25.0, 61, 0.03, 0.04, 0.06, 0.12);//descore sixth tower
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 33_in, 100.0, 20.0, 90_in, 100.0, 25.0, 123, 0.04, 0.05, 0.1, 0.2);//eleventh ball
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 10_in, 100.0, 20.0, 87.2_in, 100.0, 25.0, 123, 0.012, 0.05, 0.175, 0.12);//align to seventh tower
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 4_in, 100.0, 20.0, 93.6_in, 101.0, 25.0, 135, 0.012, 0.04, 0.1, 0.12);//seventh tower
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(750);
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 16_in, 100.0, 15.0, 93.5_in, 100.0, 35.0, 270, 0.012, 0.04, 0.06, 0.12);//descore seventh tower
	intake_on(-125);
	top_and_bottom_eject();
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 20_in, 100.0, 20.0, 33.5_in, 100.0, 25.0, 270, 0.012, 0.04, 0.1, 0.12);//align to eighth tower
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	allowable_errors_up();

	async_drive_to_point(chassis, 9_in, 100.0, 20.0, 33.5_in, 100.0, 25.0, 180, 0.012, 0.04, 0.1, 0.12);//eighth tower
	intake_ball();
	wait_for_drive_complete_2();
	top_and_bottom_spin();
	intake_on(200);
	pros::delay(700);
	top_and_bottom_off();
	intake_off();

	allowable_errors_back();

	reset_pos_y_second();

	async_drive_to_point(chassis, 30_in, 100.0, 20.0, 51_in, 100.0, 20.0, 312, 0.04, 0.05, 0.05, 0.12);
	intake_on(-200);
	top_and_bottom_eject();
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	maximum_vel_adj_up();
	allowable_errors_up_1();

	async_drive_to_point(chassis, 59.5_in, 100.0, 15.0, 17.5_in, 100.0, 25.0, 312, 0.012, 0.05, 0.04, 0.12);
	intake_on(200);
	intake_ball();
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 58.5_in, 100.0, 15.0, 27.5_in, 100.0, 25.0, 450, 0.03, 0.04, 0.05, 0.12);
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 59_in, 100.0, 15.0, 21.45_in, 100.0, 25.0, 450, 0.012, 0.1, 0.09, 0.12);
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 59_in, 100.0, 15.0, 24.85_in, 100.0, 25.0, 450, 0.012, 0.1, 0.09, 0.12);
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 59_in, 100.0, 15.0, 21.45_in, 100.0, 25.0, 450, 0.012, 0.1, 0.09, 0.12);
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	async_drive_to_point(chassis, 59_in, 100.0, 15.0, 25.15_in, 100.0, 25.0, 450, 0.012, 0.1, 0.09, 0.12);
	intake_on(200);
	wait_for_drive_complete_2();
	intake_off();

	maximum_vel_adj_back();

	async_drive_to_point(chassis, 53_in, 100.0, 25.0, 31.25_in, 100.0, 25.0, 403, 0.012, 0.07, 0.08, 0.12);
	top_and_bottom_spin();
	intake_on(200);
	wait_for_drive_complete_2();
	top_and_bottom_off();
	intake_off();

	async_drive_to_point(chassis, 40_in, 100.0, 25.0, 20_in, 100.0, 25.0, 403, 0.012, 0.06, 0.07, 0.12);
	wait_for_drive_complete_2();

	allowable_errors_back_1();





	// async_drive_to_point(chassis, 18_in, 100.0, 20.0, 0_in, 100.0, 15.0, 0, 0.009, 0.05, 0.07, 0.12);//first ball
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_on(0);
	//
	// async_drive_to_point(chassis, 6_in, 100.0, 20.0, -21_in, 100.0, 15.0, -130, 0.012, 0.05, 0.04, 0.12);//first tower
	// wait_for_drive_complete_2();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(750);
	// top_and_bottom_off();
	// intake_off();
	// intake_on(-150);
	// top_and_bottom_eject();
	// async_drive_to_point(chassis, 14_in, 100.0, 15.0, -6_in, 100.0, 40.0, -170, 0.025, 0.06, 0.1, 0.12);//eject balls 1
	// wait_for_drive_complete_2();
	// top_and_bottom_off();
	// intake_off();
	//
	// async_drive_to_point(chassis, 25_in, 100.0, 20.0, -22_in, 100.0, 15.0, -70, 0.012, 0.04, 0.06, 0.12);//second ball
	// intake_on(200);
	// // intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 63.5_in, 100.0, 15.0, -7_in, 100.0, 33.0, 13, 0.02, 0.04, 0.06, 0.12); //third ball
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 63.5_in, 100.0, 15.0, -7_in, 100.0, 33.0, -90, 0.015, 0.04, 0.06, 0.2);//turn to tower
	// wait_for_drive_complete_2();
	//
	// async_drive_to_point(chassis, 63.5_in, 100.0, 15.0, -19_in, 100.0, 15.0, -90, 0.02, 0.07, 0.07, 0.12);//second tower
	// wait_for_drive_complete_2();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(750);
	// top_and_bottom_off();
	// intake_off();
	//
	// intake_on(-150);
	// top_and_bottom_eject();
	// async_drive_to_point(chassis, 63.5_in, 100.0, 20.0, -9_in, 100.0, 25.0, -110, 0.03, 0.04, 0.06, 0.12);//eject balls 2
	// wait_for_drive_complete_2();
	// top_and_bottom_off();
	// intake_off();
	//
	// async_drive_to_point(chassis, 100.5_in, 100.0, 20.0, -22_in, 100.0, 25.0, -60, 0.015, 0.04, 0.1, 0.12);//fourth ball
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 122_in, 100.0, 30.0, -21.70_in, 100.0, 50.0, -48, 0.012, 0.06, 0.2, 0.12);//third tower
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(500);
	// top_and_bottom_off();
	// intake_off();
	//
	// async_drive_to_point(chassis, 112_in, 100.0, 15.0, -18_in, 100.0, 35.0, 90, 0.012, 0.04, 0.06, 0.12);
	// intake_on(-125);
	// top_and_bottom_eject();
	// wait_for_drive_complete_2();
	// top_and_bottom_off();
	// intake_off();
	//
	// async_drive_to_point(chassis, 112.5_in, 100.0, 15.0, 0_in, 100.0, 20.0, 90, 0.012, 0.04, 0.06, 0.12);
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 88.5_in, 100.0, 20.0, 36_in, 100.0, 20.0, 127, 0.012, 0.04, 0.06, 0.12);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 121_in, 100.0, 30.0, 37_in, 100.0, 20.0, 0, 0.012, 0.04, 0.06, 0.12);//fourth tower
	// intake_ball();
	// wait_for_drive_complete_2();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(700);
	// top_and_bottom_off();
	// intake_off();
	//
	// async_drive_to_point(chassis, 112_in, 100.0, 15.0, 42_in, 100.0, 25.0, -40, 0.007, 0.04, 0.08, 0.12);
	// intake_on(-200);
	// top_and_bottom_eject();
	// wait_for_drive_complete_2();
	// top_and_bottom_off();
	// intake_off();
	//
	// async_drive_to_point(chassis, 115_in, 100.0, 15.0, 54_in, 100.0, 25.0, 90, 0.012, 0.04, 0.06, 0.12);
	// intake_on(200);
	// //intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 101_in, 100.0, 15.0, 98_in, 100.0, 25.0, 90, 0.008, 0.04, 0.06, 0.12);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 121_in, 100.0, 20.0, 87_in, 100.0, 25.0, 45, 0.012, 0.12, 0.1, 0.12);
	// wait_for_drive_complete_2();
	// async_drive_to_point(chassis, 125.25_in, 100.0, 20.0, 94.25_in, 100.0, 25.0, 45, 0.012, 0.12, 0.1, 0.12);
	// intake_ball();
	// wait_for_drive_complete_2();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(800);
	// top_and_bottom_off();
	// intake_off();
	//
	// async_drive_to_point(chassis, 64_in, 100.0, 15.0, 58.5_in, 100.0, 35.0, 190, 0.02, 0.04, 0.1, 0.12);
	// intake_on(-200);
	// top_and_bottom_eject();
	// pros::delay(750);
	// intake_off();
	// top_and_bottom_off();
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 66_in, 100.0, 15.0, 93.5_in, 100.0, 10.0, 90, 0.04, 0.03, 0.03, 0.12);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(750);
	// top_and_bottom_off();
	// intake_off();
	//
	// intake_on(-150);
	// top_and_bottom_eject();
	// async_drive_to_point(chassis, 64_in, 100.0, 20.0, 84_in, 100.0, 25.0, 65, 0.03, 0.04, 0.06, 0.12);//eject balls 2
	// wait_for_drive_complete_2();
	// top_and_bottom_off();
	// intake_off();
	//
	// //0.06
	// async_drive_to_point(chassis, 33_in, 100.0, 20.0, 98_in, 100.0, 25.0, 123, 0.02, 0.04, 0.1, 0.12);//fourth ball
	// intake_on(200);
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 16_in, 100.0, 20.0, 90_in, 100.0, 25.0, 123, 0.012, 0.04, 0.1, 0.12);
	// intake_on(200);
	// intake_ball();
	// wait_for_drive_complete_2();
	// intake_off();
	//
	// async_drive_to_point(chassis, 9.5_in, 100.0, 20.0, 98.5_in, 100.0, 25.0, 135, 0.012, 0.04, 0.1, 0.12);
	// wait_for_drive_complete_2();
	// top_and_bottom_spin();
	// intake_on(200);
	// pros::delay(800);
	// top_and_bottom_off();
	// intake_off();
	//
	// async_drive_to_point(chassis, 16_in, 100.0, 15.0, 97_in, 100.0, 35.0, -90, 0.012, 0.04, 0.06, 0.12);
	// intake_on(-125);
	// top_and_bottom_eject();
	// wait_for_drive_complete_2();
	// top_and_bottom_off();
	// intake_off();
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
