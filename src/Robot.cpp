/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>

#include "WPILib.h"
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

#include <Drive/DifferentialDrive.h>
#include <IterativeRobot.h>
#include <PowerDistributionPanel.h>
#include <ctre/phoenix.h>
#include <SpeedControllerGroup.h>
#include <Joystick.h>
#include <XboxController.h>
#include <CameraServer.h>
#include <DoubleSolenoid.h>
#include <Solenoid.h>
#include <Ultrasonic.h>
#include <Encoder.h>

#define roboRioEncoders 	true
#define talonSRXEncoders 	false
#define USE_NAVX   			false
#define CNTL_BOX_A 			true
#define CNTL_BOX_B 			false
#define	ON_ROBOT   			false
#define ENABLE_USB_CAMERA	false

#if USE_NAVX
// NavX-MXP headers
#include "AHRS.h"
#endif

using namespace frc;
using namespace std;
using namespace nt;

class Robot : public frc::IterativeRobot {

public:
	void RobotInit() {
		m_chooser.AddDefault(kAutoNameDoNothing, kAutoNameDoNothing);
//		m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
		SmartDashboard::PutData("Auto Modes", &m_chooser);

		// Pneumatics
		roboCompressor.SetClosedLoopControl(true);

		// Put Solenoids into known state
		backBar.Set(DoubleSolenoid::kReverse);
		liftLocker.Set(DoubleSolenoid::kReverse);
		gripper.Set(false);
		intakeDeploy.Set(false);
		engageHook.Set(false);

		// Set initial angle on servo
//		ultrasonicServo.SetAngle(90);

		// Gyro calibration
//		analogGyro.Calibrate();
//		analogGyro.Reset();

#if ENABLE_USB_CAMERA
		// enable camera
		CameraServer::GetInstance()->StartAutomaticCapture();
#endif

		// Disable the watchdog timers to keep the warning messages down
		m_robotDrive.SetSafetyEnabled(false);

#if talonSRXEncoders
#endif

#if roboRioEncoders
		/* Defines the number of samples to average when determining the
		 * rate. On a quadrature encoder, values range from 1-255; larger
		 * values result in smoother but potentially less accurate
		 * rates than lower values.
		 */
		m_rightEncoder.SetSamplesToAverage(kNumSamplesToAverage);
		m_leftEncoder.SetSamplesToAverage(kNumSamplesToAverage);
		/* Defines how far the mechanism attached to the encoder moves
		 * per pulse.  In this case, we assume that a 360 count encoder
		 * is directly attached to a 6 inch diameter (3.0 inch radius) wheel, and that we want
		 * to measure distance in inches.
		 */
		m_rightEncoder.SetDistancePerPulse(kDistancePerPulse);
		m_leftEncoder.SetDistancePerPulse(kDistancePerPulse);
		/* Defines the lowest rate at which the encoder will not be
		 * considered stopped, for the purposes of the GetStopped() method.
		 * Units are in distance / second, where distance refers to the
		 * units of distance that you are using, in this case inches.
		 */
		m_rightEncoder.SetMinRate(kMinRateNotStopped);
		m_leftEncoder.SetMinRate(kMinRateNotStopped);
#endif

#if USE_NAVX
    	NetworkTableInstance::GetDefault().GetTable("datatable");
		try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 *
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
        	ahrs = new AHRS(SPI::Port::kMXP);
            //ahrs = new AHRS(I2C::Port::kMXP);
            ahrs->EnableLogging(true);
        } catch (std::exception& ex ) {
            std::string err_string = "Error instantiating navX MXP:  ";
            err_string += ex.what();
            DriverStation::ReportError(err_string.c_str());
        }
#endif
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to
	 * select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to
	 * the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as
	 * well.
	 */
	void AutonomousInit() override {
		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		if (gameData.length() > 0) {
			if (gameData[0] == 'L') {
				DriverStation::ReportError("Switch is on the left");
			}
			else {
				DriverStation::ReportError("Switch is on the right");
			}
			if (gameData[1] == 'L') {
				DriverStation::ReportError("Scale is on the left");
			}
			else {
				DriverStation::ReportError("Scale is on the right");
			}
		}

        autoLoopCounter = 0;
		m_autoSelected = m_chooser.GetSelected();
		// m_autoSelected = SmartDashboard::GetString(
		// 		"Auto Selector", kAutoNameDefault);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

//		if (m_autoSelected == kAutoNameCustom) {
//			// Custom Auto goes here
//		} else {
//			// Default Auto goes here
//		}
	}

	void AutonomousPeriodic() {
//		if (m_autoSelected == kAutoNameCustom) {
			// Custom Auto goes here
//			if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
//			{
//				gyroAngle = analogGyro.GetAngle();
//				// Do something w/ the Angle
//				m_robotDrive.ArcadeDrive(-0.5, -gyroAngle * Kp); 	// drive forwards half speed

//				autoLoopCounter++;
//				} else {
//					m_robotDrive.ArcadeDrive(0.0, 0.0); 	// stop robot
//			}
//		} else {
			// Default Auto goes here
//			if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
//			{
//				gyroAngle = analogGyro.GetAngle();
//				// Do something w/ the Angle
//				m_robotDrive.ArcadeDrive(-0.5, -gyroAngle * Kp); 	// drive forwards half speed
//
//				autoLoopCounter++;
//				} else {
//					m_robotDrive.ArcadeDrive(0.0, 0.0); 	// stop robot
//			}
//		}
	}

	void TeleopInit() {
	}

	void TeleopPeriodic() {
		int offsetVal = 1;
		int servoAngle = 80;
		int passCount = 0;
		const int numLoopsPerScan = 100;
		float rightY;
		float leftY;
//		bool buttonA;
//		bool buttonY;
//		bool forward;
//		bool reverse;

		while (IsOperatorControl()) {

#if talonSRXEncoders
#endif

#if roboRioEncoders
			// Retrieve the net displacement of the Encoders since the last
			// Reset.
			SmartDashboard::PutNumber(
					"Left Encoder Distance", m_leftEncoder.GetDistance());
			SmartDashboard::PutNumber(
					"Right Encoder Distance", m_rightEncoder.GetDistance());

			// Retrieve the current rate of the encoders.
			SmartDashboard::PutNumber(
					"Encoder Rate", m_leftEncoder.GetRate());
			SmartDashboard::PutNumber(
					"Encoder Rate", m_rightEncoder.GetRate());
#endif

#if USE_NAVX
	        if ( !ahrs ) return;

	        // Uses button 6 on the XBox Controller = RB (right bumper)
	        bool reset_yaw_button_pressed = DriverStation::GetInstance().GetStickButton(0,6);
	        if ( reset_yaw_button_pressed ) {
	        	ahrs->ZeroYaw();
	        }


	        SmartDashboard::PutBoolean( "IMU_Connected",        ahrs->IsConnected());
	        SmartDashboard::PutNumber(  "IMU_Yaw",              ahrs->GetYaw());
	        SmartDashboard::PutNumber(  "IMU_Pitch",            ahrs->GetPitch());
	        SmartDashboard::PutNumber(  "IMU_Roll",             ahrs->GetRoll());
	        SmartDashboard::PutNumber(  "IMU_CompassHeading",   ahrs->GetCompassHeading());
	        SmartDashboard::PutNumber(  "IMU_Update_Count",     ahrs->GetUpdateCount());
	        SmartDashboard::PutNumber(  "IMU_Byte_Count",       ahrs->GetByteCount());
	        SmartDashboard::PutNumber(  "IMU_Timestamp",        ahrs->GetLastSensorTimestamp());

	        /* These functions are compatible w/the WPI Gyro Class */
	        SmartDashboard::PutNumber(  "IMU_TotalYaw",         ahrs->GetAngle());
	        SmartDashboard::PutNumber(  "IMU_YawRateDPS",       ahrs->GetRate());

	        SmartDashboard::PutNumber(  "IMU_Accel_X",          ahrs->GetWorldLinearAccelX());
	        SmartDashboard::PutNumber(  "IMU_Accel_Y",          ahrs->GetWorldLinearAccelY());
	        SmartDashboard::PutBoolean( "IMU_IsMoving",         ahrs->IsMoving());
	        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
	        SmartDashboard::PutBoolean( "IMU_IsCalibrating",    ahrs->IsCalibrating());

	        SmartDashboard::PutNumber(  "Velocity_X",           ahrs->GetVelocityX() );
	        SmartDashboard::PutNumber(  "Velocity_Y",           ahrs->GetVelocityY() );
	        SmartDashboard::PutNumber(  "Displacement_X",       ahrs->GetDisplacementX() );
	        SmartDashboard::PutNumber(  "Displacement_Y",       ahrs->GetDisplacementY() );

	        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
	        /* NOTE:  These values are not normally necessary, but are made available   */
	        /* for advanced users.  Before using this data, please consider whether     */
	        /* the processed data (see above) will suit your needs.                     */

	        SmartDashboard::PutNumber(  "RawGyro_X",            ahrs->GetRawGyroX());
	        SmartDashboard::PutNumber(  "RawGyro_Y",            ahrs->GetRawGyroY());
	        SmartDashboard::PutNumber(  "RawGyro_Z",            ahrs->GetRawGyroZ());
	        SmartDashboard::PutNumber(  "RawAccel_X",           ahrs->GetRawAccelX());
	        SmartDashboard::PutNumber(  "RawAccel_Y",           ahrs->GetRawAccelY());
	        SmartDashboard::PutNumber(  "RawAccel_Z",           ahrs->GetRawAccelZ());
	        SmartDashboard::PutNumber(  "RawMag_X",             ahrs->GetRawMagX());
	        SmartDashboard::PutNumber(  "RawMag_Y",             ahrs->GetRawMagY());
	        SmartDashboard::PutNumber(  "RawMag_Z",             ahrs->GetRawMagZ());
	        SmartDashboard::PutNumber(  "IMU_Temp_C",           ahrs->GetTempC());
	        /* Omnimount Yaw Axis Information                                           */
	        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
	        AHRS::BoardYawAxis yaw_axis = ahrs->GetBoardYawAxis();
	        SmartDashboard::PutString(  "YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
	        SmartDashboard::PutNumber(  "YawAxis",              yaw_axis.board_axis );

	        /* Sensor Board Information                                                 */
	        SmartDashboard::PutString(  "FirmwareVersion",      ahrs->GetFirmwareVersion());

	        /* Quaternion Data                                                          */
	        /* Quaternions are fascinating, and are the most compact representation of  */
	        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
	        /* from the Quaternions.  If interested in motion processing, knowledge of  */
	        /* Quaternions is highly recommended.                                       */
	        SmartDashboard::PutNumber(  "QuaternionW",          ahrs->GetQuaternionW());
	        SmartDashboard::PutNumber(  "QuaternionX",          ahrs->GetQuaternionX());
	        SmartDashboard::PutNumber(  "QuaternionY",          ahrs->GetQuaternionY());
	        SmartDashboard::PutNumber(  "QuaternionZ",          ahrs->GetQuaternionZ());
#endif
			// Implement the deadband
			rightY = -m_stick.GetY(frc::GenericHID::kRightHand);
			leftY  = -m_stick.GetY(frc::GenericHID::kLeftHand);

			if ((rightY > kBottomOfDeadBand) and (rightY < kTopOfDeadBand) )
				rightY = 0;

			if ((leftY > kBottomOfDeadBand) and (leftY < kTopOfDeadBand) )
				leftY = 0;

			// Make it so....
			m_robotDrive.TankDrive(leftY, rightY);

			// Solenoid code
//			buttonA = m_stick.GetRawButton(kDoubleSolenoidReverse);
//			buttonY = m_stick.GetRawButton(kDoubleSolenoidForward);

			// In order to set the double solenoid, we will say that if neither
			//   button is pressed, it is off, if just one button is pressed,
			//   set the solenoid to correspond to that button, and if both
			//   are pressed, set the solenoid to Forwards.
//			if (buttonY)
//				piston.Set(DoubleSolenoid::kForward);
//			else if (buttonA)
//				piston.Set(DoubleSolenoid::kReverse);
//			else
//				piston.Set(DoubleSolenoid::kOff);

//			if (buttonA) {
				// Checks to see if the solenoid has already been set
				// I think that this was a problem last year, so I don't know if this works
//				if (piston.Get() != DoubleSolenoid::kReverse)
//					piston.Set(DoubleSolenoid::kReverse);
//			}

//			if (buttonY) {
//				// Checks to see if the solenoid has already been set
//				// I think that this was a problem last year, so I don't know if this works
//				if (piston.Get() != DoubleSolenoid::kForward)
//					piston.Set(DoubleSolenoid::kForward);
//			}
			// End Solenoid Code

			// Spike Relay Code
			// Retrieve the button values. GetRawButton will return
			//   true if the button is pressed and false if not.
//			forward = m_stick.GetRawButton(kRelayForwardButton);
//			reverse = m_stick.GetRawButton(kRelayReverseButton);

			// Depending on the button values, we want to use one of
			//   kOn, kOff, kForward, or kReverse.
			// kOn sets both outputs to 12V, kOff sets both to 0V,
			//   kForward sets forward to 12V and reverse to 0V, and
			//   kReverse sets reverse to 12V and forward to 0V.
//			if (forward && reverse)
//				spikeRelay.Set(Relay::kOn);
//			else if (forward)
//				spikeRelay.Set(Relay::kForward);
//			else if (reverse)
//				spikeRelay.Set(Relay::kReverse);
//			else
//				spikeRelay.Set(Relay::kOff);
			// End Spike Relay Code

			passCount++;

			if ((passCount % numLoopsPerScan) == 0) {
				if (servoAngle <= 80) {
				  offsetVal = 1;
				}
				else if (servoAngle >= 120) {
					offsetVal = -1;
				}

				passCount = 0;
				servoAngle += offsetVal;
				//
			}

//			if (!(limitSwitch.Get())) {
				// It's closed now
//			}

			//Wait(kUpdatePeriod);

		}

	}

	void TestPeriodic() {
	}

private:

	// CAN IDs  ****************************************************************
    const static int kPdpCanAddress = 15;
    const static int kPcmCanAddress = 1;

    // Channels for the wheels  (CAN IDs)
    const static int frontLeftChannel	= 3;
    const static int frontRightChannel	= 6;

    const static int rearLeftChannel	= 4;
    const static int rearRightChannel	= 5;

    // End Effector
    const static int kRightIntake		= 7;
    const static int kLeftIntake		= 8;
    const static int kSpareMC			= 9;
    const static int kRightLift			= 10;
    const static int kLeftLift			= 11;
    // End CAN IDs *************************************************************

    // Solenoid Assignments
    const static int backBarIn			= 0;  // Double
    const static int backBarOut			= 1;  // Double
    const static int liftLock			= 2;  // Double
    const static int liftUnlock			= 3;  // Double
    const static int deployIntake		= 4;  // single
    const static int gripCube			= 5;  // single
    const static int hook			    = 6;  // single

    // Analog Port assignments
    const static int rotarySw1   = 0;
    const static int rotarySw2   = 1;
    const static int rotarySw3 	 = 2;
    const static int Infrared	 = 3;

    // Digital IOs
    const static int kLeftIntakeLimitSw		= 0;
    const static int kRightIntakeLimitSw	= 1;
    const static int kLiftBottomLimitSw		= 2;
    const static int kLiftTopLimitSw		= 3;

    // Joystick port assignments
    const static int kLeftDriverJoystick	= 0;
    const static int kRightDriverJoystick	= 1;
    const static int kDisableSwitchJoystick	= 2;
    const static int kControlJoystick		= 3;
    const static int kJoystickChannel4		= 4;

	// Disable Sensor Switches (Generic USB Joystick)
    const int kDeployGrabber	  	  = 12;
    const int kDisableInfrared	  	  = 11;
    const int kDisableRightEnc	  	  = 10;
    const int kDisableLeftEnc	  	  = 9;
    const int kDisableStringPot	  	  = 8;
    const int kDisableLeftIntakeLS	  = 7;
    const int kDisableRightIntakeLS	  = 6;
    const int kDisableLiftTopLS	  	  = 5;
    const int kDisableLiftBottomLS	  = 4;
    const int kDisableExtra3	  	  = 3;
    const int kDisableExtra2	  	  = 2;
    const int kDisableExtra1	  	  = 1;

    // Control Joystick inputs (UHID)
    const int kGrabberEngage	  	  = 1;
    const int kHookDeploy		  	  = 2;
    const int kBackBarDeploy	  	  = 3;

    const int kLiftJoystickY		  = 4;  // Axis 4 Y
    const int kLiftJoystickX		  = 5;  // Axis 5 X
    const int kIntakeJoystickY		  = 1;  // Axis 1 Y
    const int kIntakeJoystickX		  = 0;  // Axis 0 X
    const int kSpeedPot				  = 2;  // Axis 2

#if talonSRXEncoders
#endif

#if roboRioEncoders
	// only used if we bring encoder inputs back to roboRIO
	const int	 kLeftEncoderAPin  = 4;
	const int	 kLeftEncoderBPin  = 5;
	const int	 kRightEncoderAPin = 6;
	const int	 kRightEncoderBPin = 7;

	// Quadrature Encoder related assignments
	const int    kNumSamplesToAverage = 5;       // Samples per round range 1-255
	const double kWheelRadius = 3.0;             // Radius in inches
	const double kPulsesPerRevolution = 4096;    // PPR of encoder
	const double k2Pi                 = 2.0 * 3.1415926; // 2 * Pi
	const double kDistancePerPulse    =
			 1.0 / (kPulsesPerRevolution * (k2Pi * kWheelRadius));
	const double kMinRateNotStopped   = 1.0;

#endif

	// Miscellaneous constants
	const float Kp = 0.03;
	const double kUpdatePeriod = 0.010;  // update period in seconds

	// Joystick deadband settings
	const float kBottomOfDeadBand = -0.1;
	const float kTopOfDeadBand    =  0.1;

	//***********************************************************************
	// Miscellaneous Globals
	int   autoLoopCounter;
	float gyroAngle;
	//***********************************************************************

	// Which Controls box are we dealing with?
#if CNTL_BOX_A
	WPI_VictorSPX *m_frontLeft   = new WPI_VictorSPX(frontLeftChannel);
    WPI_VictorSPX *m_frontRight  = new WPI_VictorSPX(frontRightChannel);

#if ON_ROBOT
	WPI_TalonSRX  *m_rearLeft    = new WPI_TalonSRX(rearLeftChannel);
    WPI_TalonSRX  *m_rearRight   = new WPI_TalonSRX(rearRightChannel);
#endif

    WPI_VictorSPX *m_rightIntake = new WPI_VictorSPX(kRightIntake);
    WPI_VictorSPX *m_leftIntake  = new WPI_VictorSPX(kLeftIntake);
    WPI_VictorSPX *m_spareMC     = new WPI_VictorSPX(kSpareMC);
    WPI_VictorSPX *m_rightLift   = new WPI_VictorSPX(kRightLift);
    WPI_VictorSPX *m_leftLift    = new WPI_VictorSPX(kLeftLift);

    // Object for dealing with the Power Distribution Panel (PDP).
	PowerDistributionPanel m_pdp{kPdpCanAddress};

	// Compressor channel -- not really needed but we'll keep it for completeness
	Compressor  roboCompressor{kPcmCanAddress};

#endif

#if CNTL_BOX_B
	WPI_VictorSPX *m_frontLeft   = new WPI_VictorSPX(frontLeftChannel);
    WPI_VictorSPX *m_frontRight  = new WPI_VictorSPX(frontRightChannel);

#if ON_ROBOT
	WPI_TalonSRX  *m_rearLeft    = new WPI_TalonSRX(rearLeftChannel);
    WPI_TalonSRX  *m_rearRight   = new WPI_TalonSRX(rearRightChannel);
#endif

    WPI_TalonSRX  *m_rightIntake = new WPI_TalonSRX(kRightIntake);
    WPI_TalonSRX  *m_leftIntake  = new WPI_TalonSRX(kLeftIntake);
    WPI_TalonSRX  *m_spareMC     = new WPI_TalonSRX(kSpareMC);
    WPI_TalonSRX  *m_rightLift   = new WPI_TalonSRX(kRightLift);
    WPI_TalonSRX  *m_leftLift    = new WPI_TalonSRX(kLeftLift);

    // Object for dealing with the Power Distribution Panel (PDP).
	PowerDistributionPanel m_pdp{kPdpCanAddress};

	// Compressor channel -- not really needed but we'll keep it for completeness
	Compressor  roboCompressor{kPcmCanAddress};
#endif

	// Declare speed controller groups if we're on the robot
#if ON_ROBOT
    SpeedControllerGroup m_left{*m_frontLeft, *m_rearLeft};
    SpeedControllerGroup m_right{*m_frontRight, *m_rearRight};
#else
    SpeedControllerGroup m_left{*m_frontLeft};
    SpeedControllerGroup m_right{*m_frontRight};
#endif

    // Tell WPIlib that we're Tank drive
    DifferentialDrive m_robotDrive{m_left, m_right};

    // Joysticks
	XboxController m_stick{kLeftDriverJoystick};

	// Pneumatics
	DoubleSolenoid backBar{kPcmCanAddress, backBarIn, backBarOut};
	DoubleSolenoid liftLocker{kPcmCanAddress, liftLock, liftUnlock};
	Solenoid 	   gripper{kPcmCanAddress, gripCube};
	Solenoid	   intakeDeploy{kPcmCanAddress, deployIntake};
	Solenoid	   engageHook{kPcmCanAddress, hook};

	// Gyro on the roboRio
//	AnalogGyro  analogGyro{gyroChannel};

	// Ultrasonic Range finder
//	Servo      ultrasonicServo{kServoChannel};
//	Ultrasonic sr04Ultrasonic{kUltraSonicPingInput, kUltraSonicPingOutput, Ultrasonic::kInches};

#if talonSRXEncoders
#endif

#if roboRioEncoders
	// Quadrature Encoders on RoboRio
	Encoder m_rightEncoder{kLeftEncoderAPin,  kLeftEncoderBPin,  false, Encoder::k4X};
	Encoder m_leftEncoder {kRightEncoderAPin, kRightEncoderBPin, false, Encoder::k4X};
#endif

#if USE_NAVX
	// Declare the NAVX
    AHRS *ahrs;

    // Network Tables Crap used by NAVX
	std::shared_ptr<NetworkTable> table{NULL};

#endif

	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDoNothing = "Do Nothing";
	const std::string kAutoCrossAutoLine = "Cross Auto Line";
	const std::string kAutoSwitchCube    = "Place Cube on Switch";
	const std::string kAutoScaleCube     = "Place Cube on Scale";
	const std::string kAutoSecondCube    = "Acquire Second Cube";

	std::string m_autoSelected;
};

START_ROBOT_CLASS(Robot)
