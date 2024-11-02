package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve;

public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(10).withKI(0.2).withKD(0.0001)
        .withKS(0).withKV(1.5).withKA(0);

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs();
//        .withKP(0.25).withKI(0).withKD(0.005)
//        .withKS(0).withKV(0.01).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 5.18;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 2;

    private static final double kDriveGearRatio = 6.545454545454545;
    private static final double kSteerGearRatio = 10.285714285714286;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = Constants.CanivoreBusIDs.BusName;
    private static final int kPigeonId = Constants.CanivoreBusIDs.Pigeon.GetID();


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = Constants.CanivoreBusIDs.Drive_FL.GetID();
    private static final int kFrontLeftSteerMotorId = Constants.CanivoreBusIDs.Steer_FL.GetID();
    private static final int kFrontLeftEncoderId = Constants.CanivoreBusIDs.CanCoder_FL.GetID();
    private static final double kFrontLeftEncoderOffset = 0.446533203125;

    private static final double kFrontLeftXPosInches = 13.625;
    private static final double kFrontLeftYPosInches = 9.375;

    // Front Right
    private static final int kFrontRightDriveMotorId = Constants.CanivoreBusIDs.Drive_FR.GetID();
    private static final int kFrontRightSteerMotorId = Constants.CanivoreBusIDs.Steer_FR.GetID();
    private static final int kFrontRightEncoderId = Constants.CanivoreBusIDs.CanCoder_FR.GetID();
    private static final double kFrontRightEncoderOffset = -0.206298828125;

    private static final double kFrontRightXPosInches = 13.625;
    private static final double kFrontRightYPosInches = -9.375;

    // Back Left
    private static final int kBackLeftDriveMotorId = Constants.CanivoreBusIDs.Drive_BL.GetID();
    private static final int kBackLeftSteerMotorId = Constants.CanivoreBusIDs.Steer_BL.GetID();
    private static final int kBackLeftEncoderId = Constants.CanivoreBusIDs.CanCoder_BL.GetID();
    private static final double kBackLeftEncoderOffset = 0.0537109375;

    private static final double kBackLeftXPosInches = -13.625;
    private static final double kBackLeftYPosInches = 9.375;

    // Back Right
    private static final int kBackRightDriveMotorId = Constants.CanivoreBusIDs.Drive_BR.GetID();
    private static final int kBackRightSteerMotorId = Constants.CanivoreBusIDs.Steer_BR.GetID();
    private static final int kBackRightEncoderId = Constants.CanivoreBusIDs.CanCoder_BR.GetID();
    private static final double kBackRightEncoderOffset = 0.364990234375;

    private static final double kBackRightXPosInches = -13.625;
    private static final double kBackRightYPosInches = -9.375;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final Swerve DriveTrain = new Swerve(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);
}