package frc.robot.subsystems;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.constants.Constants;
import frc.robot.vision.LimelightHelpers;
import frc.robot.RobotContainer;
//*TODO: Find actual poses, work out motor configurations, do commands  and logic for moving pivot



public class IntakeSubsystem extends SubsystemBase
{

    public enum EPivotPosition
    {
        Stowed(PivotLimitReverse + PivotLimitReverseBuffer),
        Intake(-0.04),
        Shoot_speaker(PivotLimitReverse),
        Amp(0.075),
        Trap(PivotLimitReverse + PivotLimitReverseBuffer),
        Climb(-0.25),
        Source(-0.237),
        Unstick(-0.166);

        public final double Rotations;
        EPivotPosition(double rotations) { Rotations = rotations; }
    }

    public enum EOuttakeType
    {
        amp(-0.5),
        speaker(-1), // known value = -60,  to testing value: 74
        None(0),

        trap(0);

        private final double DutyCycle;
        EOuttakeType(double dutyCycle) { DutyCycle = dutyCycle; }
    }

    enum EFeedType
    {
        Intake_FromGround(0.6),
        Intake_FromSource(0.4),
        Intake_ToFeeder(0.3),
        Feeder_TakeNote(-0.075),
        Feeder_GiveNote(0.5);

        private final double DutyCycle;
        EFeedType(double dutyCycle) { DutyCycle = dutyCycle; }
    }

    private final double PivotTolerance = 15.0 / 360.0;
    static private final double PivotLimitForward = 0.325;
    static private final double PivotLimitReverse = -0.31;

    // Since we zero on the hard stop, add this buffer to when going home, so we don't slam into the stop.
    static private final double PivotLimitReverseBuffer = 0.02;


    // -- Motors
    private TalonFX PivotMotor;
    private TalonFX IntakeMotor;
    private CANSparkFlex FeederMotor;
    private SparkPIDController FeederMotorPID;


    // -- Phoenix Requests
    private final MotionMagicExpoTorqueCurrentFOC PivotRequest = new MotionMagicExpoTorqueCurrentFOC(0);

//    private final VelocityTorqueCurrentFOC IntakeRequest = new VelocityTorqueCurrentFOC(0);

    private final DutyCycleOut IntakeRequest = new DutyCycleOut(0);

    private final DigitalInput LeftSwitch = new DigitalInput(1);
    private final DigitalInput RightSwitch = new DigitalInput(2);


    double lastCurrent = 0;
    int currentSpikeCount = 0;
    boolean IsFeedingNote = false;
    boolean HasGottenNote = false;

    public boolean GetIsFeedingNote() { return IsFeedingNote; }

    public IntakeSubsystem()
    {
        CreatePivotMotor();
        CreateIntakeMotor();
        CreateFeederMotor();

    }

    // --------------------------------------------------------------------------------------------
    // -- Pivot Motor
    // --------------------------------------------------------------------------------------------
    private void CreatePivotMotor()
    {
        PivotMotor = new TalonFX(Constants.CanivoreBusIDs.IntakePivot.GetID(), Constants.CanivoreBusIDs.BusName);

        var configs = new TalonFXConfiguration();

        configs.withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

        configs.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(1500)
                .withKI(0)
                .withKD(200)
                .withKS(10)
                .withKA(0)
                .withKV(0)
                .withKG(22));

        configs.withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(0)
                .withMotionMagicCruiseVelocity(4)
                .withMotionMagicExpo_kA(5)
                .withMotionMagicExpo_kV(5)
                .withMotionMagicJerk(1000));

        configs.withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(60));


        configs.withSoftwareLimitSwitch(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(PivotLimitForward)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(PivotLimitReverse));

        PivotMotor.getConfigurator().apply(configs);
        PivotMotor.setPosition(PivotLimitReverse);
    }

    // --------------------------------------------------------------------------------------------
    // -- Intake Motor
    // --------------------------------------------------------------------------------------------
    private void CreateIntakeMotor()
    {
        IntakeMotor = new TalonFX(Constants.CanivoreBusIDs.IntakeMotor.GetID(), Constants.CanivoreBusIDs.BusName);

        var configs = new TalonFXConfiguration();

        configs.withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake));

        configs.withSlot0(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(10)
                .withKI(0)
                .withKD(0)
                .withKS(40)
                .withKA(0)
                .withKV(0.25)
                .withKG(0));

        IntakeMotor.getConfigurator().apply(configs);

        IntakeMotor.stopMotor();
    }

    // --------------------------------------------------------------------------------------------
    // -- Feeder Motor
    // --------------------------------------------------------------------------------------------
    private void CreateFeederMotor()
    {
        FeederMotor = new CANSparkFlex(2, CANSparkLowLevel.MotorType.kBrushless);

        FeederMotor.restoreFactoryDefaults();
        FeederMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        FeederMotor.setInverted(true);

        FeederMotorPID = FeederMotor.getPIDController();

        FeederMotorPID.setP(0.05);
        FeederMotorPID.setI(0.0000001);
        FeederMotorPID.setD(0.01357);
        FeederMotorPID.setIZone(0);
        FeederMotorPID.setFF(0.000015);
        FeederMotorPID.setOutputRange(-1, 1);

        FeederMotor.burnFlash();

        FeederMotor.stopMotor();
    }

    // TODO: Test this change - it should fix the interrupt on let go. If it does, move the rumble
    public void RequestCancelIntake()
    {
        if (IsFeedingNote) { return; }

        CommandScheduler.getInstance().schedule(
            Commands.sequence(
                Command_StopIntake(),
                Command_SetPivotPosition(IntakeSubsystem.EPivotPosition.Stowed))
        );
    }

    private void StopMotors()
    {
        IntakeMotor.stopMotor();
        FeederMotor.stopMotor();
    }

    public double GetPivotPos() {
        return PivotMotor.getPosition().getValue();
    }

    public Command Command_SetPivotPosition(EPivotPosition position)
    {
        return Command_GoToPivotPosition(position.Rotations);
    }

    public Command Command_GoToPivotPosition(double position)
    {
        var pos = MathUtil.clamp(position, PivotLimitReverse, PivotLimitForward);
        return run(() ->
                   {
                       SmartDashboard.putNumber("Intake.PivotTarget", pos);
                       PivotMotor.setControl(PivotRequest.withPosition(pos));
                   })
            .until(() ->
                   {
                       double actualRotation = PivotMotor.getPosition().getValue();
                       return MathUtil.isNear(pos, actualRotation, PivotTolerance);
                   });
    }

    public Command Command_SetNeutralMode(NeutralModeValue mode)
    {
        return runOnce(() -> PivotMotor.setNeutralMode(mode)).ignoringDisable(true);
    }

    public Command Command_UnstickPivot()
    {
        return Commands.sequence(
            Command_SetPivotPosition(EPivotPosition.Unstick),
            Command_SetPivotPosition(EPivotPosition.Stowed)
        );
    }

    public Command Command_IntakeNote(boolean fromSource)
    {
        if ( RightSwitch.get() && LeftSwitch.get())
        {
            return Commands.sequence(
                    Commands.print("Intake Note starting"),

                    runOnce(() ->
                    {
                        IsFeedingNote = false;
                        IntakeMotor.setControl(IntakeRequest.withOutput(fromSource ? EFeedType.Intake_FromSource.DutyCycle : EFeedType.Intake_FromGround.DutyCycle));
                    }),

                    Command_SetPivotPosition(fromSource ? EPivotPosition.Source : EPivotPosition.Intake),

                    Commands.waitSeconds(0.25),

                    Commands.print(fromSource ? "At source, looking for note" : "On ground, looking for note"),

                    runOnce(() ->
                    {
                        currentSpikeCount = 0;
                        lastCurrent = IntakeMotor.getStatorCurrent().getValue();
                        if (fromSource)
                        {
                            LimelightHelpers.setLEDMode_ForceBlink("");
                        }
                    }),

                    Commands.waitUntil(() ->
                    {
                        double curCurrent = IntakeMotor.getStatorCurrent().getValue();
                        SmartDashboard.putNumber("Intake.currentDelta", curCurrent);

                        if (curCurrent - lastCurrent > 15)
                        {
                            currentSpikeCount++;
                        }

                        lastCurrent = curCurrent;
                        if (currentSpikeCount >= 1)
                        {
                            IsFeedingNote = true;
                            HasGottenNote = true;
                            LimelightHelpers.setLEDMode_ForceOff("");
                            return true;
                        }
                        return false;
                    }),

                    RobotContainer.Get().Command_RumbleControllers(),

                    Commands.print("Note got - stowing"),

                    runOnce(() -> PivotMotor.setControl(PivotRequest.withPosition(EPivotPosition.Stowed.Rotations))),

                    Commands.print("Slowing down intake, spinning up feeder"),
                    runOnce(() -> IntakeMotor.setControl(IntakeRequest.withOutput(EFeedType.Intake_ToFeeder.DutyCycle))),
                    runOnce(() -> FeederMotor.set(EFeedType.Feeder_TakeNote.DutyCycle)),

                    Commands.waitSeconds(0.1).unless(() -> fromSource),

                    Command_FeederTakeNote(true)

            )
                    .finallyDo(() ->
                    {
                        if (!IsFeedingNote)
                        {
                            StopMotors();
                            LimelightHelpers.setLEDMode_ForceOff("");
                        }
                    });
            }
            return Commands.none();
    }
//    public Command Command_IntakeNote(boolean fromSource)
//    {
//        return Commands.sequence(
//           Commands.print("Intake note starting"),
//
//           runOnce(() -> {
//                IsFeedingNote = false;
//                IntakeMotor.setControl(IntakeRequest.withOutput(fromSource ? EFeedType.Intake_FromSource.DutyCycle :  EFeedType.Intake_FromGround.DutyCycle));
//            }),
//
//           Command_SetPivotPosition(fromSource ? EPivotPosition.Source : EPivotPosition.Intake),
//
//           Commands.waitSeconds(0.25),
//
//           Commands.print(fromSource ? "at source, looking for note" : "On ground, looking for note"),
//
//           runOnce(() -> {
//               currentSpikeCount = 0;
//               lastCurrent = IntakeMotor.getStatorCurrent().getValue();
//               if (fromSource)
//               {
//                   LimelightHelpers.setLEDMode_ForceBlink("");
//               }
//            }),
//
//           Commands.waitUntil(() -> {
//                double curCurrent = IntakeMotor.getStatorCurrent().getValue();
//
//                SmartDashboard.putNumber("Intake.CurrentDelta", curCurrent - lastCurrent);
//
//                if (curCurrent - lastCurrent > 15)
//                {
//                    currentSpikeCount++;
//                }
//
//                lastCurrent = curCurrent;
//                if (currentSpikeCount >= 1)
//                {
//                    IsFeedingNote = true;
//                    HasGottenNote = true;
//                    LimelightHelpers.setLEDMode_ForceOff("");
//                    return true;
//                }
//                return false;
//            }),
//
//           RobotContainer.Get().Command_RumbleControllers(),
//
//           Commands.print("Note got - stowing"),
//           runOnce(() -> PivotMotor.setControl(PivotRequest.withPosition(EPivotPosition.Stowed.Rotations))),
//
//           Commands.print("slowing down intake, spinning up feeder"),
//           runOnce(() -> IntakeMotor.setControl(IntakeRequest.withOutput(EFeedType.Intake_ToFeeder.DutyCycle))),
//           runOnce(() -> FeederMotor.set(EFeedType.Feeder_TakeNote.DutyCycle)),
//
//           Commands.waitSeconds(0.1).unless(() -> fromSource),
//
//           Command_FeederTakeNote(true)
//        )
//        .finallyDo(() -> {
//            if (!IsFeedingNote)
//            {
//                StopMotors();
//                LimelightHelpers.setLEDMode_ForceOff("");
//            }
//        });
//    }

    public Command Command_FeederTakeNote(boolean skipWaitForSpinUp)
    {
        return Commands.sequence(
            runOnce(() -> FeederMotor.set(EFeedType.Feeder_TakeNote.DutyCycle)),

            Commands.waitSeconds(0.25).unless(() -> skipWaitForSpinUp),

            runOnce(() -> {
                currentSpikeCount = 0;
                lastCurrent = FeederMotor.getOutputCurrent();
                IntakeMotor.setControl(IntakeRequest.withOutput(EFeedType.Intake_ToFeeder.DutyCycle));
            }),

            Commands.waitUntil(() -> {
                double curCurrent = FeederMotor.getOutputCurrent();
                SmartDashboard.putNumber("Intake.CurrentDelta", curCurrent - lastCurrent);

                if (curCurrent - lastCurrent > 5)
                {
                    currentSpikeCount++;
                }
                lastCurrent = curCurrent;

                return currentSpikeCount >= 1;
            })
            .withTimeout(1),

            Commands.waitSeconds(0.25)

        ).finallyDo(() -> {
                StopMotors();
                IsFeedingNote = false;
        });
    }

    public Command Command_ConditionalStowAuto()
    {
        return runOnce(() -> {
           System.out.println("Auto Stow: " + HasGottenNote);
           if (!HasGottenNote)
           {
               StopMotors();
               PivotMotor.setControl(PivotRequest.withPosition(EPivotPosition.Stowed.Rotations));
           }
       });
    }

    public Command Command_MoveNote(boolean forward)
    {
        return startEnd(
            () -> {
                IntakeMotor.setControl(IntakeRequest.withOutput(forward ? -0.5: 0.5));
                FeederMotor.set(forward ? 0.3: -0.3);
            },
            () -> StopMotors()
        );
    }


    public Command Command_Outtake(EOuttakeType outtakeType)
    {
        return Commands.sequence(
            runOnce(() -> IntakeMotor.setControl(IntakeRequest.withOutput(outtakeType.DutyCycle))),
            Commands.waitSeconds(0.25),
            runOnce(() -> FeederMotor.set(0.5)),
            Commands.waitSeconds(0.5),
            Command_StopIntake()
        );
    }


    public Command Command_StopIntake() //use this in auto just in case we miss a note
    {
        return runOnce(() -> StopMotors());
    }


    public Command Command_ZeroPivotEncoder()
    {
        return runOnce(() -> PivotMotor.setPosition(PivotLimitReverse))
            .ignoringDisable(true);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("Intake.CurrentSpikeCount", currentSpikeCount);
        SmartDashboard.putNumber("Intake.PivotPosition", PivotMotor.getPosition().getValue());
        SmartDashboard.putNumber("Intake.IntakeCurrent", IntakeMotor.getStatorCurrent().getValue());
        SmartDashboard.putNumber("Intake.FeederCurrent", FeederMotor.getOutputCurrent());

    }



}
