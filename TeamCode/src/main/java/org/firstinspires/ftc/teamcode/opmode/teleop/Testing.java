//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.RollingAverage;
//
//import org.firstinspires.ftc.teamcode.commandbase.commands.elevator.MoveElevatorToPosition;
//import org.firstinspires.ftc.teamcode.commandbase.commands.drive.RobotCentricPID;
//import org.firstinspires.ftc.teamcode.commandbase.commands.drive.SetHeadingTarget;
//import org.firstinspires.ftc.teamcode.commandbase.commands.intake.SetIntakePower;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.DrivetrainSubsystem;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.ElevatorSubsystem;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.commandbase.subsystems.Subsystems;
//import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
//
//@TeleOp
//@Disabled
//public class Testing extends CommandOpMode {
//
//    private final RobotHardware robot = RobotHardware.getInstance();
//
//    private GamepadEx driver;
//
//    private DrivetrainSubsystem driveSS;
//    private ElevatorSubsystem elevatorSS;
//    private IntakeSubsystem intakeSS;
//
//    private Subsystems subsystems;
//
//
//    private RobotCentricPID robotCentricPID;
//
//    private SetHeadingTarget north;
//    private SetHeadingTarget east;
//    private SetHeadingTarget south;
//    private SetHeadingTarget west;
//
//    private MoveElevatorToPosition eleUp;
//    private MoveElevatorToPosition eleDown;
//
//    private SetIntakePower intakeIn;
//    private SetIntakePower intakeIdle;
//    private SetIntakePower intakeOut;
//
//
//    protected double loopTime;
//    protected RollingAverage loopAvg;
//
//
//    @Override
//    public void initialize() {
//        robot.init(hardwareMap);
//
//        driver = new GamepadEx(gamepad1);
//
//        driveSS = new DrivetrainSubsystem(robot);
//        elevatorSS = new ElevatorSubsystem(robot);
//        intakeSS = new IntakeSubsystem(robot);
//
//        //subsystems = new Subsystems(driveSS, elevatorSS, intakeSS);
//
//
//        robotCentricPID = new RobotCentricPID(
//                driveSS,
//                () -> driver.getLeftX(),
//                () -> driver.getLeftY(),
//                () -> driver.getRightX()
//        );
//
//        north = new SetHeadingTarget(driveSS, Math.toRadians(0));
//        east = new SetHeadingTarget(driveSS, Math.toRadians(90));
//        south = new SetHeadingTarget(driveSS, Math.toRadians(180));
//        west = new SetHeadingTarget(driveSS, Math.toRadians(-90));
//
//        eleUp = new MoveElevatorToPosition(elevatorSS, 8);
//        eleDown = new MoveElevatorToPosition(elevatorSS, 4);
//
//        intakeIn = new SetIntakePower(intakeSS, 1);
//        intakeIdle = new SetIntakePower(intakeSS, 0);
//        intakeOut = new SetIntakePower(intakeSS, -1);
//
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                        .whenPressed(eleDown);
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                        .whenPressed(eleUp);
//
////        driver.getGamepadButton(GamepadKeys.Button.A)
////                .whileActiveContinuous(south);
////
////        driver.getGamepadButton(GamepadKeys.Button.B)
////                .whileActiveContinuous(east);
////
////        driver.getGamepadButton(GamepadKeys.Button.Y)
////                .whileActiveContinuous(north);
////
////        driver.getGamepadButton(GamepadKeys.Button.X)
////                .whileActiveContinuous(west);
//
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenActive(intakeOut)
//                .whenInactive(intakeIdle);
//
//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenActive(intakeIn)
//                .whenInactive(intakeIdle);
//
//
//        loopAvg = new RollingAverage(50);
//
//        robotCentricPID.schedule();
//
//        telemetry.addLine("ready:");
//        telemetry.update();
//    }
//
//    @Override
//    public void run() {
//        super.run();
//        robot.read(subsystems);
//
//        robot.loop(subsystems);
//        robot.write(subsystems);
//
//        telemetry.addData("ele target power", elevatorSS.getEleTargetPower());
//        telemetry.addData("ele actual power", elevatorSS.getEleActualPower());
//        telemetry.addData("ele target", elevatorSS.getEleGoal());
//        telemetry.addData("ele setpoint", elevatorSS.getEleSetPoint());
//        telemetry.addData("ele pos", elevatorSS.getElePos());
//        telemetry.addLine();
//        telemetry.addData("heading", driveSS.getHeading());
//        telemetry.addData("theta target", driveSS.getThetaTarget());
//        telemetry.addLine();
//        telemetry.addData("loop time", System.currentTimeMillis() - loopTime);
//        telemetry.addData("loop avg", loopAvg.getAverage());
//        loopAvg.addNumber((int) (System.currentTimeMillis() - loopTime));
//        loopTime = System.currentTimeMillis();
//        telemetry.update();
//
//        robot.clearBulkCache();
//    }
//}
