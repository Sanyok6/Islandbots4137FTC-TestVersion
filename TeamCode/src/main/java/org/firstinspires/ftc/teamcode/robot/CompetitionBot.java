package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class CompetitionBot {

    // motors
    public DcMotor RFmotor, RBmotor, LFmotor, LBmotor;

    // constants
    public static final double MAX_SPEED = .7;

    public CompetitionBot(HardwareMap hwMap, Telemetry telemetry) {
        // motors
        RFmotor = hwMap.dcMotor.get("RFmotor");
        RBmotor = hwMap.dcMotor.get("RBmotor");
        LFmotor = hwMap.dcMotor.get("LFmotor");
        LBmotor = hwMap.dcMotor.get("LBmotor");

        telemetry.addData("Successfully Initialized", null);
        telemetry.update();
    }

    public double[] tankMove(double joystickMove, double joystickTurn, Telemetry telemetry) {
        double SPEED_REDUCTION = MAX_SPEED;

        double L = joystickMove * joystickMove + (0.5*joystickTurn);
        double R = joystickMove * joystickMove - (0.5*joystickTurn);

        L *= SPEED_REDUCTION;
        R *= SPEED_REDUCTION;

        double[] powerVals = {L, R};

        L *= clamp(powerVals);
        R *= clamp(powerVals);

        setMotors(L, R);

        return powerVals;
    }

    public void setMotors(double L, double R) {
        LFmotor.setPower(L);
        LBmotor.setPower(R);
    }

    private double clamp(double[] powerVals) {
        double max = Math.max(Math.max(powerVals[0], powerVals[1]), Math.max(powerVals[2], powerVals[3]));
        if (max > 1) {
            return 1 / max;
        } else {
            return 1;
        }
    }

}
