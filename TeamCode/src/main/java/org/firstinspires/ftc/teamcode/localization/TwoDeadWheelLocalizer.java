package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double parYTicks = -11733.408316759904; // y position of the parallel encoder (in tick units)
        public double perpXTicks = -12355.225445972377; // x position of the perpendicular encoder (in tick units)
    }

    public static Params PARAMS = new Params();
    public final Encoder par, perp;
    public final double inPerTick;
    private int lastParPos, lastPerpPos;
    private boolean initialized;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront"))); // Port 3 Control Hub
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "intake")));    // Port 0 Expansion Hub

        par.setDirection(DcMotorSimple.Direction.REVERSE); // Adjust based on actual setup

        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parPosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

//            lastParPos = parPosVel.position;
//            lastPerpPos = perpPosVel.position;

//            return new Twist2dDual<>(
//                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
//                    DualNum.constant(0.0, 2)
//            );
//        }

//        int parDelta = parPosVel.position - lastParPos;
//        int perpDelta = perpPosVel.position - lastPerpPos;

//        Twist2dDual<Time> twist = new Twist2dDual<>(
//                new Vector2dDual<>(
//                        new DualNum<>(new double[]{
//                                parDelta * inPerTick,
//                                parPosVel.velocity * inPerTick
//                        }),
//                        new DualNum<>(new double[]{
//                                perpDelta * inPerTick,
//                                perpPosVel.velocity * inPerTick
//                        })
//                ),
//                new DualNum<>(new double[]{
//                        0.0, // Heading change cannot be calculated from 2 encoders
//                        0.0
//                })
//        );
//
//        lastParPos = parPosVel.position;
//        lastPerpPos = perpPosVel.position;
//
//        return twist;
        }
        return null;
    }
}