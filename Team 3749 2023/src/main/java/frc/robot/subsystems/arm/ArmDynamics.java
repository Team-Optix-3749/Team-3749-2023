// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.utils.Constants;

/**
 * Converts between the system state and motor voltages for a double jointed
 * arm.
 *
 * <p>
 * https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 *
 * <p>
 * https://www.chiefdelphi.com/t/double-jointed-arm-physics-control-simulator/424307
 */
public class ArmDynamics {
    private static final double g = 9.80665;

    private final DCMotor shoulderDCMotor = DCMotor.getNEO(1).withReduction(250);
    private final DCMotor elbowDCMotor = DCMotor.getNEO(1).withReduction(200);

    public ArmDynamics() {
    }

    /** Calculates the joint voltages based on the joint positions (feedforward). */
    public Vector<N2> feedforward(Vector<N2> position) {
        return feedforward(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
    }

    /**
     * Calculates the joint voltages based on the full joint states as a matrix
     * (feedforward). The
     * rows represent each joint and the columns represent position, velocity, and
     * acceleration.
     */
    public Vector<N2> feedforward(Matrix<N2, N3> state) {
        return feedforward(
                new Vector<>(state.extractColumnVector(0)),
                new Vector<>(state.extractColumnVector(1)),
                new Vector<>(state.extractColumnVector(2)));
    }

    /**
     * Calculates the joint voltages based on the full joint states as vectors
     * (feedforward).
     */
    public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
        var torque = M(position)
                .times(acceleration)
                .plus(C(position, velocity).times(velocity))
                .plus(Tg(position));
        return VecBuilder.fill(
                shoulderDCMotor.getVoltage(torque.get(0, 0), velocity.get(0, 0)),
                elbowDCMotor.getVoltage(torque.get(1, 0), velocity.get(1, 0)));
    }

    private Matrix<N2, N2> M(Vector<N2> position) {
        var M = new Matrix<>(N2.instance, N2.instance);
        M.set(
                0,
                0,
                Constants.Arm.shoulder_mass * Math.pow(Constants.Arm.shoulder_cg_radius, 2.0)
                        + Constants.Arm.elbow_mass * (Math.pow(Constants.Arm.shoulder_length, 2.0)
                                + Math.pow(Constants.Arm.elbow_cg_radius, 2.0))
                        + Constants.Arm.shoulder_moi
                        + Constants.Arm.elbow_moi
                        + 2
                                * Constants.Arm.elbow_mass
                                * Constants.Arm.shoulder_length
                                * Constants.Arm.elbow_cg_radius
                                * Math.cos(position.get(1, 0)));
        M.set(
                1,
                0,
                Constants.Arm.elbow_mass * Math.pow(Constants.Arm.elbow_cg_radius, 2.0)
                        + Constants.Arm.elbow_moi
                        + Constants.Arm.elbow_mass * Constants.Arm.shoulder_length * Constants.Arm.elbow_cg_radius
                                * Math.cos(position.get(1, 0)));
        M.set(
                0,
                1,
                Constants.Arm.elbow_mass * Math.pow(Constants.Arm.elbow_cg_radius, 2.0)
                        + Constants.Arm.elbow_moi
                        + Constants.Arm.elbow_mass * Constants.Arm.shoulder_length * Constants.Arm.elbow_cg_radius
                                * Math.cos(position.get(1, 0)));
        M.set(1, 1, Constants.Arm.elbow_mass * Math.pow(Constants.Arm.elbow_cg_radius, 2.0) + Constants.Arm.elbow_moi);
        return M;
    }

    private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
        var C = new Matrix<>(N2.instance, N2.instance);
        C.set(
                0,
                0,
                -Constants.Arm.elbow_mass
                        * Constants.Arm.shoulder_length
                        * Constants.Arm.elbow_cg_radius
                        * Math.sin(position.get(1, 0))
                        * velocity.get(1, 0));
        C.set(
                1,
                0,
                Constants.Arm.elbow_mass
                        * Constants.Arm.shoulder_length
                        * Constants.Arm.elbow_cg_radius
                        * Math.sin(position.get(1, 0))
                        * velocity.get(0, 0));
        C.set(
                0,
                1,
                -Constants.Arm.elbow_mass
                        * Constants.Arm.shoulder_length
                        * Constants.Arm.elbow_cg_radius
                        * Math.sin(position.get(1, 0))
                        * (velocity.get(0, 0) + velocity.get(1, 0)));
        return C;
    }

    private Matrix<N2, N1> Tg(Vector<N2> position) {
        var Tg = new Matrix<>(N2.instance, N1.instance);
        Tg.set(
                0,
                0,
                (Constants.Arm.shoulder_mass * Constants.Arm.shoulder_cg_radius
                        + Constants.Arm.elbow_mass * Constants.Arm.shoulder_length)
                        * g
                        * Math.cos(position.get(0, 0))
                        + Constants.Arm.elbow_mass
                                * Constants.Arm.elbow_cg_radius
                                * g
                                * Math.cos(position.get(0, 0) + position.get(1, 0)));
        Tg.set(
                1,
                0,
                Constants.Arm.elbow_mass * Constants.Arm.elbow_cg_radius * g
                        * Math.cos(position.get(0, 0) + position.get(1, 0)));
        return Tg;
    }
}