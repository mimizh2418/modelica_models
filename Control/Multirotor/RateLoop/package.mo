within Control.Multirotor;
package RateLoop "Multirotor body-rate control"

  function bodyMoment
    "Body moment from rate feedback, trajectory acceleration, and gyroscopic feedforward"
    input Real angularVelocitySetpoint[3](each unit = "rad/s")
      "Commanded body angular velocity";
    input Real angularVelocity[3](each unit = "rad/s")
      "Measured body angular velocity";
    input Real inertia[3](each unit = "kg.m2")
      "Diagonal body inertia {ixx, iyy, izz}";
    input Real rateGain[3](each unit = "1/s")
      "Proportional body-rate bandwidth per axis";
    input Boolean gyroscopicFeedforward = true
      "Add omega x (I omega) so the moment holds a spinning body on rate";
    input Real angularAccelerationFeedforward[3](each unit = "rad/s2") = zeros(3)
      "Reference angular acceleration rotated into the actual body frame";
    input Real angularVelocityFeedforward[3](each unit = "rad/s") = zeros(3)
      "Reference angular velocity rotated into the actual body frame";
    output Real moment[3](each unit = "N.m") "Commanded body moment";
  protected
    Real angularMomentum[3](each unit = "kg.m2/s");
    Real transportedAcceleration[3](each unit = "rad/s2");
  algorithm
    // For Q = R_actual^T R_reference, d(Q omega_reference)/dt is
    // Q alpha_reference - omega_actual x (Q omega_reference).
    transportedAcceleration := angularAccelerationFeedforward
      - cross(angularVelocity, angularVelocityFeedforward);
    for axis in 1:3 loop
      moment[axis] := inertia[axis] * (
        rateGain[axis] * (angularVelocitySetpoint[axis] - angularVelocity[axis])
          + transportedAcceleration[axis]);
    end for;
    if gyroscopicFeedforward then
      angularMomentum := {
        inertia[1] * angularVelocity[1],
        inertia[2] * angularVelocity[2],
        inertia[3] * angularVelocity[3]};
      moment := moment + cross(angularVelocity, angularMomentum);
    end if;
    annotation(Documentation(info="<html>
      <p>Maps a body-rate error into a body moment for a rigid multirotor:
      <code>M = I (k (omega_sp - omega) + alpha_ff - omega x omega_ff)
      + omega x (I omega)</code>. Feedforward derivatives are rotated into
      the actual body frame by guidance; this function applies their frame
      transport term. Zero derivative inputs preserve the original law. The
      gyroscopic term is a feed-forward that cancels the rigid-body coupling so
      the proportional term only has to close the tracking error. Inputs and the
      returned moment share the vehicle body frame.</p>
    </html>"));
  end bodyMoment;

  annotation(Documentation(info="<html>
    <p>Inner body-rate loop for multirotors. It converts the angular-velocity
    setpoint produced by an attitude loop into the body moment that the control
    allocator distributes across the rotors.</p>
  </html>"));
end RateLoop;
