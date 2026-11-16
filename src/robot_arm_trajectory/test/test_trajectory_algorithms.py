import numpy as np
import pytest
from types import SimpleNamespace

from robot_arm_trajectory.joint_space.joint_trajectory import (
    generate_joint_trajectory,
)
from robot_arm_trajectory.interpolation import (
    LinearInterpolation, CubicSplineInterpolation,
    QuinticPolynomialInterpolation,
)
from robot_arm_trajectory.time_parameterization import TIME_PARAMETERIZERS
from robot_arm_trajectory.time_parameterization import (
    generate_quintic_time_scaling, SCurveProfile, SeventhOrderProfile,
    TrapezoidalProfile,
)


@pytest.mark.parametrize("name", ["quintic_scaling", "seventh_order"])
def test_profiles_rest_to_rest(name):
    profile = TIME_PARAMETERIZERS[name]().generate(2.0, 101)
    assert np.isclose(profile.s[0], 0) and np.isclose(profile.s[-1], 1)
    assert np.isclose(profile.velocity[0], 0) and np.isclose(profile.velocity[-1], 0)
    assert np.isclose(profile.acceleration[0], 0) and np.isclose(profile.acceleration[-1], 0)
    assert np.all(np.diff(profile.s) >= -1e-12)


@pytest.mark.parametrize("cls", [LinearInterpolation, CubicSplineInterpolation, QuinticPolynomialInterpolation])
def test_interpolators_pass_waypoints(cls):
    q = np.array([[0., 1.], [1., -1.], [2., 2.]])
    itp = cls().fit(q)
    assert np.allclose(itp.evaluate(np.linspace(0, 1, 3)), q)


@pytest.mark.parametrize('cls', [
    LinearInterpolation,
    CubicSplineInterpolation,
    QuinticPolynomialInterpolation,
])
def test_interpolation_parameter_domain_and_shapes(cls):
    """All interpolators accept the closed domain and preserve input shape."""
    q = np.array([[0.0, 1.0], [0.5, -0.25], [1.0, 0.5]])
    parameters = np.array([-0.2, 0.3, 1.4])
    interpolation = cls().fit(q, parameters)

    cases = (
        (parameters[0], q[0]),
        (0.1, None),
        (parameters[-1], q[-1]),
    )
    for value, expected in cases:
        result = interpolation.evaluate(value)
        assert result.shape == (2,)
        if expected is not None:
            assert np.allclose(result, expected)

    values = interpolation.evaluate(np.array([-0.2, 0.1, 1.4]))
    assert values.shape == (3, 2)
    grid = np.array([[-0.2, 0.1], [0.8, 1.4]])
    assert interpolation.evaluate(grid).shape == (2, 2, 2)

    for derivative in (0, 1, 2, 3):
        invalid_values = (-0.200001, 1.400001, np.array([-0.2, 1.400001]))
        for invalid in invalid_values:
            with pytest.raises(ValueError, match='interpolation domain'):
                interpolation.evaluate(invalid, derivative=derivative)
        for invalid in (np.nan, np.inf, -np.inf):
            with pytest.raises(ValueError, match='finite'):
                interpolation.evaluate(invalid, derivative=derivative)

    for derivative in (1, 2, 3):
        assert interpolation.evaluate(0.1, derivative).shape == (2,)
        result = interpolation.evaluate(np.array([[0.1, 0.5]]), derivative)
        assert result.shape == (1, 2, 2)


@pytest.mark.parametrize("interp", ["linear", "cubic_spline", "quintic_polynomial"])
@pytest.mark.parametrize("timing", list(TIME_PARAMETERIZERS))
def test_generator_combinations(interp, timing):
    q = np.array([[0., 1.], [0.5, -0.2], [1., 0.]])
    traj = generate_joint_trajectory(q, 2.0, 51, interp, timing)
    assert np.allclose(traj.positions[[0, -1]], q[[0, -1]])
    assert traj.positions.shape == traj.velocities.shape == traj.accelerations.shape


def test_limits_and_invalid_input():
    with pytest.raises(ValueError):
        generate_joint_trajectory([[0], [np.nan]])
    with pytest.raises(ValueError, match="position limits"):
        generate_joint_trajectory([[0], [2]], joint_position_limits=[[-1], [1]],
                                  enforce_position_limits=True)
    with pytest.raises(ValueError, match="unavailable"):
        generate_joint_trajectory([[0], [1]], enforce_acceleration_limits=True)


def test_analytic_interpolation_derivative():
    q = np.array([[0.], [1.], [0.]])
    itp = CubicSplineInterpolation().fit(q)
    x = 0.37; h = 1e-5
    numeric = (itp.evaluate(x+h) - itp.evaluate(x-h)) / (2*h)
    assert np.allclose(numeric, itp.evaluate(x, 1), atol=1e-7)


def test_quintic_hermite_defaults_are_global_spline_derivatives():
    q = np.array([[0.0], [0.3], [0.2], [0.8], [1.0]])
    s = np.array([0.0, 0.12, 0.41, 0.77, 1.0])
    from scipy.interpolate import CubicSpline
    reference = CubicSpline(s, q, axis=0, bc_type="not-a-knot")
    hermite = QuinticPolynomialInterpolation().fit(q, s)
    assert np.allclose(hermite.v, reference(s, 1), atol=1e-12)
    assert np.allclose(hermite.a, reference(s, 2), atol=1e-12)
    assert np.allclose(hermite.evaluate(s), q, atol=1e-12)
    dense = np.linspace(0.0, 1.0, 301)
    cubic = CubicSplineInterpolation().fit(q, s)
    assert np.allclose(hermite.evaluate(dense), cubic.evaluate(dense), atol=2e-11)
    for boundary in s[1:-1]:
        eps = 1e-8
        for derivative in (0, 1, 2):
            left = hermite.evaluate(boundary - eps, derivative)
            right = hermite.evaluate(boundary + eps, derivative)
            assert np.allclose(left, right, atol=2e-5)


def test_quintic_hermite_analytic_derivatives_match_finite_difference():
    q = np.array([[0.0], [0.4], [-0.1], [0.9]])
    itp = QuinticPolynomialInterpolation().fit(q)
    x, h = 0.37, 2e-5
    d1 = (itp.evaluate(x+h) - itp.evaluate(x-h)) / (2*h)
    d2 = (itp.evaluate(x+h) - 2*itp.evaluate(x) + itp.evaluate(x-h)) / h**2
    assert np.allclose(d1, itp.evaluate(x, 1), rtol=2e-7, atol=2e-7)
    assert np.allclose(d2, itp.evaluate(x, 2), rtol=2e-5, atol=2e-5)


def test_time_scaling_chain_rule_matches_finite_differences():
    q = np.array([[0.0], [0.3], [-0.2], [0.8]])
    traj = generate_joint_trajectory(
        q, duration=2.0, num_samples=4001,
        interpolation_method="quintic_polynomial")
    h = traj.time[1] - traj.time[0]
    qd = np.gradient(traj.positions, h, axis=0, edge_order=2)
    qdd = np.gradient(traj.velocities, h, axis=0, edge_order=2)
    qddd = np.gradient(traj.accelerations, h, axis=0, edge_order=2)
    assert np.allclose(qd[10:-10], traj.velocities[10:-10], rtol=8e-3, atol=2e-6)
    assert np.allclose(qdd[10:-10], traj.accelerations[10:-10], rtol=3e-2, atol=2e-4)
    assert np.allclose(qddd[20:-20], traj.jerks[20:-20], rtol=2e-1, atol=3e-3)


def test_derivative_limit_checks_between_output_samples():
    # A coarse output grid with rest endpoints misses an interior peak.
    q = np.array([[0.0], [1.0]])
    with pytest.raises(ValueError, match="continuous-domain extrema estimate"):
        generate_joint_trajectory(
            q, duration=4.0, num_samples=3,
            interpolation_method="linear",
            joint_velocity_limits=[0.3], enforce_velocity_limits=True)


@pytest.mark.parametrize("vmax,amax", [(0.7, 2.0), (0.8, 1.2), (0.6, 2.0)])
def test_trapezoidal_feasible_limits(vmax, amax):
    profile = TrapezoidalProfile().generate(
        2.0, 1001, max_velocity=vmax, max_acceleration=amax)
    assert np.isclose(profile.s[0], 0.0)
    assert np.isclose(profile.s[-1], 1.0)
    assert np.isclose(profile.velocity[0], 0.0)
    assert np.isclose(profile.velocity[-1], 0.0)
    assert np.all(np.abs(profile.velocity) <= vmax + 1e-12)
    assert np.all(np.abs(profile.acceleration) <= amax + 1e-12)
    assert np.all(np.diff(profile.s) >= -1e-12)
    integrated = np.sum(0.5 * (profile.velocity[1:] + profile.velocity[:-1]) * np.diff(profile.time))
    assert np.isclose(integrated, 1.0, atol=2e-5)
    # The acceleration at the endpoints reflects the adjacent physical phase.
    assert profile.acceleration[0] > 0.0
    assert profile.acceleration[-1] < 0.0


@pytest.mark.parametrize("kwargs, message", [
    ({"max_velocity": 0.49, "max_acceleration": 2.0}, "peak velocity"),
    ({"max_velocity": 1.0, "max_acceleration": 0.99}, "acceleration"),
])
def test_trapezoidal_infeasible_limits(kwargs, message):
    with pytest.raises(ValueError, match=message):
        TrapezoidalProfile().generate(2.0, 51, **kwargs)


def test_trapezoidal_reported_phase_acceleration():
    p = TrapezoidalProfile().generate(2.0, 1001, max_velocity=0.7,
                                       max_acceleration=2.0)
    phases = np.unique(np.round(p.acceleration, 10))
    assert np.allclose(phases, [-1.225, 0.0, 1.225])


def test_seventh_order_derivatives_and_peak_jerk():
    duration = 2.3
    p = SeventhOrderProfile().generate(duration, 10001)
    u = p.time / duration
    expected_s = 35*u**4 - 84*u**5 + 70*u**6 - 20*u**7
    expected_v = (140*u**3 - 420*u**4 + 420*u**5 - 140*u**6) / duration
    expected_a = (420*u**2 - 1680*u**3 + 2100*u**4 - 840*u**5) / duration**2
    expected_j = (840*u - 5040*u**2 + 8400*u**3 - 4200*u**4) / duration**3
    assert np.allclose(p.s, expected_s)
    assert np.allclose(p.velocity, expected_v)
    assert np.allclose(p.acceleration, expected_a)
    assert np.allclose(p.jerk, expected_j)
    assert np.isclose(np.max(np.abs(p.jerk)), 52.5 / duration**3, rtol=1e-7)
    assert np.allclose([p.velocity[0], p.velocity[-1]], [0.0, 0.0], atol=1e-12)
    assert np.allclose([p.acceleration[0], p.acceleration[-1]], [0.0, 0.0], atol=1e-12)
    assert np.allclose([p.jerk[0], p.jerk[-1]], [0.0, 0.0], atol=1e-12)
    h = p.time[1] - p.time[0]
    assert np.allclose(np.gradient(p.s, h, edge_order=2)[2:-2], p.velocity[2:-2], rtol=2e-5, atol=1e-7)
    assert np.allclose(np.gradient(p.velocity, h, edge_order=2)[2:-2], p.acceleration[2:-2], rtol=2e-3, atol=1e-6)
    assert np.allclose(np.gradient(p.acceleration, h, edge_order=2)[2:-2], p.jerk[2:-2], rtol=2e-3, atol=1e-5)


def test_seventh_order_jerk_limit_and_legacy_imports():
    required = 52.5 / 3.0**3
    SeventhOrderProfile().generate(3.0, 51, max_jerk=required)
    with pytest.raises(ValueError, match="above max_jerk"):
        SeventhOrderProfile().generate(3.0, 51, max_jerk=required * 0.99)
    with pytest.warns(DeprecationWarning):
        alias = SCurveProfile().generate(3.0, 51)
    assert np.allclose(alias.s, SeventhOrderProfile().generate(3.0, 51).s)
    legacy = generate_quintic_time_scaling(2.0, 11)
    assert np.isclose(legacy.s[-1], 1.0)
    assert TIME_PARAMETERIZERS["seventh_order"] is SeventhOrderProfile
    assert TIME_PARAMETERIZERS["s_curve"] is SCurveProfile


def _model(lower=(-1.0, -1.0), upper=(1.0, 1.0), velocity=(10.0, 10.0)):
    return SimpleNamespace(dof=2, lower_limits=np.array(lower),
                           upper_limits=np.array(upper),
                           velocity_limits=np.array(velocity))


def test_position_enforcement_requires_limits_and_can_be_disabled():
    q = [[0.0, 0.0], [0.5, -0.5]]
    with pytest.raises(ValueError, match="position limits are unavailable"):
        generate_joint_trajectory(q, enforce_position_limits=True)
    # Explicitly disabled enforcement preserves legacy calls without limits.
    trajectory = generate_joint_trajectory(q, enforce_position_limits=False)
    assert np.allclose(trajectory.positions[[0, -1]], q)


def test_position_enforcement_with_valid_limits():
    q = [[0.0, 0.0], [0.5, -0.5]]
    trajectory = generate_joint_trajectory(
        q, joint_position_limits=[[-1.0, -1.0], [1.0, 1.0]],
        enforce_position_limits=True)
    assert np.all(trajectory.positions <= 1.0)
    with pytest.raises(ValueError, match="violates joint position"):
        generate_joint_trajectory(
            [[0.0, 0.0], [1.1, 0.0]],
            joint_position_limits=[[-1.0, -1.0], [1.0, 1.0]],
            enforce_position_limits=True)


def test_robot_model_position_and_velocity_constraints():
    q = [[0.0, 0.0], [0.5, -0.5]]
    model = _model(velocity=(0.1, 0.1))
    trajectory = generate_joint_trajectory(
        q, robot_model=model, enforce_position_limits=True)
    assert np.all(trajectory.positions >= model.lower_limits)
    with pytest.raises(ValueError, match="Generated velocity violates"):
        generate_joint_trajectory(
            q, robot_model=model, enforce_position_limits=True,
            enforce_velocity_limits=True)
    with pytest.raises(ValueError, match="conflict with RobotModel"):
        generate_joint_trajectory(
            q, robot_model=model,
            joint_position_limits=[[-2.0, -2.0], [2.0, 2.0]])


def test_robot_model_dof_mismatch_and_bad_limits():
    with pytest.raises(ValueError, match="DOF"):
        generate_joint_trajectory([[0.0], [0.5]], robot_model=_model())
    for bad_limits in (
        [[-1.0, np.nan], [1.0, 1.0]],
        [[1.0, -1.0], [-1.0, 1.0]],
        [[-1.0], [1.0]],
    ):
        with pytest.raises(ValueError, match="Position limits"):
            generate_joint_trajectory(
                [[0.0, 0.0], [0.1, 0.1]],
                joint_position_limits=bad_limits,
                enforce_position_limits=False)
    with pytest.raises(ValueError, match="Velocity limits"):
        generate_joint_trajectory(
            [[0.0, 0.0], [0.1, 0.1]], joint_velocity_limits=[1.0, np.inf])


def test_robot_model_rejects_nonfinite_config_and_joint_values(tmp_path):
    import yaml
    from pathlib import Path
    from robot_arm_kinematics.robot_model import RobotModel

    config = Path(__file__).parents[2] / "robot_arm_kinematics" / "robot_arm_kinematics" / "config" / "generic_robot.yaml"
    raw = yaml.safe_load(config.read_text())
    raw["robot"]["joint_limits"]["lower_deg"][0] = float("nan")
    invalid = tmp_path / "invalid.yaml"
    invalid.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="position limits must be finite"):
        RobotModel(str(invalid))

    valid = RobotModel(str(config))
    with pytest.raises(ValueError, match="finite"):
        valid.check_joint_limits([np.nan] * valid.dof)
    with pytest.raises(ValueError, match="finite"):
        valid.enforce_joint_limits([np.inf] * valid.dof)


def test_quintic_uses_explicit_waypoint_derivatives():
    q = np.array([[0.0], [1.0]])
    s = np.array([0.0, 1.0])

    velocities = np.array([[2.0], [-1.0]])
    accelerations = np.array([[3.0], [4.0]])

    interpolator = QuinticPolynomialInterpolation().fit(
        q,
        s,
        velocities=velocities,
        accelerations=accelerations,
    )

    assert np.allclose(interpolator.v, velocities)
    assert np.allclose(interpolator.a, accelerations)

    assert np.allclose(interpolator.evaluate(s, derivative=0), q)
    assert np.allclose(
        interpolator.evaluate(s, derivative=1), velocities
    )
    assert np.allclose(
        interpolator.evaluate(s, derivative=2), accelerations
    )


def test_quintic_position_extrema_with_nonuniform_intervals():
    s = np.array([0.0, 0.07, 0.63, 1.0])

    q = s**3 - 1.5 * s**2 + 0.5 * s
    velocities = 3.0 * s**2 - 3.0 * s + 0.5
    accelerations = 6.0 * s - 3.0

    interpolator = QuinticPolynomialInterpolation().fit(
        q[:, None],
        s,
        velocities=velocities[:, None],
        accelerations=accelerations[:, None],
    )

    minimum, maximum = interpolator.position_extrema()

    expected_extreme = np.sqrt(3.0) / 36.0

    assert np.allclose(minimum, [-expected_extreme], atol=1e-9)
    assert np.allclose(maximum, [expected_extreme], atol=1e-9)


def test_quintic_hermite_c2_continuity_at_all_waypoints():
    s = np.array([0.0, 0.08, 0.35, 0.72, 1.0])

    q = np.array([
        [0.0, 0.2],
        [0.3, 0.5],
        [0.7, 0.1],
        [0.4, -0.3],
        [0.9, 0.2],
    ])

    velocities = np.array([
        [0.5, 0.1],
        [0.8, -0.4],
        [-0.2, 0.6],
        [0.3, -0.5],
        [0.0, 0.2],
    ])

    accelerations = np.array([
        [0.2, -0.1],
        [0.4, 0.3],
        [-0.5, 0.2],
        [0.1, -0.4],
        [0.0, 0.0],
    ])

    interpolator = QuinticPolynomialInterpolation().fit(
        q,
        s,
        velocities=velocities,
        accelerations=accelerations,
    )

    for i, coefficients in enumerate(interpolator.coefficients):
        h = s[i + 1] - s[i]
        powers = np.arange(6)

        # Evaluate the left segment at its endpoint.
        left_position = np.sum(
            coefficients * h**powers[:, None], axis=0
        )

        left_velocity = np.sum(
            coefficients[1:] *
            (powers[1:] * h**(powers[1:] - 1))[:, None],
            axis=0,
        )

        left_acceleration = np.sum(
            coefficients[2:] *
            (powers[2:] * (powers[2:] - 1) *
             h**(powers[2:] - 2))[:, None],
            axis=0,
        )

        assert np.allclose(left_position, q[i + 1], atol=1e-10)
        assert np.allclose(left_velocity, velocities[i + 1], atol=1e-10)
        assert np.allclose(
            left_acceleration, accelerations[i + 1], atol=1e-10
        )

        # Evaluate the right segment at its starting point.
        right_position = coefficients[0]
        right_velocity = coefficients[1]
        right_acceleration = 2.0 * coefficients[2]

        assert np.allclose(right_position, q[i], atol=1e-10)
        assert np.allclose(right_velocity, velocities[i], atol=1e-10)
        assert np.allclose(
            right_acceleration, accelerations[i], atol=1e-10
        )
