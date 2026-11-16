import numpy as np

from robot_arm_trajectory.time_parameterization import (
    generate_quintic_time_scaling
)


# Generate a 4-second trajectory
profile = generate_quintic_time_scaling(
    duration=4.0,
    num_samples=101
)


print("\n========== QUINTIC TIME SCALING TEST ==========\n")

print("Trajectory duration:", profile.time[-1], "seconds")

print("Number of samples:", len(profile.time))

print("\nInitial scaling:", profile.s[0])
print("Final scaling:", profile.s[-1])

print("\nInitial velocity:", profile.velocity[0])
print("Final velocity:", profile.velocity[-1])

print("\nInitial acceleration:", profile.acceleration[0])
print("Final acceleration:", profile.acceleration[-1])

print("\nMaximum scaling velocity:",
      np.max(profile.velocity))

print("Maximum absolute scaling acceleration:",
      np.max(np.abs(profile.acceleration)))


# Validate boundary conditions
assert np.isclose(profile.s[0], 0.0)
assert np.isclose(profile.s[-1], 1.0)

assert np.isclose(profile.velocity[0], 0.0)
assert np.isclose(profile.velocity[-1], 0.0)

assert np.isclose(profile.acceleration[0], 0.0)
assert np.isclose(profile.acceleration[-1], 0.0)

assert np.all(np.diff(profile.s) >= 0)

print("\nBoundary conditions: PASSED")
print("Monotonic progression: PASSED")
print("\nOVERALL VALIDATION: PASSED")
