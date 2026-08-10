# Reference semantics specification

Step references jump causally at the frozen issue offset. Minimum-jerk references use the classical quintic with analytic position, velocity, acceleration, and jerk. Approach-stop and waypoint trajectories use a septic endpoint-rest profile whose first three derivatives vanish at stops. Waypoint geometry is a deterministic function of trajectory_seed. References are evaluated analytically at each 1000 Hz physics tick and expose no preview.
