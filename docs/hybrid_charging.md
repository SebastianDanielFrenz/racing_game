# Hybrid charging

Tuning / Engine / Electric drive / Charging power sets mechanical ICE generating power,
from 5 to 200 kW (default 50 kW). Generator losses and electric traction reduce
net battery charging power. The controller chooses a feasible RPM/load point and
minimizes modeled fuel consumption; unavailable fuel estimates use the lowest
feasible RPM. Engine, generator and battery limits cap the achievable power.

Upgrades / Electric drive / Generator offers stock, performance, heavy-duty and
high-output generators. The conversion's stock generator is 50 kW / 200 Nm /
30 kg. Upgrades are 100 / 150 / 200 kW, with 400 / 600 / 800 Nm generating
torque, 45 / 60 / 75 kg mass and proportionally larger rotor inertia. These are
development assumptions. The chosen charging power remains a separate tuning
setting. Generator upgrades retain the traction motors and the battery.

The traction-battery generator is distinct from a conventional low-voltage
alternator. This menu upgrades the crank generator used by the hybrid model.

Implementation and approximation limits: physics_sim docs/electric_drive.md,
section Disconnected hybrid charging power. Validation covers power targets,
generator and battery limits, speed stability, fuel estimates and disconnected
engine isolation from wheel traction control.
