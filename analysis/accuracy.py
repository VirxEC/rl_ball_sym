import json
from pathlib import Path

import matplotlib.pyplot as plt

# run `cargo r --example accuracy --features=serde` first!

with open(Path(__file__).parent / "accuracy.json") as f:
    data = json.load(f)

rocketsim = [[], [], [], []]
rl_ball_sym = [[], [], [], []]

for ball in data["rocketsim"]:
    rocketsim[0].append(ball["time"])
    rocketsim[1].append(ball["location"])
    rocketsim[2].append(ball["velocity"])
    rocketsim[3].append(ball["angular_velocity"])

for ball in data["rl_ball_sym"]:
    rl_ball_sym[0].append(ball["time"])
    rl_ball_sym[1].append(ball["location"])
    rl_ball_sym[2].append(ball["velocity"])
    rl_ball_sym[3].append(ball["angular_velocity"])

start_time = 0 * 120
# time_limit = 250
time_limit = len(rocketsim[0])

# start_time = 7 * 120
# time_limit = 8 * 120

location_diff = []

for i in range(start_time, time_limit):
    location_diff.append(
        (
            (rocketsim[1][i][0] - rl_ball_sym[1][i][0]) ** 2
            + (rocketsim[1][i][1] - rl_ball_sym[1][i][1]) ** 2
            + (rocketsim[1][i][2] - rl_ball_sym[1][i][2]) ** 2
        )
        ** 0.5
    )

    if location_diff[-1] > 0:
        print(rl_ball_sym[0][i])

velocity_diff = []

for i in range(start_time, time_limit):
    velocity_diff.append(
        (
            (rocketsim[2][i][0] - rl_ball_sym[2][i][0]) ** 2
            + (rocketsim[2][i][1] - rl_ball_sym[2][i][1]) ** 2
            + (rocketsim[2][i][2] - rl_ball_sym[2][i][2]) ** 2
        )
        ** 0.5
    )

angular_velocity_diff = []

for i in range(start_time, time_limit):
    angular_velocity_diff.append(
        (
            (rocketsim[3][i][0] - rl_ball_sym[3][i][0]) ** 2
            + (rocketsim[3][i][1] - rl_ball_sym[3][i][1]) ** 2
            + (rocketsim[3][i][2] - rl_ball_sym[3][i][2]) ** 2
        )
        ** 0.5
    )

ax = plt.figure().add_subplot()

ax.plot(rocketsim[0][start_time:time_limit], location_diff, label="location")
ax.plot(rocketsim[0][start_time:time_limit], velocity_diff, label="velocity")
ax.plot(rocketsim[0][start_time:time_limit], angular_velocity_diff, label="angular velocity")

ax.legend()
ax.set_title("Time vs Difference")
ax.set_xlabel("Time")
ax.set_ylabel("Magnitude of difference")

plt.show()

