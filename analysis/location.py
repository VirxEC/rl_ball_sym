import json
import matplotlib.pyplot as plt

with open("accuracy.json") as f:
    data = json.load(f)

rocketsim = [[], [], []]
rl_ball_sym = [[], [], []]

for ball in data["rocketsim"]:
    rocketsim[0].append(ball["location"][0])
    rocketsim[1].append(ball["location"][1])
    rocketsim[2].append(ball["location"][2])

for ball in data["rl_ball_sym"]:
    rl_ball_sym[0].append(ball["location"][0])
    rl_ball_sym[1].append(ball["location"][1])
    rl_ball_sym[2].append(ball["location"][2])

ax = plt.figure().add_subplot(projection='3d')

ax.plot(rocketsim[0], rocketsim[1], rocketsim[2], label="rocketsim")
ax.plot(rl_ball_sym[0], rl_ball_sym[1], rl_ball_sym[2], label="rl_ball_sym")

ax.legend()
ax.set_title("Location")
ax.set_xlabel("X")
ax.set_ylabel("Z")
ax.set_zlabel("Y")

plt.show()
