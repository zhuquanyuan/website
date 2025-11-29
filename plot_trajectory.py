import pandas as pd
import matplotlib.pyplot as plt

# 读取 CSV 日志
df = pd.read_csv("robot_trajectory.csv")

# 绘制每个机器人的轨迹
plt.figure(figsize=(10, 10))
for robot_id in df["robot_id"].unique():
    robot_data = df[df["robot_id"] == robot_id]
    plt.plot(robot_data["x_pos"], robot_data["y_pos"], label=f"Robot {int(robot_id)}")
    # 标记起点和终点
    plt.scatter(robot_data["x_pos"].iloc[0], robot_data["y_pos"].iloc[0], color="green", s=100, zorder=5)
    plt.scatter(robot_data["x_pos"].iloc[-1], robot_data["y_pos"].iloc[-1], color="red", s=100, zorder=5)

# 添加网格和图例
plt.grid(True)
plt.legend()
plt.xlabel("X Position (Grid Units)")
plt.ylabel("Y Position (Grid Units)")
plt.title("Robot Movement Trajectory in Smart Warehouse")
plt.axis("equal")
plt.savefig("robot_trajectory.png")
plt.show()