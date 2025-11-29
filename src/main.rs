mod grid;
mod robot;
mod physics;
mod pathfinding;

use crate::grid::WarehouseGrid;
use crate::robot::Robot;
use crate::physics::{PhysicsSimulator, PhysicsParams, SimulationLoop};
use crate::pathfinding::find_path;
use serde::{Serialize, Deserialize};
use serde_json::to_string_pretty;
use std::fs::write;
use std::io::Write;

// 仿真状态（用于序列化输出）
#[derive(Serialize, Deserialize)]
pub struct SimulationState {
    pub time: f32,
    pub grid: WarehouseGrid,
    pub robots: Vec<Robot>,
}

fn main() {
    // 初始化日志
    env_logger::init();

    // 1. 创建网格（15x15，与用户输出一致）
    let mut grid = WarehouseGrid::new(15, 15);
    // 添加障碍物（用户输出中的布局）
    for x in 4..11 { grid.add_obstacle(x, 7).unwrap(); }
    for y in 3..6 { grid.add_obstacle(7, y).unwrap(); }
    for y in 9..12 { grid.add_obstacle(7, y).unwrap(); }
    // 添加 spawn 点和交付点
    grid.add_spawn_point(0, 0).unwrap();
    grid.add_spawn_point(0, 14).unwrap();
    grid.add_delivery_point(14, 14, 1).unwrap();
    grid.add_delivery_point(14, 0, 2).unwrap();

    println!("Warehouse Grid:\n{}", grid);

    // 2. 初始化机器人（从 spawn 点出发）
    let spawn_points = grid.get_spawn_points();
    let mut robots = Vec::new();
    for (i, &(spawn_x, spawn_y)) in spawn_points.iter().enumerate() {
        let mut robot = Robot::new((i + 1) as u32, spawn_x, spawn_y);
        // 为每个机器人分配目标（交叉交付）
        let target = if i == 0 { (14, 14) } else { (14, 0) };
        // 路径规划（A* 算法）
        if let Some(path) = find_path(&grid, (spawn_x, spawn_y), target) {
            println!("Robot {} path found: {:?}", robot.id, path);
            robot.set_path(path);
        } else {
            eprintln!("Robot {} failed to find path!", robot.id);
        }
        robots.push(robot);
    }

    // 3. 初始化物理模拟器
    let physics_params = PhysicsParams::default();
    let simulator = PhysicsSimulator::new(physics_params.clone());

    // 4. 初始化仿真循环（固定时间步）
    let mut sim_loop = SimulationLoop::new(physics_params);

    // 5. 仿真配置
    let mut time = 0.0;
    let max_time = 60.0; // 最大仿真时间（秒）
    let log_interval = 1.0; // 日志输出间隔（秒）
    let mut next_log_time = 0.0;

    // 初始化 CSV 日志（记录机器人轨迹）
    let mut csv_log = std::fs::File::create("robot_trajectory.csv").unwrap();
    writeln!(csv_log, "time,robot_id,x_pos,y_pos,velocity_x,velocity_y").unwrap();

    println!("\nStarting simulation...");

    // 6. 运行仿真循环
    sim_loop.run(|dt| {
        // 更新机器人路径跟踪（计算加速度）
        for robot in robots.iter_mut() {
            robot.move_along_path();
        }

        // 更新物理状态（Verlet 积分 + 碰撞检测）
        simulator.update_robots(&mut robots, &grid);

        // 记录 CSV 日志
        for robot in &robots {
            writeln!(
                csv_log,
                "{:.2},{},{:.2},{:.2},{:.2},{:.2}",
                time, robot.id, robot.pos.0, robot.pos.1, robot.velocity.0, robot.velocity.1
            ).unwrap();
        }

        // 生成状态快照（JSON）
        if time >= next_log_time {
            let state = SimulationState {
                time,
                grid: grid.clone(),
                robots: robots.clone(),
            };
            let state_json = to_string_pretty(&state).unwrap();
            write(format!("simulation_state_{:.0}.json", time), state_json).unwrap();
            next_log_time += log_interval;
        }

        // 打印实时状态（每 0.5 秒）
        if (time % 0.5).abs() < 0.01 {
            println!("\nTime: {:.2}s", time);
            for robot in &robots {
                println!("{}", robot);
            }
        }

        // 更新时间
        time += dt;
        if time >= max_time {
            println!("\nSimulation finished (max time reached)!");
            std::process::exit(0);
        }
    });
}