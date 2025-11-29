use serde::{Serialize, Deserialize};
use crate::robot::Robot;
use crate::grid::WarehouseGrid;
use std::time::{Instant, Duration};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PhysicsParams {
    pub dt: f32,         // 固定时间步（秒），60FPS 对应 1/60
    pub damping: f32,    // 速度阻尼（0.0-1.0），模拟摩擦
    pub max_velocity: f32,// 最大速度，避免运动过快
    pub restitution: f32,// 碰撞恢复系数（0.0-1.0），模拟弹性
}

impl Default for PhysicsParams {
    fn default() -> Self {
        Self {
            dt: 1.0 / 60.0,    // 60FPS，平衡平滑性和性能
            damping: 0.95,     // 适中阻尼，运动自然减速
            max_velocity: 2.0, // 最大速度（网格单位/秒）
            restitution: 0.3,  // 轻微弹性，碰撞后不卡顿
        }
    }
}

pub struct PhysicsSimulator {
    params: PhysicsParams,
}

impl PhysicsSimulator {
    pub fn new(params: PhysicsParams) -> Self {
        Self { params }
    }

    // 更新所有机器人状态（Verlet 积分 + 碰撞检测）
    pub fn update_robots(&self, robots: &mut [Robot], grid: &WarehouseGrid) {
        // 1. Verlet 积分更新位置
        for robot in robots.iter_mut() {
            self.update_robot_position(robot);
        }

        // 2. 碰撞检测（机器人-障碍物 + 机器人-机器人）
        self.handle_collisions(robots, grid);

        // 3. 速度限制和阻尼
        for robot in robots.iter_mut() {
            self.apply_velocity_limits(robot);
            self.apply_damping(robot);
        }
    }

    // Verlet 积分更新位置：x(t+dt) = 2x(t) - x(t-dt) + a*dt²
    fn update_robot_position(&self, robot: &mut Robot) {
        let (x, y) = robot.pos;
        let (prev_x, prev_y) = robot.prev_pos;
        let (acc_x, acc_y) = robot.acceleration;

        // 计算新位置
        let new_x = 2.0 * x - prev_x + acc_x * self.params.dt.powi(2);
        let new_y = 2.0 * y - prev_y + acc_y * self.params.dt.powi(2);

        // 更新前一位置和当前位置
        robot.prev_pos = (x, y);
        robot.pos = (new_x, new_y);

        // 计算当前速度（用于后续限制和阻尼）
        robot.velocity = (
            (new_x - prev_x) / self.params.dt,
            (new_y - prev_y) / self.params.dt,
        );
    }

    // 碰撞检测：机器人-障碍物 + 机器人-机器人
    fn handle_collisions(&self, robots: &mut [Robot], grid: &WarehouseGrid) {
        let robot_count = robots.len();

        // 1. 机器人-障碍物碰撞
        for robot in robots.iter_mut() {
            let (x, y) = robot.pos;
            let (radius_x, radius_y) = (robot.radius / 2.0, robot.radius / 2.0);

            // 检查四个方向的障碍物（简化版，避免穿墙）
            let check_points = [
                (x - radius_x, y), (x + radius_x, y),
                (x, y - radius_y), (x, y + radius_y),
            ];

            for (cx, cy) in check_points {
                let (cx_usize, cy_usize) = (cx.round() as usize, cy.round() as usize);
                if grid.is_obstacle(cx_usize, cy_usize) {
                    // 碰撞后反弹（反向移动）
                    robot.pos = robot.prev_pos;
                    robot.velocity = (-robot.velocity.0 * self.params.restitution, -robot.velocity.1 * self.params.restitution);
                    break;
                }
            }
        }

        // 2. 机器人-机器人碰撞（基于距离检测）
        for i in 0..robot_count {
            for j in (i + 1)..robot_count {
                let (robot_a, robot_b) = robots.split_at_mut(j);
                let robot_a = &mut robot_a[i];
                let robot_b = &mut robot_b[0];

                let dx = robot_b.pos.0 - robot_a.pos.0;
                let dy = robot_b.pos.1 - robot_a.pos.1;
                let distance = (dx.powi(2) + dy.powi(2)).sqrt();
                let min_distance = robot_a.radius + robot_b.radius;

                if distance < min_distance {
                    // 碰撞后分离并反弹
                    let overlap = min_distance - distance;
                    let separation_x = (dx / distance) * overlap / 2.0;
                    let separation_y = (dy / distance) * overlap / 2.0;

                    // 分离两个机器人
                    robot_a.pos.0 -= separation_x;
                    robot_a.pos.1 -= separation_y;
                    robot_b.pos.0 += separation_x;
                    robot_b.pos.1 += separation_y;

                    // 反弹速度（基于恢复系数）
                    let vel_a = robot_a.velocity;
                    let vel_b = robot_b.velocity;

                    robot_a.velocity = (
                        vel_a.0 * (1.0 - self.params.restitution),
                        vel_a.1 * (1.0 - self.params.restitution),
                    );
                    robot_b.velocity = (
                        vel_b.0 * (1.0 - self.params.restitution),
                        vel_b.1 * (1.0 - self.params.restitution),
                    );
                }
            }
        }
    }

    // 应用最大速度限制
    fn apply_velocity_limits(&self, robot: &mut Robot) {
        let speed = (robot.velocity.0.powi(2) + robot.velocity.1.powi(2)).sqrt();
        if speed > self.params.max_velocity {
            let scale = self.params.max_velocity / speed;
            robot.velocity.0 *= scale;
            robot.velocity.1 *= scale;
        }
    }

    // 应用速度阻尼（模拟摩擦）
    fn apply_damping(&self, robot: &mut Robot) {
        robot.velocity.0 *= self.params.damping;
        robot.velocity.1 *= self.params.damping;
    }
}

// 固定时间步仿真循环
pub struct SimulationLoop {
    last_update: Instant,
    accumulator: Duration,
    params: PhysicsParams,
}

impl SimulationLoop {
    pub fn new(params: PhysicsParams) -> Self {
        Self {
            last_update: Instant::now(),
            accumulator: Duration::ZERO,
            params,
        }
    }

    // 运行仿真循环，每帧调用更新逻辑
    pub fn run<F>(&mut self, mut update_fn: F)
    where
        F: FnMut(f32),
    {
        let now = Instant::now();
        let elapsed = now.duration_since(self.last_update);
        self.last_update = now;
        self.accumulator += elapsed;

        // 固定时间步更新（确保仿真速度一致）
        let dt_duration = Duration::from_secs_f32(self.params.dt);
        while self.accumulator >= dt_duration {
            update_fn(self.params.dt);
            self.accumulator -= dt_duration;
        }
    }
}