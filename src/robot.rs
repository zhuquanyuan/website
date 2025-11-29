use serde::{Serialize, Deserialize};
use std::fmt;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Robot {
    pub id: u32,
    pub pos: (f32, f32),         // 当前位置
    pub prev_pos: (f32, f32),    // 前一位置（Verlet 积分用）
    pub velocity: (f32, f32),    // 当前速度
    pub acceleration: (f32, f32),// 当前加速度（由路径规划控制）
    pub radius: f32,             // 机器人半径（碰撞检测用）
    pub max_velocity: f32,       // 最大速度
    pub current_target: Option<(usize, usize)>, // 当前目标点
    pub path: Vec<(usize, usize)>, // 剩余路径
}

impl Robot {
    pub fn new(id: u32, spawn_x: usize, spawn_y: usize) -> Self {
        let spawn_x = spawn_x as f32;
        let spawn_y = spawn_y as f32;
        Self {
            id,
            pos: (spawn_x, spawn_y),
            prev_pos: (spawn_x, spawn_y), // 初始前一位置 = 当前位置
            velocity: (0.0, 0.0),
            acceleration: (0.0, 0.0),
            radius: 0.4, // 机器人半径（网格单位）
            max_velocity: 2.0, // 最大速度（与物理参数一致）
            current_target: None,
            path: Vec::new(),
        }
    }

    // 设置路径并更新当前目标
    pub fn set_path(&mut self, path: Vec<(usize, usize)>) {
        self.path = path;
        self.update_current_target();
    }

    // 更新当前目标（取路径第一个点）
    fn update_current_target(&mut self) {
        self.current_target = self.path.first().copied();
    }

    // 沿路径移动：计算朝向目标的加速度
    pub fn move_along_path(&mut self) {
        if let Some((target_x, target_y)) = self.current_target {
            let target_x = target_x as f32;
            let target_y = target_y as f32;

            // 计算到目标的向量
            let dx = target_x - self.pos.0;
            let dy = target_y - self.pos.1;
            let distance = (dx.powi(2) + dy.powi(2)).sqrt();

            if distance < 0.5 { // 到达目标点（误差范围）
                self.path.remove(0); // 移除已到达的点
                self.update_current_target();
                self.acceleration = (0.0, 0.0); // 到达后减速
            } else {
                // 计算加速度（朝向目标，大小与距离成正比）
                let acc_scale = 1.0; // 加速度缩放系数
                self.acceleration = (dx / distance * acc_scale, dy / distance * acc_scale);
            }
        } else {
            self.acceleration = (0.0, 0.0); // 无路径时停止加速
        }
    }
}

// 机器人状态可视化
impl fmt::Display for Robot {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Robot {}: Pos=({:.2}, {:.2}), Vel=({:.2}, {:.2}), Target={:?}",
            self.id, self.pos.0, self.pos.1, self.velocity.0, self.velocity.1, self.current_target
        )
    }
}

// 测试用例
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_robot_creation() {
        let robot = Robot::new(1, 0, 0);
        assert_eq!(robot.id, 1);
        assert_eq!(robot.pos, (0.0, 0.0));
    }

    #[test]
    fn test_path_following() {
        let mut robot = Robot::new(1, 0, 0);
        robot.set_path(vec![(2, 0), (2, 2)]);
        assert_eq!(robot.current_target, Some((2, 0)));

        // 模拟移动到第一个目标
        robot.pos = (2.0, 0.0);
        robot.move_along_path();
        assert_eq!(robot.current_target, Some((2, 2))); // 切换到下一个目标
    }
}