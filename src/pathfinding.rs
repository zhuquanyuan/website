// src/pathfinding.rs
use crate::grid::WarehouseGrid;
use pathfinding::prelude::astar;

/// 在网格中查找从起点到终点的路径
/// 使用 A* 算法，返回路径点的向量（包含起点和终点）
pub fn find_path(
    grid: &WarehouseGrid,
    start: (usize, usize),
    end: (usize, usize),
) -> Option<Vec<(usize, usize)>> {
    // 检查起点或终点是否是障碍物
    if grid.is_obstacle(start.0, start.1) || grid.is_obstacle(end.0, end.1) {
        return None;
    }

    // A* 算法实现
    astar(
        &start,
        // 邻居节点生成器：上下左右四个方向
        |&(x, y)| {
            let mut neighbors = Vec::new();
            for &(dx, dy) in &[(-1, 0), (1, 0), (0, -1), (0, 1)] {
                let nx = x as i32 + dx;
                let ny = y as i32 + dy;
                // 检查邻居是否在网格内且不是障碍物
                if nx >= 0 && nx < grid.width as i32 && ny >= 0 && ny < grid.height as i32 {
                    let nx_usize = nx as usize;
                    let ny_usize = ny as usize;
                    if !grid.is_obstacle(nx_usize, ny_usize) {
                        // 移动成本为 1
                        neighbors.push(((nx_usize, ny_usize), 1));
                    }
                }
            }
            neighbors
        },
        // 启发函数：曼哈顿距离
        |&(x, y)| {
            (x as i32 - end.0 as i32).abs() + (y as i32 - end.1 as i32).abs()
        },
        // 目标节点检查
        |&pos| pos == end,
    )
    // 只返回路径，忽略总成本
    .map(|(path, _cost)| path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::WarehouseGrid;

    #[test]
    fn test_find_path_clear() {
        let grid = WarehouseGrid::new(5, 5);
        let path = find_path(&grid, (0, 0), (4, 4)).unwrap();
        assert!(!path.is_empty());
        assert_eq!(path.first(), Some(&(0, 0)));
        assert_eq!(path.last(), Some(&(4, 4)));
    }

    #[test]
    fn test_find_path_blocked() {
        let mut grid = WarehouseGrid::new(5, 5);
        // 创建一道横向障碍物
        for x in 0..5 {
            grid.add_obstacle(x, 2).unwrap();
        }
        // 路径应该能绕过去
        let path = find_path(&grid, (0, 0), (4, 4));
        assert!(path.is_some());
        let path = path.unwrap();
        // 路径中不应包含障碍物所在的行 (y=2)
        assert!(!path.iter().any(|&(_, y)| y == 2));
    }
}