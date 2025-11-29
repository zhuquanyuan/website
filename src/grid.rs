use serde::{Serialize, Deserialize};
use std::fmt;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub enum CellType {
    Empty,
    Obstacle,
    SpawnPoint,
    DeliveryPoint(u32), // 交付点关联包裹ID
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Cell {
    pub cell_type: CellType,
    pub x: usize,
    pub y: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WarehouseGrid {
    pub width: usize,
    pub height: usize,
    pub cells: Vec<Vec<Cell>>,
}

impl WarehouseGrid {
    // 创建空网格
    pub fn new(width: usize, height: usize) -> Self {
        let cells = (0..height)
            .map(|y| (0..width)
                .map(|x| Cell { cell_type: CellType::Empty, x, y })
                .collect())
            .collect();
        Self { width, height, cells }
    }

    // 添加障碍物
    pub fn add_obstacle(&mut self, x: usize, y: usize) -> Result<(), String> {
        if x >= self.width || y >= self.height {
            return Err("坐标超出网格范围".to_string());
        }
        self.cells[y][x].cell_type = CellType::Obstacle;
        Ok(())
    }

    // 添加 spawn 点
    pub fn add_spawn_point(&mut self, x: usize, y: usize) -> Result<(), String> {
        if x >= self.width || y >= self.height {
            return Err("坐标超出网格范围".to_string());
        }
        self.cells[y][x].cell_type = CellType::SpawnPoint;
        Ok(())
    }

    // 添加交付点
    pub fn add_delivery_point(&mut self, x: usize, y: usize, package_id: u32) -> Result<(), String> {
        if x >= self.width || y >= self.height {
            return Err("坐标超出网格范围".to_string());
        }
        self.cells[y][x].cell_type = CellType::DeliveryPoint(package_id);
        Ok(())
    }

    // 检查是否为障碍物
    pub fn is_obstacle(&self, x: usize, y: usize) -> bool {
        if x >= self.width || y >= self.height {
            return true; // 边界视为障碍物
        }
        matches!(self.cells[y][x].cell_type, CellType::Obstacle)
    }

    // 获取所有 spawn 点
    pub fn get_spawn_points(&self) -> Vec<(usize, usize)> {
        self.cells.iter()
            .flat_map(|row| row.iter())
            .filter(|cell| matches!(cell.cell_type, CellType::SpawnPoint))
            .map(|cell| (cell.x, cell.y))
            .collect()
    }
}

// 网格可视化
impl fmt::Display for WarehouseGrid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for row in &self.cells {
            for cell in row {
                let symbol = match cell.cell_type {
                    CellType::Empty => ". ",
                    CellType::Obstacle => "# ",
                    CellType::SpawnPoint => "S ",
                    CellType::DeliveryPoint(_) => "D ",
                };
                write!(f, "{}", symbol)?;
            }
            writeln!(f)?;
        }
        Ok(())
    }
}

// 测试用例
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_grid_creation() {
        let grid = WarehouseGrid::new(5, 5);
        assert_eq!(grid.width, 5);
        assert_eq!(grid.height, 5);
    }

    #[test]
    fn test_obstacle_detection() {
        let mut grid = WarehouseGrid::new(5, 5);
        grid.add_obstacle(2, 2).unwrap();
        assert!(grid.is_obstacle(2, 2));
        assert!(!grid.is_obstacle(0, 0));
    }
}