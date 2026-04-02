# rmuc_config.py

class RMUCConfig:
    def __init__(self, team_color='red', is_test_field=True):
        self.team_color = team_color.lower()
        self.is_test_field = is_test_field
        
        # ==========================================
        # 1. 实验室测试场地 (在 Rviz 中打点获取的坐标)
        # ==========================================
        self.test_field = {
            'blue': {
                'outpost_guard': {'x': 8.08, 'y': 9.73},  # 模拟前哨站防守位
                'base_guard': {'x': 8.08, 'y': 9.73},     # 模拟基地防守位
                'heal_zone': {'x': 8.08, 'y': 9.73},      # 模拟补血/补弹点
                'patrol_route': [{'x': 4.87, 'y': 2.14}]
            },
            'red': {
                'outpost_guard': {'x': 8.08, 'y': 9.73},  
                'base_guard': {'x': 8.08, 'y': 9.73},     
                'heal_zone': {'x': 8.08, 'y': 9.73},
                'patrol_route': [{'x': 4.87, 'y': 2.14}]
            }
        }

        # ==========================================
        # 2. RMUC 真实比赛场地 (相对于 Nav2 的 Map 原点)
        # ==========================================
        self.rmuc_field = {
            'blue': {
                'outpost_guard': {'x': 9.0, 'y': 11.0}, 
                'base_guard': {'x': 2.0, 'y': 13.0},    
                'heal_zone': {'x': 1.0, 'y': 14.0},     
                'patrol_route': [{'x': 5.0, 'y': 11.0}, {'x': 9.0, 'y': 9.0}, {'x': 6.0, 'y': 13.0}]
            },
            'red': {
                'outpost_guard': {'x': 19.0, 'y': 4.0}, 
                'base_guard': {'x': 26.0, 'y': 2.0},    
                'heal_zone': {'x': 27.0, 'y': 1.0},     
                'patrol_route': [{'x': 23.0, 'y': 4.0}, {'x': 19.0, 'y': 6.0}, {'x': 22.0, 'y': 2.0}]
            }
        }

    def get_tactical_points(self):
        """根据当前模式返回对应的坐标集"""
        field_data = self.test_field if self.is_test_field else self.rmuc_field
        return field_data.get(self.team_color, field_data['red'])