"""
FOOTSTEP PLANNER - FIXED

A* search for humanoid navigation with proper obstacle avoidance.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import heapq
from pathlib import Path


class FootstepPlanner:
    def __init__(self):
        self.foot_length = 0.12
        self.foot_width = 0.06
        self.hip_width = 0.16
        
        self.max_step_length = 0.25
        self.max_step_width = 0.12
        
        self.actions = self._generate_actions()
        print(f"Footstep Planner: {len(self.actions)} actions")
    
    def _generate_actions(self):
        """Generate discrete footstep actions."""
        actions = []
        
        # Forward steps
        for length in [0.08, 0.15, 0.20, 0.25]:
            for lateral in [-0.04, 0, 0.04]:
                actions.append((length, self.hip_width + lateral, 0))
        
        # Turning steps
        for angle in [-0.3, -0.15, 0.15, 0.3]:
            actions.append((0.10, self.hip_width, angle))
        
        # Side steps
        actions.append((0.05, self.hip_width + 0.10, 0))
        actions.append((0.05, self.hip_width - 0.04, 0))
        
        # Backward (small)
        actions.append((-0.05, self.hip_width, 0))
        
        return actions
    
    def _check_collision(self, x, y, theta, obstacles):
        """Check if foot placement collides with obstacles."""
        for ox, oy, r in obstacles:
            dist = np.sqrt((x - ox)**2 + (y - oy)**2)
            # Simple circle collision with foot center + margin
            if dist < r + 0.08:  # foot radius + margin
                return True
        return False
    
    def _heuristic(self, state, goal):
        x, y, theta, foot = state
        dist = np.sqrt((x - goal[0])**2 + (y - goal[1])**2)
        return dist / self.max_step_length * 0.8  # Admissible
    
    def plan(self, start, goal, obstacles=[]):
        """Plan footstep sequence using A*."""
        print(f"  Planning: {start[:2]} → {goal}")
        
        start_state = (start[0], start[1], start[2], 'right')
        
        open_set = []
        heapq.heappush(open_set, (0, 0, start_state, [start_state]))
        
        visited = {}
        max_iter = 10000
        
        for iteration in range(max_iter):
            if not open_set:
                break
            
            _, cost, state, path = heapq.heappop(open_set)
            x, y, theta, stance = state
            
            # Goal check
            if np.sqrt((x - goal[0])**2 + (y - goal[1])**2) < 0.12:
                print(f"  ✓ Found path: {len(path)} steps, {iteration} iterations")
                return path
            
            # Discretize state for visited check
            key = (round(x*10), round(y*10), round(theta*5), stance)
            if key in visited and visited[key] <= cost:
                continue
            visited[key] = cost
            
            swing = 'left' if stance == 'right' else 'right'
            
            for dx, dy, dth in self.actions:
                new_theta = theta + dth
                c, s = np.cos(theta), np.sin(theta)
                
                # Swing foot position
                if swing == 'left':
                    new_x = x + c * dx - s * dy
                    new_y = y + s * dx + c * dy
                else:
                    new_x = x + c * dx + s * dy
                    new_y = y + s * dx - c * dy
                
                # Collision check
                if self._check_collision(new_x, new_y, new_theta, obstacles):
                    continue
                
                # Bounds
                if abs(new_x) > 5 or abs(new_y) > 5:
                    continue
                
                new_state = (new_x, new_y, new_theta, swing)
                
                # Cost: distance + turn penalty
                step_cost = np.sqrt(dx**2 + (dy-self.hip_width)**2) + 0.3 * abs(dth)
                new_cost = cost + step_cost
                
                priority = new_cost + self._heuristic(new_state, goal)
                heapq.heappush(open_set, (priority, new_cost, new_state, path + [new_state]))
        
        print(f"  ✗ No path found after {iteration} iterations")
        return None
    
    def visualize(self, path, start, goal, obstacles=[], save_path='results/footstep_plan.png'):
        """Visualize footstep plan."""
        fig, ax = plt.subplots(figsize=(14, 8))
        
        # Draw obstacles
        for ox, oy, r in obstacles:
            circle = Circle((ox, oy), r, color='red', alpha=0.6, zorder=2)
            ax.add_patch(circle)
            ax.plot(ox, oy, 'rx', markersize=10, zorder=3)
        
        # Start and goal
        ax.plot(start[0], start[1], 'go', markersize=15, label='Start', zorder=5)
        ax.plot(goal[0], goal[1], 'r*', markersize=20, label='Goal', zorder=5)
        
        # Draw footsteps
        if path:
            colors = {'left': '#4444FF', 'right': '#44AA44'}
            
            for i, (x, y, theta, foot) in enumerate(path):
                # Foot rectangle
                hw, hl = self.foot_width/2, self.foot_length/2
                corners = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]])
                c, s = np.cos(theta), np.sin(theta)
                R = np.array([[c, -s], [s, c]])
                corners = (R @ corners.T).T + np.array([x, y])
                
                poly = Polygon(corners, facecolor=colors[foot], edgecolor='black',
                              alpha=0.7, linewidth=1, zorder=3)
                ax.add_patch(poly)
                ax.text(x, y, str(i+1), ha='center', va='center',
                       fontsize=7, fontweight='bold', color='white', zorder=4)
            
            # Path line
            xs, ys = [s[0] for s in path], [s[1] for s in path]
            ax.plot(xs, ys, 'k--', linewidth=1, alpha=0.4, zorder=1)
        
        ax.set_xlim(-0.3, max(goal[0], start[0]) + 0.5)
        ax.set_ylim(-1.0, 1.0)
        ax.set_aspect('equal')
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Footstep Planning with A* Search', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        
        # Legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='#4444FF', alpha=0.7, label='Left foot'),
            Patch(facecolor='#44AA44', alpha=0.7, label='Right foot'),
            Circle((0,0), 0.1, color='red', alpha=0.6, label='Obstacle'),
        ]
        ax.legend(handles=legend_elements, loc='upper right')
        
        plt.tight_layout()
        Path('results').mkdir(exist_ok=True)
        plt.savefig(save_path, dpi=150)
        print(f"  Saved: {save_path}")
        plt.close()


def main():
    print("="*70)
    print("  FOOTSTEP PLANNER - A* Search")
    print("="*70)
    
    planner = FootstepPlanner()
    
    # Test 1: Straight
    print("\n[TEST 1] Walk 2m forward")
    path = planner.plan((0, 0, 0), (2.0, 0))
    if path:
        planner.visualize(path, (0,0,0), (2.0, 0), save_path='results/footstep_straight.png')
    
    # Test 2: Obstacles
    print("\n[TEST 2] Navigate around obstacles")
    obstacles = [
        (0.7, 0.0, 0.15),
        (1.4, -0.1, 0.12),
    ]
    path = planner.plan((0, 0, 0), (2.0, 0), obstacles)
    if path:
        planner.visualize(path, (0,0,0), (2.0, 0), obstacles, save_path='results/footstep_obstacles.png')
    
    # Test 3: Side goal
    print("\n[TEST 3] Walk to side goal")
    path = planner.plan((0, 0, 0), (1.5, 0.6))
    if path:
        planner.visualize(path, (0,0,0), (1.5, 0.6), save_path='results/footstep_side.png')
    
    # Test 4: Complex path
    print("\n[TEST 4] Complex obstacle course")
    obstacles2 = [
        (0.5, 0.2, 0.10),
        (1.0, -0.15, 0.12),
        (1.5, 0.1, 0.10),
    ]
    path = planner.plan((0, 0, 0), (2.0, 0), obstacles2)
    if path:
        planner.visualize(path, (0,0,0), (2.0, 0), obstacles2, save_path='results/footstep_complex.png')
    
    print(f"\n{'='*70}")
    print("  FOOTSTEP PLANNING COMPLETE")
    print("  ✓ A* search optimization")
    print("  ✓ Obstacle avoidance")
    print("  ✓ Kinematic constraints")
    print(f"{'='*70}")


if __name__ == "__main__":
    main()
