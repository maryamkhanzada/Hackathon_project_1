"""
Isaac Gym Humanoid Walking Training Example
Trains a PPO policy for bipedal locomotion
"""

from isaacgym import gymapi
from isaacgym import gymutil
from isaacgym import gymtorch
import torch
import numpy as np

class HumanoidTrainer:
    def __init__(self, num_envs=4096):
        self.num_envs = num_envs
        self.gym = gymapi.acquire_gym()

        # Configure simulation
        sim_params = gymapi.SimParams()
        sim_params.dt = 1.0 / 60.0
        sim_params.substeps = 2
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

        # PhysX parameters
        sim_params.physx.solver_type = 1
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 1
        sim_params.physx.use_gpu = True
        sim_params.use_gpu_pipeline = True

        # Create sim
        self.sim = self.gym.create_sim(
            0, 0, gymapi.SIM_PHYSX, sim_params
        )

        # Load humanoid asset
        asset_root = "../../assets"
        asset_file = "mjcf/nv_humanoid.xml"
        asset_options = gymapi.AssetOptions()
        asset_options.angular_damping = 0.01
        asset_options.linear_damping = 0.01
        asset_options.max_angular_velocity = 100.0
        asset_options.density = 1000.0
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_NONE

        self.humanoid_asset = self.gym.load_asset(
            self.sim, asset_root, asset_file, asset_options
        )

        # Get DOF properties
        self.num_dof = self.gym.get_asset_dof_count(self.humanoid_asset)
        self.num_bodies = self.gym.get_asset_rigid_body_count(self.humanoid_asset)

        # Create environments
        self.envs = []
        self.actors = []

        env_spacing = 2.0
        envs_per_row = int(np.sqrt(num_envs))
        env_lower = gymapi.Vec3(-env_spacing, -env_spacing, 0.0)
        env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

        for i in range(num_envs):
            env = self.gym.create_env(
                self.sim, env_lower, env_upper, envs_per_row
            )

            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0.0, 0.0, 1.34)  # Start height
            pose.r = gymapi.Quat(0.0, 0.0, 0.0, 1.0)

            actor = self.gym.create_actor(
                env, self.humanoid_asset, pose, "humanoid", i, 1
            )

            self.envs.append(env)
            self.actors.append(actor)

        # Prepare tensors
        self.gym.prepare_sim(self.sim)

        # Get state tensors
        self.root_states = self.gym.acquire_actor_root_state_tensor(self.sim)
        self.dof_states = self.gym.acquire_dof_state_tensor(self.sim)
        self.rb_states = self.gym.acquire_rigid_body_state_tensor(self.sim)

        # Convert to torch tensors
        self.root_states = gymtorch.wrap_tensor(self.root_states)
        self.dof_states = gymtorch.wrap_tensor(self.dof_states)
        self.rb_states = gymtorch.wrap_tensor(self.rb_states)

        # Observation and action buffers
        self.obs_buf = torch.zeros((num_envs, 76), dtype=torch.float32, device='cuda:0')
        self.actions = torch.zeros((num_envs, self.num_dof), dtype=torch.float32, device='cuda:0')
        self.rewards = torch.zeros(num_envs, dtype=torch.float32, device='cuda:0')
        self.resets = torch.zeros(num_envs, dtype=torch.bool, device='cuda:0')

    def compute_observations(self):
        """Compute observation vector"""
        # Root orientation (quaternion)
        root_quat = self.root_states[:, 3:7]

        # Root linear and angular velocity
        root_lin_vel = self.root_states[:, 7:10]
        root_ang_vel = self.root_states[:, 10:13]

        # DOF positions and velocities
        dof_pos = self.dof_states[:, 0]
        dof_vel = self.dof_states[:, 1]

        # Concatenate observations
        self.obs_buf = torch.cat([
            root_quat,        # 4
            root_lin_vel,     # 3
            root_ang_vel,     # 3
            dof_pos,          # num_dof
            dof_vel,          # num_dof
            self.actions,     # num_dof (previous actions)
        ], dim=-1)

        return self.obs_buf

    def compute_rewards(self):
        """Compute reward for each environment"""
        # Target forward velocity
        target_vel = 1.0  # m/s

        # Forward velocity reward
        vel_reward = torch.exp(-2.0 * torch.abs(self.root_states[:, 7] - target_vel))

        # Upright reward (penalize tilting)
        up_vec = torch.tensor([0.0, 0.0, 1.0], device='cuda:0')
        heading = self.root_states[:, 3:7]  # quaternion
        up_proj = torch.sum(up_vec * heading[:, :3], dim=1)  # z-component
        up_reward = torch.square(up_proj)

        # Energy penalty (encourage efficiency)
        energy_penalty = -0.05 * torch.sum(torch.square(self.actions), dim=1)

        # Survival bonus
        alive_reward = 2.0

        # Total reward
        self.rewards = vel_reward + 2.0 * up_reward + energy_penalty + alive_reward

        # Reset if fallen
        self.resets = self.root_states[:, 2] < 0.5  # Torso below 50cm

    def reset_envs(self, env_ids):
        """Reset specified environments"""
        if len(env_ids) == 0:
            return

        # Reset positions
        self.root_states[env_ids, :3] = torch.tensor([0.0, 0.0, 1.34], device='cuda:0')
        self.root_states[env_ids, 3:7] = torch.tensor([0.0, 0.0, 0.0, 1.0], device='cuda:0')
        self.root_states[env_ids, 7:] = 0.0

        # Reset DOF states
        self.dof_states[env_ids, :] = 0.0

        # Apply resets
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(
            self.sim, gymtorch.unwrap_tensor(self.root_states),
            gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32)
        )
        self.gym.set_dof_state_tensor_indexed(
            self.sim, gymtorch.unwrap_tensor(self.dof_states),
            gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32)
        )

    def step(self, actions):
        """Step simulation with actions"""
        self.actions = actions.clone()

        # Apply actions as PD targets
        self.gym.set_dof_position_target_tensor(
            self.sim, gymtorch.unwrap_tensor(self.actions)
        )

        # Step simulation
        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)

        # Compute observations and rewards
        obs = self.compute_observations()
        self.compute_rewards()

        # Reset fallen robots
        reset_env_ids = self.resets.nonzero(as_tuple=False).flatten()
        if len(reset_env_ids) > 0:
            self.reset_envs(reset_env_ids)

        return obs, self.rewards, self.resets, {}

# Training loop (pseudo-code, integrate with RL library)
if __name__ == "__main__":
    trainer = HumanoidTrainer(num_envs=4096)

    # Initialize policy (use rl_games or stable-baselines3)
    from rl_games.torch_runner import Runner

    config = {
        'params': {
            'algo': {'name': 'a2c_continuous'},
            'model': {'name': 'continuous_a2c_logstd'},
            'network': {
                'name': 'actor_critic',
                'separate': False,
                'mlp': {
                    'units': [512, 256, 128],
                    'activation': 'elu',
                }
            },
            'config': {
                'name': 'humanoid',
                'env_name': 'rlgpu',
                'ppo': True,
                'normalize_input': True,
                'gamma': 0.99,
                'learning_rate': 3e-4,
                'max_epochs': 10000,
            }
        }
    }

    runner = Runner()
    runner.load(config)
    runner.run({'train': True})
