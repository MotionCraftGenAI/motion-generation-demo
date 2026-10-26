"""The motion policy: a high-level controller (HLC) picks a motion latent from the observation
and heading command; a low-level controller (LLC) turns observation + latent into 28 joint
actions. Only the actor networks are here, with the exact layer structure used in training."""
import torch
import torch.nn as nn
import torch.nn.functional as F

OBS_SIZE = 228          # 223 proprioceptive values + a 5-value heading task
TASK_SIZE = 5
NUM_ACTIONS = 28
LATENT_SIZE = 64


class _Normalizer(nn.Module):
    """Observation normalisation with the statistics frozen at the end of training."""

    def __init__(self, mean: torch.Tensor, var: torch.Tensor, epsilon: float = 1e-5):
        super().__init__()
        self.register_buffer("mean", mean.float())
        self.register_buffer("std", torch.sqrt(var.float() + epsilon))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return torch.clamp((x - self.mean) / self.std, min=-5.0, max=5.0)


def _mlp(sizes: list[int]) -> nn.Sequential:
    layers: list[nn.Module] = []
    for i in range(len(sizes) - 1):
        layers += [nn.Linear(sizes[i], sizes[i + 1]), nn.ReLU()]
    return nn.Sequential(*layers)


class HighLevelController(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor_mlp = _mlp([OBS_SIZE, 1024, 512])
        self.mu = nn.Linear(512, LATENT_SIZE)

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return torch.clamp(torch.tanh(self.mu(self.actor_mlp(obs))), -1.0, 1.0)


class LowLevelController(nn.Module):
    def __init__(self):
        super().__init__()
        self.style_mlp = _mlp([LATENT_SIZE, 512, 256])
        self.style_dense = nn.Linear(256, LATENT_SIZE)
        self.dense_layers = nn.ModuleList([
            nn.Linear(OBS_SIZE - TASK_SIZE + LATENT_SIZE, 1024),
            nn.Linear(1024, 1024),
            nn.Linear(1024, 512),
        ])
        self.mu = nn.Linear(512, NUM_ACTIONS)

    def forward(self, obs: torch.Tensor, latent: torch.Tensor) -> torch.Tensor:
        style = torch.tanh(self.style_dense(self.style_mlp(latent)))
        h = torch.cat([obs, style], dim=-1)
        for dense in self.dense_layers:
            h = torch.relu(dense(h))
        return torch.clamp(self.mu(h), -1.0, 1.0)


class MotionPolicy:
    """Observation in, joint actions out. Call reset() at the start of every session."""

    def __init__(self, weights_path: str, device: str = "cpu"):
        blob = torch.load(weights_path, map_location=device, weights_only=True)
        self.device = torch.device(device)
        # The HLC decides every `llc_steps` control steps, as in training.
        self.llc_steps = int(blob["llc_steps"])

        self.hlc = HighLevelController()
        self.hlc.load_state_dict(blob["hlc"]["actor"])
        self.hlc_norm = _Normalizer(blob["hlc"]["obs_mean"], blob["hlc"]["obs_var"])

        self.llc = LowLevelController()
        self.llc.load_state_dict(blob["llc"]["actor"])
        self.llc_norm = _Normalizer(blob["llc"]["obs_mean"], blob["llc"]["obs_var"])

        for module in (self.hlc, self.hlc_norm, self.llc, self.llc_norm):
            module.to(self.device).eval()
        self.reset()

    def reset(self) -> None:
        self._step = 0
        self._latent: torch.Tensor | None = None

    @torch.no_grad()
    def act(self, obs: torch.Tensor) -> torch.Tensor:
        """obs: (N, 228) float32. Returns (N, 28) float32 actions in [-1, 1]."""
        if self._step == 0 or self._latent is None:
            self._latent = self.hlc(self.hlc_norm(obs))
        self._step = (self._step + 1) % self.llc_steps

        # The LLC does not see the heading task.
        llc_obs = self.llc_norm(obs[..., :OBS_SIZE - TASK_SIZE])
        return self.llc(llc_obs, F.normalize(self._latent, dim=-1))
