from collections.abc import Callable
from dataclasses import dataclass, field

from balancer.control import Command, Controller
from balancer.sim import FALLEN_PITCH, Sample, Simulation


@dataclass(frozen=True)
class Push:
    time: float
    force: float
    duration: float = 0.1


@dataclass(frozen=True)
class Scenario:
    name: str
    duration: float
    command_at: Callable[[float], Command] = lambda _: Command()
    pushes: tuple[Push, ...] = ()
    initial_pitch: float = 0.05


@dataclass
class ScenarioResult:
    scenario: Scenario
    samples: list[Sample] = field(default_factory=list)

    @property
    def fell(self) -> bool:
        return any(abs(sample.pitch) > FALLEN_PITCH for sample in self.samples)


def _drive_command(time: float) -> Command:
    return Command(forward_velocity=0.5 if 2.0 <= time < 10.0 else 0.0)


STAND = Scenario(name="stand", duration=30.0)
PUSH = Scenario(name="push", duration=10.0, pushes=(Push(time=3.0, force=6.0),))
DRIVE = Scenario(name="drive", duration=14.0, command_at=_drive_command)

SCENARIOS: dict[str, Scenario] = {scenario.name: scenario for scenario in (STAND, PUSH, DRIVE)}


def run_scenario(
    scenario: Scenario,
    controller: Controller,
    on_step: Callable[[Simulation], None] | None = None,
) -> ScenarioResult:
    simulation = Simulation(controller, initial_pitch=scenario.initial_pitch)
    result = ScenarioResult(scenario)
    pending = sorted(scenario.pushes, key=lambda push: push.time)
    while simulation.time < scenario.duration:
        while pending and simulation.time >= pending[0].time:
            push = pending.pop(0)
            simulation.push(push.force, push.duration)
        result.samples.append(simulation.step(scenario.command_at(simulation.time)))
        if on_step is not None:
            on_step(simulation)
    return result
