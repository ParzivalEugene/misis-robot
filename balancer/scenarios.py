from collections.abc import Callable
from dataclasses import dataclass, field

from balancer.control import Command, Controller
from balancer.sim import Sample, Simulation


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
        return any(sample.fallen for sample in self.samples)


def _drive_command(time: float) -> Command:
    return Command(forward_velocity=0.5 if 2.0 <= time < 10.0 else 0.0)


def _crouch_command(time: float) -> Command:
    if 3.0 <= time < 7.0:
        return Command(height=0.11)
    if 7.0 <= time < 11.0:
        return Command(height=0.22)
    return Command()


STAND = Scenario(name="stand", duration=30.0)
PUSH = Scenario(name="push", duration=10.0, pushes=(Push(time=3.0, force=6.0),))
DRIVE = Scenario(name="drive", duration=14.0, command_at=_drive_command)
CROUCH = Scenario(name="crouch", duration=15.0, command_at=_crouch_command)
KNOCKDOWN = Scenario(name="knockdown", duration=10.0, pushes=(Push(time=2.0, force=80.0),))

SCENARIOS: dict[str, Scenario] = {
    scenario.name: scenario for scenario in (STAND, PUSH, DRIVE, CROUCH, KNOCKDOWN)
}


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
