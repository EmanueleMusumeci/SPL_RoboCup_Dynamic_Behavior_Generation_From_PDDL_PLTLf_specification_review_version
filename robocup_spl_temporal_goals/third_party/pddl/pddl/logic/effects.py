# -*- coding: utf-8 -*-
#
# Copyright 2021 WhiteMech
#
# ------------------------------
#
# This file is part of pddl.
#
# pddl is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# pddl is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with pddl.  If not, see <https://www.gnu.org/licenses/>.
#

"""This modules implements PDDL effects."""
import functools
from typing import AbstractSet, Collection, Generic, Optional, Sequence, TypeVar, Union

from third_party.pddl.pddl.helpers.base import ensure_set
from third_party.pddl.pddl.helpers.cache_hash import cache_hash
from third_party.pddl.pddl.logic import Variable
from third_party.pddl.pddl.logic.base import Atomic, Formula, Not, OneOf, MonotoneOp, BinaryOp
from third_party.pddl.pddl.parser.symbols import Symbols

EffectType = TypeVar("EffectType")


@cache_hash
@functools.total_ordering
class AndEffect(Generic[EffectType]):
    """Conjunction of effects."""

    def __init__(self, *operands: EffectType):
        """
        Initialize a conjunction of (conditional) effects.

        :param operands: the operands.
        """
        self._operands = list(operands)

    @property
    def operands(self) -> Sequence[EffectType]:
        """Get the operands."""
        return tuple(self._operands)

    def __str__(self) -> str:
        """Get the string representation."""
        return f"({Symbols.AND.value} {' '.join(map(str, self.operands))})"

    def __repr__(self) -> str:
        """Get an unambiguous string representation."""
        return f"{type(self).__name__}({repr(self._operands)})"

    def __eq__(self, other):
        """Compare with another object."""
        return isinstance(other, type(self)) and self.operands == other.operands

    def __lt__(self, other) -> bool:
        """Compare with another object."""
        if isinstance(other, AndEffect):
            return tuple(self.operands) < tuple(other.operands)
        return super().__lt__(other)  # type: ignore

    def __hash__(self) -> int:
        """Compute the hash of the object."""
        return hash((type(self), self.operands))


@cache_hash
@functools.total_ordering
class When:
    """Represent the 'When' effect."""

    def __init__(self, condition: Formula, effect: "CondEffect") -> None:
        """
        Initialize the effect.

        :param condition: the condition
        :param effect: the effect
        """
        self._condition = condition
        self._effect = effect

    @property
    def condition(self) -> Formula:
        """Get the condition."""
        return self._condition

    @property
    def effect(self) -> "CondEffect":
        """Get the effect."""
        return self._effect

    def __str__(self) -> str:
        """Get the string representation."""
        return f"({Symbols.WHEN.value} {self._condition} {self.effect})"

    def __repr__(self) -> str:
        """Get an unambiguous string representation."""
        return f"{type(self).__name__}({self.condition}, {self.effect})"

    def __eq__(self, other) -> bool:
        """Compare with another object."""
        return (
            isinstance(other, type(self))
            and self.condition == other.condition
            and self.effect == other.effect
        )

    def __hash__(self) -> int:
        """Compute the hash of the object."""
        return hash((type(self), self.condition, self.effect))

    def __lt__(self, other):
        """Compare with another object."""
        if isinstance(other, When):
            return (self.condition, self.effect) < (other.condition, other.effect)
        return super().__lt__(other)


@cache_hash
@functools.total_ordering
class Forall:
    """Represent the 'Forall' effect."""

    def __init__(
        self, effect: "Effect", variables: Optional[Collection[Variable]] = None
    ) -> None:
        """Initialize the 'forall' effect."""
        self._effect = effect
        self._variables = ensure_set(variables)

    @property
    def effect(self) -> "Effect":
        """Get the effect."""
        return self._effect

    @property
    def variables(self) -> AbstractSet[Variable]:
        """Get the variables."""
        return self._variables

    def __str__(self) -> str:
        """Get the string representation."""
        return f"({Symbols.FORALL.value} ({' '.join(map(str, self.variables))}) {self.effect})"

    def __repr__(self) -> str:
        """Get an unambiguous string representation."""
        return f"{type(self).__name__}({self.variables}, {self.effect})"

    def __eq__(self, other) -> bool:
        """Compare with another object."""
        return (
            isinstance(other, type(self))
            and self.variables == other.variables
            and self.effect == other.effect
        )

    def __hash__(self) -> int:
        """Compute the hash of the object."""
        return hash((type(self), self.variables, self.effect))

    def __lt__(self, other):
        """Compare with another object."""
        if isinstance(other, Forall):
            return (self.variables, self.effect) < (other.variables, other.effect)
        return super().__lt__(other)


PEffect = Union[Atomic, Not]
CEffect = Union[Forall, When, OneOf, "PEffect"]
Effect = Union[AndEffect["CEffect"], CEffect]
CondEffect = Union[AndEffect["PEffect"], "PEffect"]
