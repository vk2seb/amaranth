import builtins
from collections import OrderedDict
from collections.abc import Iterable, MutableMapping, MutableSet

from .. import tracer
from ..tools import *


__all__ = [
    "Value", "Const", "Operator", "Mux", "Part", "Slice", "Cat", "Repl",
    "Signal", "ClockSignal", "ResetSignal",
    "Statement", "Assign", "Switch",
    "ValueKey", "ValueDict", "ValueSet",
]


class DUID:
    """Deterministic Unique IDentifier"""
    __next_uid = 0
    def __init__(self):
        self.duid = DUID.__next_uid
        DUID.__next_uid += 1


class Value:
    @staticmethod
    def wrap(obj):
        """Ensures that the passed object is a Migen value. Booleans and integers
        are automatically wrapped into ``Const``."""
        if isinstance(obj, Value):
            return obj
        elif isinstance(obj, (bool, int)):
            return Const(obj)
        else:
            raise TypeError("Object {} of type {} is not a Migen value"
                            .format(repr(obj), type(obj)))

    def __bool__(self):
        # Special case: Consts and Signals are part of a set or used as
        # dictionary keys, and Python needs to check for equality.
        if isinstance(self, Operator) and self.op == "==":
            a, b = self.operands
            if isinstance(a, Const) and isinstance(b, Const):
                return a.value == b.value
            if isinstance(a, Signal) and isinstance(b, Signal):
                return a is b
            if (isinstance(a, Const) and isinstance(b, Signal)
                    or isinstance(a, Signal) and isinstance(b, Const)):
                return False
        raise TypeError("Attempted to convert Migen value to boolean")

    def __invert__(self):
        return Operator("~", [self])
    def __neg__(self):
        return Operator("-", [self])

    def __add__(self, other):
        return Operator("+", [self, other])
    def __radd__(self, other):
        return Operator("+", [other, self])
    def __sub__(self, other):
        return Operator("-", [self, other])
    def __rsub__(self, other):
        return Operator("-", [other, self])
    def __mul__(self, other):
        return Operator("*", [self, other])
    def __rmul__(self, other):
        return Operator("*", [other, self])
    def __mod__(self, other):
        return Operator("%", [self, other])
    def __rmod__(self, other):
        return Operator("%", [other, self])
    def __div__(self, other):
        return Operator("/", [self, other])
    def __rdiv__(self, other):
        return Operator("/", [other, self])
    def __lshift__(self, other):
        return Operator("<<<", [self, other])
    def __rlshift__(self, other):
        return Operator("<<<", [other, self])
    def __rshift__(self, other):
        return Operator(">>>", [self, other])
    def __rrshift__(self, other):
        return Operator(">>>", [other, self])
    def __and__(self, other):
        return Operator("&", [self, other])
    def __rand__(self, other):
        return Operator("&", [other, self])
    def __xor__(self, other):
        return Operator("^", [self, other])
    def __rxor__(self, other):
        return Operator("^", [other, self])
    def __or__(self, other):
        return Operator("|", [self, other])
    def __ror__(self, other):
        return Operator("|", [other, self])

    def __eq__(self, other):
        return Operator("==", [self, other])
    def __ne__(self, other):
        return Operator("!=", [self, other])
    def __lt__(self, other):
        return Operator("<", [self, other])
    def __le__(self, other):
        return Operator("<=", [self, other])
    def __gt__(self, other):
        return Operator(">", [self, other])
    def __ge__(self, other):
        return Operator(">=", [self, other])

    def __len__(self):
        return self.bits_sign()[0]

    def __getitem__(self, key):
        n = len(self)
        if isinstance(key, int):
            if key not in range(-n, n):
                raise IndexError("Cannot index {} bits into {}-bit value".format(key, n))
            if key < 0:
                key += n
            return Slice(self, key, key + 1)
        elif isinstance(key, slice):
            start, stop, step = key.indices(n)
            if step != 1:
                return Cat(self[i] for i in range(start, stop, step))
            return Slice(self, start, stop)
        else:
            raise TypeError("Cannot index value with {}".format(repr(key)))

    def bool(self):
        """Conversion to boolean.

        Returns
        -------
        Value, out
            Output ``Value``. If any bits are set, returns ``1``, else ``0``.
        """
        return Operator("b", [self])

    def part(self, offset, width):
        """Indexed part-select.

        Selects a constant width but variable offset part of a ``Value``.

        Parameters
        ----------
        offset : Value, in
            start point of the selected bits
        width : int
            number of selected bits

        Returns
        -------
        Part, out
            Selected part of the ``Value``
        """
        return Part(self, offset, width)

    def eq(self, value):
        """Assignment.

        Parameters
        ----------
        value : Value, in
            Value to be assigned.

        Returns
        -------
        Assign
            Assignment statement that can be used in combinatorial or synchronous context.
        """
        return Assign(self, value)

    def bits_sign(self):
        """Bit length and signedness of a value.

        Returns
        -------
        int, bool
            Number of bits required to store `v` or available in `v`, followed by
            whether `v` has a sign bit (included in the bit count).

        Examples
        --------
        >>> Value.bits_sign(Signal(8))
        8, False
        >>> Value.bits_sign(C(0xaa))
        8, False
        """
        raise TypeError("Cannot calculate bit length of {!r}".format(self))

    def _lhs_signals(self):
        raise TypeError("Value {!r} cannot be used in assignments".format(self))

    def _rhs_signals(self):
        raise NotImplementedError

    def __hash__(self):
        raise TypeError("Unhashable type: {}".format(type(self).__name__))


class Const(Value):
    """A constant, literal integer value.

    Parameters
    ----------
    value : int
    bits_sign : int or tuple or None
        Either an integer `bits` or a tuple `(bits, signed)`
        specifying the number of bits in this `Const` and whether it is
        signed (can represent negative values). `bits_sign` defaults
        to the minimum width and signedness of `value`.

    Attributes
    ----------
    nbits : int
    signed : bool
    """
    def __init__(self, value, bits_sign=None):
        self.value = int(value)
        if bits_sign is None:
            bits_sign = self.value.bit_length(), self.value < 0
        if isinstance(bits_sign, int):
            bits_sign = bits_sign, self.value < 0
        self.nbits, self.signed = bits_sign
        if not isinstance(self.nbits, int) or self.nbits < 0:
            raise TypeError("Width must be a positive integer")

    def bits_sign(self):
        return self.nbits, self.signed

    def _rhs_signals(self):
        return ValueSet()

    def __eq__(self, other):
        return self.value == other.value

    def __hash__(self):
        return hash(self.value)

    def __repr__(self):
        return "(const {}'{}d{})".format(self.nbits, "s" if self.signed else "", self.value)


C = Const  # shorthand


class Operator(Value):
    def __init__(self, op, operands):
        super().__init__()
        self.op = op
        self.operands = [Value.wrap(o) for o in operands]

    @staticmethod
    def _bitwise_binary_bits_sign(a, b):
        if not a[1] and not b[1]:
            # both operands unsigned
            return max(a[0], b[0]), False
        elif a[1] and b[1]:
            # both operands signed
            return max(a[0], b[0]), True
        elif not a[1] and b[1]:
            # first operand unsigned (add sign bit), second operand signed
            return max(a[0] + 1, b[0]), True
        else:
            # first signed, second operand unsigned (add sign bit)
            return max(a[0], b[0] + 1), True

    def bits_sign(self):
        obs = list(map(lambda x: x.bits_sign(), self.operands))
        if self.op == "+" or self.op == "-":
            if len(obs) == 1:
                if self.op == "-" and not obs[0][1]:
                    return obs[0][0] + 1, True
                else:
                    return obs[0]
            n, s = self._bitwise_binary_bits_sign(*obs)
            return n + 1, s
        elif self.op == "*":
            if not obs[0][1] and not obs[1][1]:
                # both operands unsigned
                return obs[0][0] + obs[1][0], False
            elif obs[0][1] and obs[1][1]:
                # both operands signed
                return obs[0][0] + obs[1][0] - 1, True
            else:
                # one operand signed, the other unsigned (add sign bit)
                return obs[0][0] + obs[1][0] + 1 - 1, True
        elif self.op == "<<<":
            if obs[1][1]:
                extra = 2**(obs[1][0] - 1) - 1
            else:
                extra = 2**obs[1][0] - 1
            return obs[0][0] + extra, obs[0][1]
        elif self.op == ">>>":
            if obs[1][1]:
                extra = 2**(obs[1][0] - 1)
            else:
                extra = 0
            return obs[0][0] + extra, obs[0][1]
        elif self.op == "&" or self.op == "^" or self.op == "|":
            return self._bitwise_binary_bits_sign(*obs)
        elif (self.op == "<" or self.op == "<=" or self.op == "==" or self.op == "!=" or
              self.op == ">" or self.op == ">="):
            return 1, False
        elif self.op == "~":
            return obs[0]
        elif self.op == "m":
            return _bitwise_binary_bits_sign(obs[1], obs[2])
        else:
            raise TypeError

    def _rhs_signals(self):
        return union(op._rhs_signals() for op in self.operands)

    def __repr__(self):
        if len(self.operands) == 1:
            return "({} {})".format(self.op, self.operands[0])
        elif len(self.operands) == 2:
            return "({} {} {})".format(self.op, self.operands[0], self.operands[1])


def Mux(sel, val1, val0):
    """Choose between two values.

    Parameters
    ----------
    sel : Value, in
        Selector.
    val1 : Value, in
    val0 : Value, in
        Input values.

    Returns
    -------
    Value, out
        Output ``Value``. If ``sel`` is asserted, the Mux returns ``val1``, else ``val0``.
    """
    return Operator("m", [sel, val1, val0])


class Slice(Value):
    def __init__(self, value, start, end):
        if not isinstance(start, int):
            raise TypeError("Slice start must be integer, not {!r}".format(start))
        if not isinstance(end, int):
            raise TypeError("Slice end must be integer, not {!r}".format(end))

        n = len(value)
        if start not in range(-n, n):
            raise IndexError("Cannot start slice {} bits into {}-bit value".format(start, n))
        if start < 0:
            start += n
        if end not in range(-(n+1), n+1):
            raise IndexError("Cannot end slice {} bits into {}-bit value".format(end, n))
        if end < 0:
            end += n

        super().__init__()
        self.value = Value.wrap(value)
        self.start = start
        self.end   = end

    def bits_sign(self):
        return self.end - self.start, False

    def _lhs_signals(self):
        return self.value._lhs_signals()

    def _rhs_signals(self):
        return self.value._rhs_signals()

    def __repr__(self):
        return "(slice {} {}:{})".format(repr(self.value), self.start, self.end)


class Part(Value):
    def __init__(self, value, offset, width):
        if not isinstance(width, int) or width < 0:
            raise TypeError("Part width must be a positive integer, not {!r}".format(width))

        super().__init__()
        self.value  = value
        self.offset = Value.wrap(offset)
        self.width  = width

    def bits_sign(self):
        return self.width, False

    def _lhs_signals(self):
        return self.value._lhs_signals()

    def _rhs_signals(self):
        return self.value._rhs_signals()

    def __repr__(self):
        return "(part {} {})".format(repr(self.value), repr(self.offset), self.width)


class Cat(Value):
    """Concatenate values.

    Form a compound ``Value`` from several smaller ones by concatenation.
    The first argument occupies the lower bits of the result.
    The return value can be used on either side of an assignment, that
    is, the concatenated value can be used as an argument on the RHS or
    as a target on the LHS. If it is used on the LHS, it must solely
    consist of ``Signal`` s, slices of ``Signal`` s, and other concatenations
    meeting these properties. The bit length of the return value is the sum of
    the bit lengths of the arguments::

        len(Cat(args)) == sum(len(arg) for arg in args)

    Parameters
    ----------
    *args : Values or iterables of Values, inout
        ``Value`` s to be concatenated.

    Returns
    -------
    Value, inout
        Resulting ``Value`` obtained by concatentation.
    """
    def __init__(self, *args):
        super().__init__()
        self.operands = [Value.wrap(v) for v in flatten(args)]

    def bits_sign(self):
        return sum(len(op) for op in self.operands), False

    def _lhs_signals(self):
        return union(op._lhs_signals() for op in self.operands)

    def _rhs_signals(self):
        return union(op._rhs_signals() for op in self.operands)

    def __repr__(self):
        return "(cat {})".format(" ".join(map(repr, self.operands)))


class Repl(Value):
    """Replicate a value

    An input value is replicated (repeated) several times
    to be used on the RHS of assignments::

        len(Repl(s, n)) == len(s) * n

    Parameters
    ----------
    value : Value, in
        Input value to be replicated.
    count : int
        Number of replications.

    Returns
    -------
    Repl, out
        Replicated value.
    """
    def __init__(self, value, count):
        if not isinstance(count, int) or count < 0:
            raise TypeError("Replication count must be a positive integer, not {!r}".format(count))

        super().__init__()
        self.value = Value.wrap(value)
        self.count = count

    def bits_sign(self):
        return len(self.value) * self.count, False

    def _rhs_signals(self):
        return value._rhs_signals()


class Signal(Value, DUID):
    """A varying integer value.

    Parameters
    ----------
    bits_sign : int or tuple or None
        Either an integer ``bits`` or a tuple ``(bits, signed)`` specifying the number of bits
        in this ``Signal`` and whether it is signed (can represent negative values).
        ``bits_sign`` defaults to 1-bit and non-signed.
    name : str
        Name hint for this signal. If ``None`` (default) the name is inferred from the variable
        name this ``Signal`` is assigned to. Name collisions are automatically resolved by
        prepending names of objects that contain this ``Signal`` and by appending integer
        sequences.
    reset : int
        Reset (synchronous) or default (combinatorial) value.
        When this ``Signal`` is assigned to in synchronous context and the corresponding clock
        domain is reset, the ``Signal`` assumes the given value. When this ``Signal`` is unassigned
        in combinatorial context (due to conditional assignments not being taken), the ``Signal``
        assumes its ``reset`` value. Defaults to 0.
    reset_less : bool
        If ``True``, do not generate reset logic for this ``Signal`` in synchronous statements.
        The ``reset`` value is only used as a combinatorial default or as the initial value.
        Defaults to ``False``.
    min : int or None
    max : int or None
        If `bits_sign` is `None`, the signal bit width and signedness are
        determined by the integer range given by `min` (inclusive,
        defaults to 0) and `max` (exclusive, defaults to 2).
    attrs : dict
        Dictionary of synthesis attributes.

    Attributes
    ----------
    nbits : int
    signed : bool
    name : str
    reset : int
    reset_less : bool
    attrs : dict
    """

    def __init__(self, bits_sign=None, name=None, reset=0, reset_less=False, min=None, max=None,
                 attrs=None):
        super().__init__()

        if name is None:
            try:
                name = tracer.get_var_name()
            except tracer.NameNotFound:
                name = "$signal"
        self.name = name

        if bits_sign is None:
            if min is None:
                min = 0
            if max is None:
                max = 2
            max -= 1  # make both bounds inclusive
            if not min < max:
                raise ValueError("Lower bound {!r} should be less than higher bound {!r}"
                                 .format(min, max))
            self.signed = min < 0 or max < 0
            self.nbits  = builtins.max(bits_for(min, self.signed), bits_for(max, self.signed))

        elif isinstance(bits_sign, int):
            if not (min is None or max is None):
                raise ValueError("Only one of bits/signedness or bounds may be specified")
            self.nbits, self.signed = bits_sign, False

        else:
            self.nbits, self.signed = bits_sign

        if not isinstance(self.nbits, int) or self.nbits < 0:
            raise TypeError("Width must be a positive integer, not {!r}".format(self.nbits))
        self.reset = reset
        self.reset_less = reset_less

        self.attrs = OrderedDict(() if attrs is None else attrs)

    @classmethod
    def like(cls, other, **kwargs):
        """Create Signal based on another.

        Parameters
        ----------
        other : Value
            Object to base this Signal on.
        """
        kw = dict(bits_sign=cls.wrap(other).bits_sign())
        if isinstance(other, cls):
            kw.update(reset=other.reset.value, reset_less=other.reset_less, attrs=other.attrs)
        kw.update(kwargs)
        return cls(**kw)

    def bits_sign(self):
        return self.nbits, self.signed

    def _lhs_signals(self):
        return ValueSet((self,))

    def _rhs_signals(self):
        return ValueSet((self,))

    def __repr__(self):
        return "(sig {})".format(self.name)


class ClockSignal(Value):
    """Clock signal for a given clock domain.

    ``ClockSignal`` s for a given clock domain can be retrieved multiple
    times. They all ultimately refer to the same signal.

    Parameters
    ----------
    cd : str
        Clock domain to obtain a clock signal for. Defaults to `"sys"`.
    """
    def __init__(self, cd="sys"):
        super().__init__()
        if not isinstance(cd, str):
            raise TypeError("Clock domain name must be a string, not {!r}".format(cd))
        self.cd = cd

    def __repr__(self):
        return "(clk {})".format(self.cd)


class ResetSignal(Value):
    """Reset signal for a given clock domain

    `ResetSignal` s for a given clock domain can be retrieved multiple
    times. They all ultimately refer to the same signal.

    Parameters
    ----------
    cd : str
        Clock domain to obtain a reset signal for. Defaults to `"sys"`.
    """
    def __init__(self, cd="sys"):
        super().__init__()
        if not isinstance(cd, str):
            raise TypeError("Clock domain name must be a string, not {!r}".format(cd))
        self.cd = cd

    def __repr__(self):
        return "(rst {})".format(self.cd)


class Statement:
    @staticmethod
    def wrap(obj):
        if isinstance(obj, Iterable):
            return sum((Statement.wrap(e) for e in obj), [])
        else:
            if isinstance(obj, Statement):
                return [obj]
            else:
                raise TypeError("Object {!r} is not a Migen statement".format(obj))


class Assign(Statement):
    def __init__(self, lhs, rhs):
        self.lhs = Value.wrap(lhs)
        self.rhs = Value.wrap(rhs)

    def _lhs_signals(self):
        return self.lhs._lhs_signals()

    def _rhs_signals(self):
        return self.rhs._rhs_signals()

    def __repr__(self):
        return "(eq {!r} {!r})".format(self.lhs, self.rhs)


class Switch(Statement):
    def __init__(self, test, cases):
        self.test  = Value.wrap(test)
        self.cases = OrderedDict()
        for key, stmts in cases.items():
            if isinstance(key, (bool, int)):
                key = "{:0{}b}".format(key, len(test))
            elif isinstance(key, str):
                assert len(key) == len(test)
            else:
                raise TypeError
            if not isinstance(stmts, Iterable):
                stmts = [stmts]
            self.cases[key] = Statement.wrap(stmts)

    def _lhs_signals(self):
        return union(s._lhs_signals() for ss in self.cases.values() for s in ss )

    def _rhs_signals(self):
        signals = union(s._rhs_signals() for ss in self.cases.values() for s in ss)
        return self.test._rhs_signals() | signals

    def __repr__(self):
        cases = ["(case {} {})".format(key, " ".join(map(repr, stmts)))
                 for key, stmts in self.cases.items()]
        return "(switch {!r} {})".format(self.test, " ".join(cases))


class ValueKey:
    def __init__(self, value):
        self.value = Value.wrap(value)

    def __hash__(self):
        if isinstance(self.value, Const):
            return hash(self.value)
        elif isinstance(self.value, Signal):
            return hash(id(self.value))
        elif isinstance(self.value, Slice):
            return hash((ValueKey(self.value.value), self.value.start, self.value.end))
        else:
            raise TypeError

    def __eq__(self, other):
        if not isinstance(other, ValueKey):
            return False
        if type(self.value) != type(other.value):
            return False

        if isinstance(self.value, Const):
            return self.value == other.value
        elif isinstance(self.value, Signal):
            return id(self.value) == id(other.value)
        elif isinstance(self.value, Slice):
            return (ValueKey(self.value.value) == ValueKey(other.value.value) and
                    self.value.start == other.value.start and
                    self.value.end == other.value.end)
        else:
            raise TypeError

    def __lt__(self, other):
        if not isinstance(other, ValueKey):
            return False
        if type(self.value) != type(other.value):
            return False

        if isinstance(self.value, Const):
            return self.value < other.value
        elif isinstance(self.value, Signal):
            return self.value.duid < other.value.duid
        elif isinstance(self.value, Slice):
            return (ValueKey(self.value.value) < ValueKey(other.value.value) and
                    self.value.start < other.value.start and
                    self.value.end < other.value.end)
        else:
            raise TypeError


class ValueDict(MutableMapping):
    def __init__(self, pairs=()):
        self._inner = dict()
        for key, value in pairs:
            self[key] = value

    def __getitem__(self, key):
        key = None if key is None else ValueKey(key)
        return self._inner[key]

    def __setitem__(self, key, value):
        key = None if key is None else ValueKey(key)
        self._inner[key] = value

    def __delitem__(self, key):
        key = None if key is None else ValueKey(key)
        del self._inner[key]

    def __iter__(self):
        return map(lambda x: None if x is None else x.value, sorted(self._inner))

    def __len__(self):
        return len(self._inner)


class ValueSet(MutableSet):
    def __init__(self, elements=()):
        self._inner = set()
        for elem in elements:
            self.add(elem)

    def add(self, value):
        self._inner.add(ValueKey(value))

    def update(self, values):
        for value in values:
            self.add(value)

    def discard(self, value):
        self._inner.discard(ValueKey(value))

    def __contains__(self, value):
        return ValueKey(value) in self._inner

    def __iter__(self):
        return map(lambda x: x.value, sorted(self._inner))

    def __len__(self):
        return len(self._inner)

    def __repr__(self):
        return "ValueSet({})".format(", ".join(repr(x) for x in self))
