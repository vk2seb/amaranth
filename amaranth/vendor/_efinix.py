from abc import abstractmethod

from ..hdl import *
from ..hdl._ir import RequirePosedge
from ..lib import io, wiring
from ..build import *


class InnerBuffer(wiring.Component):
    """A private component used to implement ``lib.io`` buffers.

    Works like ``lib.io.Buffer``, with the following differences:

    - ``port.invert`` is ignored (handling the inversion is the outer buffer's responsibility)
    - ``t`` is per-pin inverted output enable
    """
    def __init__(self, direction, port):
        self.direction = direction
        self.port = port
        members = {}
        if direction is not io.Direction.Output:
            members["i"] = wiring.In(len(port))
        if direction is not io.Direction.Input:
            members["o"] = wiring.Out(len(port))
            members["t"] = wiring.Out(len(port))
        super().__init__(wiring.Signature(members).flip())

    def elaborate(self, platform):
        m = Module()

        if isinstance(self.port, io.SingleEndedPort):
            io_port = self.port.io
        elif isinstance(self.port, io.DifferentialPort):
            io_port = self.port.p
        else:
            raise TypeError(f"Unknown port type {self.port!r}")

        for bit in range(len(self.port)):
            name = f"buf{bit}"
            if self.direction is io.Direction.Input:
                m.submodules[name] = Instance("IB",
                    i_I=io_port[bit],
                    o_O=self.i[bit],
                )
            elif self.direction is io.Direction.Output:
                m.submodules[name] = Instance("OBZ",
                    i_T=self.t[bit],
                    i_I=self.o[bit],
                    o_O=io_port[bit],
                )
            elif self.direction is io.Direction.Bidir:
                m.submodules[name] = Instance("BB",
                    i_T=self.t[bit],
                    i_I=self.o[bit],
                    o_O=self.i[bit],
                    io_B=io_port[bit],
                )
            else:
                assert False # :nocov:

        return m


class IOBuffer(io.Buffer):
    def elaborate(self, platform):
        m = Module()

        m.submodules.buf = buf = InnerBuffer(self.direction, self.port)
        inv_mask = sum(inv << bit for bit, inv in enumerate(self.port.invert))

        if self.direction is not io.Direction.Output:
            m.d.comb += self.i.eq(buf.i ^ inv_mask)

        if self.direction is not io.Direction.Input:
            m.d.comb += buf.o.eq(self.o ^ inv_mask)
            m.d.comb += buf.t.eq(~self.oe.replicate(len(self.port)))

        return m


def _make_oereg_trion(m, domain, oe, q):
    for bit in range(len(q)):
        m.submodules[f"oe_ff{bit}"] = Instance("OFS1P3DX",
            i_SCLK=ClockSignal(domain),
            i_SP=Const(1),
            i_CD=Const(0),
            i_D=oe,
            o_Q=q[bit],
        )


def _make_oereg_titanium(m, domain, oe, q):
    for bit in range(len(q)):
        m.submodules[f"oe_ff{bit}"] = Instance("OFD1P3DX",
            i_CK=ClockSignal(domain),
            i_SP=Const(1),
            i_CD=Const(0),
            i_D=oe,
            o_Q=q[bit],
        )


class FFBufferTrion(io.FFBuffer):
    def elaborate(self, platform):
        m = Module()

        m.submodules.buf = buf = InnerBuffer(self.direction, self.port)
        inv_mask = sum(inv << bit for bit, inv in enumerate(self.port.invert))

        if self.direction is not io.Direction.Output:
            m.submodules += RequirePosedge(self.i_domain)
            i_inv = Signal.like(self.i)
            for bit in range(len(self.port)):
                m.submodules[f"i_ff{bit}"] = Instance("IFS1P3DX",
                    i_SCLK=ClockSignal(self.i_domain),
                    i_SP=Const(1),
                    i_CD=Const(0),
                    i_D=buf.i[bit],
                    o_Q=i_inv[bit],
                )
            m.d.comb += self.i.eq(i_inv ^ inv_mask)

        if self.direction is not io.Direction.Input:
            m.submodules += RequirePosedge(self.o_domain)
            o_inv = Signal.like(self.o)
            m.d.comb += o_inv.eq(self.o ^ inv_mask)
            for bit in range(len(self.port)):
                m.submodules[f"o_ff{bit}"] = Instance("OFS1P3DX",
                    i_SCLK=ClockSignal(self.o_domain),
                    i_SP=Const(1),
                    i_CD=Const(0),
                    i_D=o_inv[bit],
                    o_Q=buf.o[bit],
                )
            _make_oereg_trion(m, self.o_domain, ~self.oe, buf.t)

        return m


class FFBufferTitanium(io.FFBuffer):
    def elaborate(self, platform):
        m = Module()

        m.submodules.buf = buf = InnerBuffer(self.direction, self.port)
        inv_mask = sum(inv << bit for bit, inv in enumerate(self.port.invert))

        if self.direction is not io.Direction.Output:
            m.submodules += RequirePosedge(self.i_domain)
            i_inv = Signal.like(self.i)
            for bit in range(len(self.port)):
                m.submodules[f"i_ff{bit}"] = Instance("IFD1P3DX",
                    i_CK=ClockSignal(self.i_domain),
                    i_SP=Const(1),
                    i_CD=Const(0),
                    i_D=buf.i[bit],
                    o_Q=i_inv[bit],
                )
            m.d.comb += self.i.eq(i_inv ^ inv_mask)

        if self.direction is not io.Direction.Input:
            m.submodules += RequirePosedge(self.o_domain)
            o_inv = Signal.like(self.o)
            m.d.comb += o_inv.eq(self.o ^ inv_mask)
            for bit in range(len(self.port)):
                m.submodules[f"o_ff{bit}"] = Instance("OFD1P3DX",
                    i_CK=ClockSignal(self.o_domain),
                    i_SP=Const(1),
                    i_CD=Const(0),
                    i_D=o_inv[bit],
                    o_Q=buf.o[bit],
                )
            _make_oereg_titanium(m, self.o_domain, ~self.oe, buf.t)

        return m


class DDRBufferTrion(io.DDRBuffer):
    def elaborate(self, platform):
        m = Module()

        m.submodules.buf = buf = InnerBuffer(self.direction, self.port)
        inv_mask = sum(inv << bit for bit, inv in enumerate(self.port.invert))

        if self.direction is not io.Direction.Output:
            m.submodules += RequirePosedge(self.i_domain)
            i0_inv = Signal(len(self.port))
            i1_inv = Signal(len(self.port))
            for bit in range(len(self.port)):
                m.submodules[f"i_ddr{bit}"] = Instance("IDDRX1F",
                    i_SCLK=ClockSignal(self.i_domain),
                    i_RST=Const(0),
                    i_D=buf.i[bit],
                    o_Q0=i0_inv[bit],
                    o_Q1=i1_inv[bit],
                )
            m.d.comb += self.i[0].eq(i0_inv ^ inv_mask)
            m.d.comb += self.i[1].eq(i1_inv ^ inv_mask)

        if self.direction is not io.Direction.Input:
            m.submodules += RequirePosedge(self.o_domain)
            o0_inv = Signal(len(self.port))
            o1_inv = Signal(len(self.port))
            m.d.comb += [
                o0_inv.eq(self.o[0] ^ inv_mask),
                o1_inv.eq(self.o[1] ^ inv_mask),
            ]
            for bit in range(len(self.port)):
                m.submodules[f"o_ddr{bit}"] = Instance("ODDRX1F",
                    i_SCLK=ClockSignal(self.o_domain),
                    i_RST=Const(0),
                    i_D0=o0_inv[bit],
                    i_D1=o1_inv[bit],
                    o_Q=buf.o[bit],
                )
            _make_oereg_trion(m, self.o_domain, ~self.oe, buf.t)

        return m


class DDRBufferTitanium(io.DDRBuffer):
    def elaborate(self, platform):
        m = Module()

        m.submodules.buf = buf = InnerBuffer(self.direction, self.port)
        inv_mask = sum(inv << bit for bit, inv in enumerate(self.port.invert))

        if self.direction is not io.Direction.Output:
            m.submodules += RequirePosedge(self.i_domain)
            i0_inv = Signal(len(self.port))
            i1_inv = Signal(len(self.port))
            for bit in range(len(self.port)):
                m.submodules[f"i_ddr{bit}"] = Instance("IDDRX1",
                    i_SCLK=ClockSignal(self.i_domain),
                    i_RST=Const(0),
                    i_D=buf.i[bit],
                    o_Q0=i0_inv[bit],
                    o_Q1=i1_inv[bit],
                )
            m.d.comb += self.i[0].eq(i0_inv ^ inv_mask)
            m.d.comb += self.i[1].eq(i1_inv ^ inv_mask)

        if self.direction is not io.Direction.Input:
            m.submodules += RequirePosedge(self.o_domain)
            o0_inv = Signal(len(self.port))
            o1_inv = Signal(len(self.port))
            m.d.comb += [
                o0_inv.eq(self.o[0] ^ inv_mask),
                o1_inv.eq(self.o[1] ^ inv_mask),
            ]
            for bit in range(len(self.port)):
                m.submodules[f"o_ddr{bit}"] = Instance("ODDRX1",
                    i_SCLK=ClockSignal(self.o_domain),
                    i_RST=Const(0),
                    i_D0=o0_inv[bit],
                    i_D1=o1_inv[bit],
                    o_Q=buf.o[bit],
                )
            _make_oereg_titanium(m, self.o_domain, ~self.oe, buf.t)

        return m


class EfinixPlatform(TemplatedPlatform):
    """
    .. rubric:: Efinity toolchain

    Required tools:
        * ``efinity`` (Efinix Efinity IDE)

    The environment is populated by running the script specified in the environment variable
    ``AMARANTH_ENV_EFINITY``, if present. Set this to the Efinity installation path.

    Device naming:
        The ``device`` property must include the timing model suffix (e.g., "T20F256C4" where
        "C4" is the timing model). The platform will automatically split this into device and
        timing model components.

    Available overrides:
        * ``synth_mode``: synthesis optimization mode (``speed``, ``area``, ``area2``).
        * ``infer_clk_enable``: infers the flip-flop clock enable signal (0-4).
        * ``infer_sync_set_reset``: infer synchronous set/reset signals (0-1).
        * ``bram_output_regs_packing``: pack registers into the output of BRAM (0-1).
        * ``retiming``: perform retiming optimization (0-2).
        * ``seq_opt``: turn on sequential optimization (0-1).
        * ``mult_input_regs_packing``: allow packing of multiplier input registers (0-1, Trion only).
        * ``mult_output_regs_packing``: allow packing of multiplier output registers (0-1, Trion only).
        * ``spi_mode``: SPI configuration mode (``active``, ``passive``).
        * ``spi_width``: SPI bus width (``1``, ``2``, ``4``).
        * ``oscillator_clock_divider``: configuration oscillator divider (``DIV8``, etc.).
        * ``script_project``: inserts commands at the end of the Python configuration script.
        * ``add_constraints``: inserts commands at the end of the SDC file.

    Build products:
        * ``{{name}}.xml``: project file.
        * ``{{name}}.peri.xml``: peripheral configuration (generated by iface.py).
        * ``{{name}}.sdc``: user timing constraints.
        * ``{{name}}_merged.sdc``: merged timing constraints (PT + user).
        * ``{{name}}.v``: Verilog RTL.
        * ``{{name}}_iface.py``: interface configuration script.
        * ``outflow/{{name}}.bit``: binary bitstream.
        * ``outflow/{{name}}.hex``: HEX bitstream (for flash).
        * ``outflow/{{name}}.log``: build log.

    Supported device families:
        * **Trion** (T-series): Entry-level FPGAs
        * **Topaz** (Tz-series): Low-power FPGAs
        * **Titanium** (Ti-series): High-performance FPGAs
    """

    toolchain = "Efinity"

    device  = property(abstractmethod(lambda: None))
    package = property(abstractmethod(lambda: None))
    speed   = property(abstractmethod(lambda: None))  # Note: Include timing model in device name

    # Efinity templates

    _efinity_required_tools = [
        "python3"
    ]
    _efinity_file_templates = {
        **TemplatedPlatform.build_script_templates,
        "build_{{name}}.sh": r"""
            #!/bin/sh
            # {{autogenerated}}
            set -e{{verbose("x")}}
            if [ -z "$BASH" ] ; then exec /bin/bash "$0" "$@"; fi
            if [ -n "${{platform._toolchain_env_var}}" ]; then
                . "${{platform._toolchain_env_var}}"
            fi
            {{emit_commands("sh")}}
        """,
        "{{name}}.v": r"""
            /* {{autogenerated}} */
            {{emit_verilog()}}
        """,
        "{{name}}.debug.v": r"""
            /* {{autogenerated}} */
            {{emit_debug_verilog()}}
        """,
        "{{name}}.sdc": r"""
            # {{autogenerated}}
            {% for signal, frequency in platform.iter_signal_clock_constraints() -%}
                create_clock -name {{signal.name}} -period {{1000000000/frequency}} [get_nets {{'{'}}{{signal|hierarchy(".")}}{{'}'}} ]
            {% endfor %}
            {% for port, frequency in platform.iter_port_clock_constraints() -%}
                create_clock -name {{port.name}} -period {{1000000000/frequency}} [get_ports {{'{'}}{{port.name}}{{'}'}}]
            {% endfor %}
            {{get_override("add_constraints")|default("# (add_constraints placeholder)")}}
        """,
        "{{name}}.xml": r"""
            <?xml version="1.0" encoding="UTF-8"?>
            <efx:project xmlns:efx="http://www.efinixinc.com/enf_proj" name="{{name}}" location="." sw_version="{{platform._efinity_version}}" last_change_date="">
                <efx:device_info>
                    <efx:family name="{{platform.family}}"/>
                    <efx:device name="{{platform.device}}"/>
                    <efx:timing_model name="{{platform.timing_model}}"/>
                </efx:device_info>
                <efx:design_info def_veri_version="verilog_2k" def_vhdl_version="vhdl_2008">
                    <efx:top_module name="{{name}}"/>
                    {% for file in platform.iter_files(".v", ".sv") -%}
                        <efx:design_file name="{{file}}" version="default" library="default"/>
                    {% endfor %}
                    <efx:design_file name="{{name}}.v" version="default" library="default"/>
                </efx:design_info>
                <efx:constraint_info>
                    <efx:sdc_file name="{{name}}_merged.sdc"/>
                </efx:constraint_info>
                <efx:sim_info/>
                <efx:misc_info/>
                <efx:ip_info/>
                <efx:synthesis tool_name="efx_map">
                    <efx:param name="mode" value="{{get_override("synth_mode")|default("speed")}}" value_type="e_option"/>
                    <efx:param name="infer-clk-enable" value="{{get_override("infer_clk_enable")|default("3")}}" value_type="e_option"/>
                    <efx:param name="infer-sync-set-reset" value="{{get_override("infer_sync_set_reset")|default("1")}}" value_type="e_option"/>
                    <efx:param name="bram_output_regs_packing" value="{{get_override("bram_output_regs_packing")|default("1")}}" value_type="e_option"/>
                    <efx:param name="retiming" value="{{get_override("retiming")|default("1")}}" value_type="e_option"/>
                    <efx:param name="seq_opt" value="{{get_override("seq_opt")|default("1")}}" value_type="e_option"/>
                    {% if platform.family == "Trion" -%}
                        <efx:param name="mult_input_regs_packing" value="{{get_override("mult_input_regs_packing")|default("1")}}" value_type="e_option"/>
                        <efx:param name="mult_output_regs_packing" value="{{get_override("mult_output_regs_packing")|default("1")}}" value_type="e_option"/>
                    {% endif %}
                </efx:synthesis>
                <efx:place_and_route tool_name="efx_pnr"/>
                <efx:bitstream_generation tool_name="efx_pgm">
                    <efx:param name="mode" value="{{get_override("spi_mode")|default("active")}}" value_type="e_option"/>
                    <efx:param name="width" value="{{get_override("spi_width")|default("1")}}" value_type="e_option"/>
                    <efx:param name="oscillator_clock_divider" value="{{get_override("oscillator_clock_divider")|default("DIV8")}}" value_type="e_option"/>
                    <efx:param name="spi_low_power_mode" value="off" value_type="e_bool"/>
                    <efx:param name="io_weak_pullup" value="on" value_type="e_bool"/>
                    <efx:param name="enable_roms" value="on" value_type="e_bool"/>
                    <efx:param name="enable_crc_check" value="on" value_type="e_bool"/>
                </efx:bitstream_generation>
                <efx:debugger/>
                <efx:security/>
            </efx:project>
        """,
        "{{name}}_iface.py": r"""
            # {{autogenerated}}
            import os
            import sys

            # Add Efinity Python API to path
            efinity_path = os.environ.get("AMARANTH_ENV_EFINITY", "").rstrip('/')
            if not efinity_path:
                print("Error: AMARANTH_ENV_EFINITY environment variable not set", file=sys.stderr)
                sys.exit(1)

            sys.path.insert(0, os.path.join(efinity_path, "scripts"))

            from PythonInterface import *

            design = PinEdit("{{name}}")

            # Configure iobank info if provided
            {% if platform.iobank_info -%}
                {% for iobank, info in platform.iobank_info.items() -%}
                    {% for key, value in info.items() -%}
                        design.set_iobank("{{iobank}}", "{{key}}", "{{value}}")
                    {% endfor %}
                {% endfor %}
            {% endif %}

            # Create GPIO instances and configure pins
            {% for port_name, pin_name, attrs in platform.iter_port_constraints_bits() -%}
                # Pin: {{port_name}} -> {{pin_name}}
                design.assign_pkg_pin("{{port_name}}", "{{pin_name}}")
                {% for attr_name, attr_value in attrs.items() -%}
                    {% if attr_name == "IOSTANDARD" -%}
                        design.set_property("{{port_name}}", "IO_STANDARD", "{{attr_value}}")
                    {% elif attr_name == "PULLMODE" -%}
                        {% if attr_value in ["UP", "DOWN"] -%}
                            design.set_property("{{port_name}}", "PULL_OPTION", "WEAK_PULL{{attr_value}}")
                        {% endif %}
                    {% elif attr_name == "DRIVE" -%}
                        design.set_property("{{port_name}}", "DRIVE_STRENGTH", "{{attr_value}}")
                    {% elif attr_name == "SLEWRATE" -%}
                        {% if attr_value == "FAST" -%}
                            design.set_property("{{port_name}}", "SLEW_RATE", "1")
                        {% else -%}
                            design.set_property("{{port_name}}", "SLEW_RATE", "0")
                        {% endif %}
                    {% elif attr_name == "SCHMITT_TRIGGER" -%}
                        design.set_property("{{port_name}}", "SCHMITT_TRIGGER", "{{attr_value}}")
                    {% else -%}
                        design.set_property("{{port_name}}", "{{attr_name}}", "{{attr_value}}")
                    {% endif %}
                {% endfor %}
            {% endfor %}

            {{get_override("script_project")|default("# (script_project placeholder)")}}

            design.save()
        """,
    }
    _efinity_command_templates = [
        # Run interface configuration script
        r"""
        {{invoke_tool("python3")}}
            $AMARANTH_ENV_EFINITY/bin/python3
            {{name}}_iface.py
        """,
        # Merge SDC files (PT-generated + user constraints)
        r"""
        {{invoke_tool("sh")}}
            -c
            'if [ -f outflow/{{name}}.pt.sdc ]; then
                cat outflow/{{name}}.pt.sdc > {{name}}_merged.sdc
                echo "" >> {{name}}_merged.sdc
                echo "#########################" >> {{name}}_merged.sdc
                echo "# User Constraints" >> {{name}}_merged.sdc
                echo "#########################" >> {{name}}_merged.sdc
                echo "" >> {{name}}_merged.sdc
                cat {{name}}.sdc >> {{name}}_merged.sdc
            else
                cp {{name}}.sdc {{name}}_merged.sdc
            fi'
        """,
        # Run Efinity toolchain
        r"""
        {{invoke_tool("python3")}}
            $AMARANTH_ENV_EFINITY/bin/python3
            $AMARANTH_ENV_EFINITY/scripts/efx_run.py
            {{name}}.xml
            --flow compile
        """,
    ]

    # Common logic

    def __init__(self, *, toolchain="Efinity", iobank_info=None):
        super().__init__()

        # Extract timing model from device name (e.g., "T20F256C4" -> device="T20F256", speed="C4")
        # This matches LiteX's approach
        device = self.device
        if len(device) >= 2:
            self.timing_model = device[-2:]
            device_base = device[:-2]
        else:
            raise ValueError(f"Device '{self.device}' has invalid format")

        # Determine device family
        if device_base.startswith("Ti"):
            self.family = "Titanium"
        elif device_base.startswith("Tz"):
            self.family = "Topaz"
        elif device_base.startswith("T"):
            self.family = "Trion"
        else:
            raise ValueError(f"Device '{device_base}' is not recognized")

        # Store both original and parsed device names
        self._device_full = self.device
        self.device = device_base

        # Store iobank configuration
        self.iobank_info = iobank_info

        # Get Efinity version if available
        import os
        self._efinity_path = None
        self._efinity_version = "unknown"

        efinity_path = os.environ.get("AMARANTH_ENV_EFINITY", "")
        if efinity_path:
            self._efinity_path = efinity_path.rstrip('/')
            version_file = os.path.join(self._efinity_path, "scripts/sw_version.txt")
            try:
                with open(version_file, "r") as f:
                    self._efinity_version = f.readline().strip()
            except (FileNotFoundError, IOError):
                self._efinity_version = "unknown"

        assert toolchain == "Efinity"
        self.toolchain = toolchain

    @property
    def required_tools(self):
        # Check toolchain availability if building
        if self._efinity_path is None:
            raise OSError(
                "Unable to find Efinity toolchain. Please set AMARANTH_ENV_EFINITY "
                "environment variable to the Efinity installation path."
            )
        return self._efinity_required_tools

    @property
    def file_templates(self):
        return self._efinity_file_templates

    @property
    def command_templates(self):
        return self._efinity_command_templates

    @property
    def default_clk_constraint(self):
        # Use the defined Clock resource
        return super().default_clk_constraint

    def create_missing_domain(self, name):
        # Efinix devices have a power-on reset that is released after configuration.
        # For the sync domain, we create a simple reset synchronizer if a default
        # clock and reset are defined.
        if name == "sync" and self.default_clk is not None:
            m = Module()

            clk_io = self.request(self.default_clk, dir="-")
            m.submodules.clk_buf = clk_buf = io.Buffer("i", clk_io)
            clk_i = clk_buf.i

            if self.default_rst is not None:
                rst_io = self.request(self.default_rst, dir="-")
                m.submodules.rst_buf = rst_buf = io.Buffer("i", rst_io)
                rst_i = rst_buf.i
            else:
                rst_i = Const(0)

            # Simple reset synchronizer
            sync0 = Signal()
            sync1 = Signal()
            m.submodules += [
                Instance("FD1P3AX",
                    i_CK=clk_i,
                    i_D=~rst_i,
                    o_Q=sync0,
                ),
                Instance("FD1P3AX",
                    i_CK=clk_i,
                    i_D=sync0,
                    o_Q=sync1,
                ),
            ]

            m.domains += ClockDomain("sync", reset_less=self.default_rst is None)
            m.d.comb += ClockSignal("sync").eq(clk_i)
            if self.default_rst is not None:
                m.d.comb += ResetSignal("sync").eq(~sync1)

            return m

    def get_io_buffer(self, buffer):
        if isinstance(buffer, io.Buffer):
            result = IOBuffer(buffer.direction, buffer.port)
        elif isinstance(buffer, io.FFBuffer):
            if self.family in ("Trion", "Topaz"):
                result = FFBufferTrion(buffer.direction, buffer.port,
                                      i_domain=buffer.i_domain,
                                      o_domain=buffer.o_domain)
            elif self.family == "Titanium":
                result = FFBufferTitanium(buffer.direction, buffer.port,
                                         i_domain=buffer.i_domain,
                                         o_domain=buffer.o_domain)
            else:
                raise NotImplementedError # :nocov:
        elif isinstance(buffer, io.DDRBuffer):
            if self.family in ("Trion", "Topaz"):
                result = DDRBufferTrion(buffer.direction, buffer.port,
                                       i_domain=buffer.i_domain,
                                       o_domain=buffer.o_domain)
            elif self.family == "Titanium":
                result = DDRBufferTitanium(buffer.direction, buffer.port,
                                          i_domain=buffer.i_domain,
                                          o_domain=buffer.o_domain)
            else:
                raise NotImplementedError # :nocov:
        else:
            raise TypeError(f"Unsupported buffer type {buffer!r}") # :nocov:
        if buffer.direction is not io.Direction.Output:
            result.i = buffer.i
        if buffer.direction is not io.Direction.Input:
            result.o = buffer.o
            result.oe = buffer.oe
        return result

    # CDC primitives are not currently specialized for Efinix.
