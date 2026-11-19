"""Emit default-returning definitions for every out-of-line PROS device member the
sim does NOT implement in pros_sim.cpp (the vtables need all of them).
usage: python gen_stubs.py <include/pros dir> <out.cpp> <decls.tsv>
Re-run after adding a member to pros_sim.cpp, and add its name to EXCLUDE.
ponytail: regex/brace parser tuned to PROS 4 headers, not a C++ parser.
"""
import re, sys, os

INC, OUT, TSV = sys.argv[1:4]
# (header, class name, qualified prefix inside namespace pros{inline namespace v5{...}})
CLASSES = [
    ("device.hpp", "Device"), ("motors.hpp", "Motor"), ("motor_group.hpp", "MotorGroup"),
    ("imu.hpp", "Imu"), ("rotation.hpp", "Rotation"), ("distance.hpp", "Distance"),
]

EXCLUDE = {
    "Device": {"get_port", "is_installed"},
    "MotorGroup": {"move", "move_voltage", "move_velocity", "brake", "get_position", "get_position_all",
                   "get_actual_velocity", "get_actual_velocity_all", "get_voltage", "get_current_draw", "get_torque",
                   "get_temperature", "get_gearing_all", "get_gearing", "size", "get_port", "get_port_all",
                   "get_brake_mode", "get_brake_mode_all", "get_encoder_units", "get_encoder_units_all",
                   "set_brake_mode", "set_brake_mode_all", "set_encoder_units", "set_encoder_units_all",
                   "tare_position", "tare_position_all", "set_zero_position", "set_zero_position_all",
                   "is_reversed", "set_reversed", "set_reversed_all"},
    "Imu": {"reset", "set_data_rate", "get_rotation", "get_heading", "get_pitch", "get_roll", "get_yaw",
            "tare_rotation", "tare_heading", "tare_yaw", "tare", "set_rotation", "set_heading", "set_yaw",
            "get_status", "is_calibrating"},
    "Rotation": {"reset", "set_data_rate", "set_position", "reset_position", "get_position", "get_velocity",
                 "get_angle", "set_reversed", "reverse", "get_reversed"},
    "Distance": {"get", "get_distance", "get_confidence", "get_object_size", "get_object_velocity"},
}

def strip_comments(s):
    def blank(m): return re.sub(r"[^\n]", " ", m.group(0))
    s = re.sub(r"/\*.*?\*/", blank, s, flags=re.S)
    return re.sub(r"//[^\n]*", blank, s)

def class_body(src, name):
    m = re.search(r"\bclass\s+%s\b[^;{]*\{" % name, src)
    i = m.end(); depth = 1
    while depth:
        c = src[i]; depth += (c == "{") - (c == "}"); i += 1
    return m.end(), i - 1

def statements(src, a, b):
    """top-level statements in body [a,b): yields (start_offset, text, has_inline_body)"""
    i, start, depth, par = a, a, 0, 0
    while i < b:
        c = src[i]
        if c == "(": par += 1
        elif c == ")": par -= 1
        elif c == "{" and par == 0:
            j, d = i + 1, 1
            while d:
                d += (src[j] == "{") - (src[j] == "}"); j += 1
            yield start, src[start:j], True
            if j < b and src[j] == ";": j += 1
            i = start = j; continue
        elif c == ";" and par == 0:
            yield start, src[start:i], False
            start = i + 1
        elif c == ":" and par == 0 and re.fullmatch(r"\s*(public|private|protected)\s*", src[start:i]):
            start = i + 1
        i += 1

def strip_defaults(params):
    out, depth, cur, skipping = [], 0, "", False
    for c in params + ",":
        if c in "(<{": depth += 1
        if c in ")>}": depth -= 1
        if c == "," and depth == 0:
            out.append(cur.strip()); cur = ""; skipping = False; continue
        if c == "=" and depth == 0: skipping = True
        if not skipping: cur += c
    return ", ".join(p for p in out if p)

defs, rows = [], []
for hdr, cls in CLASSES:
    raw = open(os.path.join(INC, hdr), encoding="utf-8").read()
    src = strip_comments(raw)
    a, b = class_body(src, cls)
    for off, text, inline in statements(src, a, b):
        t = " ".join(text.split())
        if "(" not in t or inline or t.startswith(("template", "friend", "using", "typedef")) or re.search(r"=\s*(default|delete|0)\s*$", t):
            continue
        line = src.count("\n", 0, off + len(text) - len(text.lstrip())) + 1
        is_virtual = bool(re.search(r"\bvirtual\b", t))
        t2 = re.sub(r"\[\[[^\]]*\]\]|\b(virtual|explicit|static|inline)\b", "", t).strip()
        m = re.match(r"(.*?)\b(~?\w+|operator\s*[^\s(]+)\s*\((.*)\)\s*(const)?\s*(override)?\s*$", t2)
        if not m:
            print("SKIP", hdr, line, t, file=sys.stderr); continue
        ret, name, params, const = m.group(1).strip(), m.group(2), m.group(3), m.group(4) or ""
        rows.append(f"{cls}\t{name}\t{hdr}:{line}\t{'virtual' if is_virtual or m.group(5) else ''}\t{t}")
        if name in (cls, "~" + cls) or name in EXCLUDE.get(cls, ()):
            continue  # ctors/dtor are hand written
        body = "{}" if ret == "void" else (f"{{ return {cls}(std::uint8_t{{0}}); }}" if ret == cls else "{ return {}; }")
        defs.append(f"{ret} {cls}::{name}({strip_defaults(params)}) {const} {body}")

with open(OUT, "w") as f:
    f.write('#include "api.h"\nnamespace pros { inline namespace v5 {\n')
    f.write("\n".join(defs))
    f.write("\n}}\n")
with open(TSV, "w") as f:
    f.write("\n".join(rows) + "\n")
print(len(defs), "stub defs,", len(rows), "decls")
