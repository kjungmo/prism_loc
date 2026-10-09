#!/usr/bin/env python3
"""Check that a running prism_loc node actually applied every value in a params YAML.

ROS 2 silently ignores YAML keys that a node never declares, so a typo, a key the
backend does not declare, or a key declared only under some condition would leave
the default in place without any warning. This script reads the YAML, waits for the
node's parameter services, and fails if any YAML key is not declared by the node or
holds a different value. Keys the launch file overrides on purpose (for example
map_pcd_path) are named with --allow-override and only need to be declared.

With --diagnostics NAME it also waits for a /diagnostics status called NAME.

    python3 scripts/check_param_binding.py prism_loc/params/laser2d.yaml --node prism_loc
    python3 scripts/check_param_binding.py prism_loc/params/ndt3d.yaml --node prism_loc \\
        --allow-override map_pcd_path --diagnostics "prism_loc: localization"
"""

import argparse
import math
import sys
import time

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, ListParameters


def yaml_params(path):
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    params = {}
    for section in doc.values():  # "/**" or a node name
        params.update(section.get("ros__parameters", {}))
    return params


def value_of(pv):
    return {
        ParameterType.PARAMETER_BOOL: lambda: pv.bool_value,
        ParameterType.PARAMETER_INTEGER: lambda: pv.integer_value,
        ParameterType.PARAMETER_DOUBLE: lambda: pv.double_value,
        ParameterType.PARAMETER_STRING: lambda: pv.string_value,
    }.get(pv.type, lambda: None)()


def same(expected, actual):
    if isinstance(expected, bool) or isinstance(actual, bool):
        return expected is actual
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        return math.isclose(float(expected), float(actual), rel_tol=0.0, abs_tol=1e-12)
    return expected == actual


def call(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=timeout):
        sys.exit(f"FAIL: service {client.srv_name} not available after {timeout:.0f} s")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if future.result() is None:
        sys.exit(f"FAIL: no response from {client.srv_name}")
    return future.result()


def wait_for_status(node, name, timeout):
    seen = []
    node.create_subscription(DiagnosticArray, "/diagnostics",
                             lambda m: seen.extend(s for s in m.status if s.name == name), 10)
    end = time.monotonic() + timeout
    while not seen and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    return seen[-1] if seen else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml")
    ap.add_argument("--node", default="prism_loc")
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--allow-override", action="append", default=[], metavar="KEY",
                    help="YAML key the launch file sets on purpose; must be declared, value not compared")
    ap.add_argument("--diagnostics", metavar="NAME", help="also require a /diagnostics status NAME")
    args = ap.parse_args()

    expected = yaml_params(args.yaml)
    unknown = [k for k in args.allow_override if k not in expected]
    if unknown:
        sys.exit(f"FAIL: --allow-override keys not in {args.yaml}: {unknown}")
    rclpy.init()
    node = rclpy.create_node("prism_loc_param_binding_check")
    target = "/" + args.node.lstrip("/")
    listed = call(node, node.create_client(ListParameters, f"{target}/list_parameters"),
                  ListParameters.Request(), args.timeout)
    declared = set(listed.result.names)

    undeclared = sorted(k for k in expected if k not in declared)
    keys = sorted(k for k in expected if k in declared)
    req = GetParameters.Request()
    req.names = keys
    got = call(node, node.create_client(GetParameters, f"{target}/get_parameters"), req, args.timeout)
    actual = {k: value_of(v) for k, v in zip(keys, got.values)}
    mismatched = [(k, expected[k], actual[k]) for k in keys
                  if k not in args.allow_override and not same(expected[k], actual[k])]

    print(f"{args.yaml}: {len(expected)} YAML keys, {len(keys) - len(mismatched)} applied "
          f"({len([k for k in args.allow_override if k in declared])} overridden by launch)")
    for k in undeclared:
        print(f"  NOT DECLARED by {target}: {k} = {expected[k]!r} (the node ignores it)")
    for k, e, a in mismatched:
        print(f"  MISMATCH {k}: YAML {e!r}, node {a!r}")
    failed = bool(undeclared or mismatched)
    if args.diagnostics:
        st = wait_for_status(node, args.diagnostics, args.timeout)
        if st is None:
            print(f"  NO /diagnostics status named {args.diagnostics!r} within {args.timeout:.0f} s")
            failed = True
        else:
            lv = st.level[0] if isinstance(st.level, bytes) else st.level
            print(f"  /diagnostics {args.diagnostics!r}: level {lv}, {st.message!r}")
    node.destroy_node()
    rclpy.shutdown()
    if failed:
        sys.exit("FAIL: parameter binding")
    print("PASS: every YAML key is declared by the node and holds the YAML value")


if __name__ == "__main__":
    main()
