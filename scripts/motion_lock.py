#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Lock or operator-authorize runtime motion for one Piper provider."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import stat
import sys

import grpc


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
CONTRACT_ID = "robonix/primitive/arm/motion_lock"


def _safe_provider_id(provider_id: str) -> str:
    return "".join(
        char if char.isalnum() or char in {"-", "_"} else "_"
        for char in provider_id
    )


def _read_operator_token(path: Path) -> str:
    info = path.stat()
    if info.st_uid != os.geteuid():
        raise RuntimeError(f"operator token is not owned by uid {os.geteuid()}: {path}")
    if stat.S_IMODE(info.st_mode) & 0o077:
        raise RuntimeError(f"operator token permissions must be 0600: {path}")
    token = path.read_text(encoding="utf-8").strip()
    if not token:
        raise RuntimeError(f"operator token is empty: {path}")
    return token


def _connect(provider_id: str):
    from robonix_api import ATLAS
    from robonix_api.atlas_types import Transport

    providers = ATLAS.query(contract_id=CONTRACT_ID, transport=Transport.GRPC)
    providers = [provider for provider in providers if provider.id == provider_id]
    if not providers:
        raise RuntimeError(
            f"active provider {provider_id!r} does not expose {CONTRACT_ID!r}"
        )
    channel_ref = ATLAS.connect_capability(
        consumer_id="piper-motion-lock-operator",
        provider_id=provider_id,
        contract_id=CONTRACT_ID,
        transport=Transport.GRPC,
    )
    endpoint = channel_ref.endpoint
    endpoint = endpoint.removeprefix("http://").removeprefix("https://")
    endpoint = endpoint.replace("localhost", "127.0.0.1")
    return channel_ref, endpoint


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("lock", "unlock"))
    parser.add_argument("--provider", required=True, help="deployment provider id")
    parser.add_argument(
        "--token-file",
        type=Path,
        help="override the generated per-activation token file",
    )
    args = parser.parse_args()

    sys.path.insert(0, str(PACKAGE_ROOT / "rbnx-build" / "codegen" / "proto_gen"))
    import piper_arm_pb2
    import robonix_contracts_pb2_grpc

    locked = args.action == "lock"
    token = ""
    if not locked:
        token_path = args.token_file or (
            PACKAGE_ROOT
            / "rbnx-build"
            / "data"
            / f"{_safe_provider_id(args.provider)}.motion-token"
        )
        token = _read_operator_token(token_path)

    channel_ref, endpoint = _connect(args.provider)
    try:
        with grpc.insecure_channel(endpoint) as channel:
            stub = robonix_contracts_pb2_grpc.RobonixPrimitiveArmMotionLockStub(
                channel
            )
            response = stub.SetMotionLock(
                piper_arm_pb2.SetMotionLock_Request(
                    locked=locked,
                    operator_token=token,
                )
            )
    finally:
        channel_ref.close()

    print(response.detail)
    return 0 if response.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
