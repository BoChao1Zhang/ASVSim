from __future__ import annotations

import json
import os
from datetime import datetime, timezone


def diagnostics_enabled() -> bool:
    return os.getenv("RL_DIAGNOSTICS", "").strip().lower() in {"1", "true", "yes", "on"}


def _normalize(value):
    if isinstance(value, dict):
        return {str(key): _normalize(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_normalize(item) for item in value]
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            return repr(value)
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return repr(value)


def diag_log(event: str, **fields) -> None:
    if not diagnostics_enabled():
        return

    payload = {
        "ts": datetime.now(timezone.utc).isoformat(),
        "event": event,
        **{key: _normalize(value) for key, value in fields.items()},
    }
    print(f"[RL_DIAG] {json.dumps(payload, sort_keys=True, ensure_ascii=True)}")
