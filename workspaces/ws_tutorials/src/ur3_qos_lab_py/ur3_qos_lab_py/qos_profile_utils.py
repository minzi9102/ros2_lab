from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def _normalize(text: str) -> str:
    return text.strip().lower().replace("-", "_")


def parse_reliability(value: str) -> ReliabilityPolicy:
    normalized = _normalize(value)
    if normalized == "reliable":
        return ReliabilityPolicy.RELIABLE
    if normalized in {"best_effort", "besteffort"}:
        return ReliabilityPolicy.BEST_EFFORT
    raise ValueError(
        f"Invalid reliability '{value}'. Expected: reliable | best_effort"
    )


def parse_history(value: str) -> HistoryPolicy:
    normalized = _normalize(value)
    if normalized == "keep_last":
        return HistoryPolicy.KEEP_LAST
    if normalized == "keep_all":
        return HistoryPolicy.KEEP_ALL
    raise ValueError(f"Invalid history '{value}'. Expected: keep_last | keep_all")


def make_qos_profile(reliability: str, history: str, depth: int) -> QoSProfile:
    history_policy = parse_history(history)
    qos_depth = max(1, int(depth))

    return QoSProfile(
        history=history_policy,
        depth=qos_depth,
        reliability=parse_reliability(reliability),
    )
