from pathlib import Path
import sys


def main() -> int:
    if len(sys.argv) < 4:
        print("usage: inspect_lines.py <path> <start> <end>")
        return 1
    path = Path(sys.argv[1])
    start = int(sys.argv[2])
    end = int(sys.argv[3])
    lines = path.read_text(encoding="utf-8").splitlines()
    for idx in range(max(start, 1) - 1, min(end, len(lines))):
        print(f"{idx + 1}:{lines[idx]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
