from pathlib import Path
import sys


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: check_python_syntax.py <file>")
        return 1
    path = Path(sys.argv[1])
    source = path.read_text(encoding="utf-8")
    compile(source, str(path), "exec")
    print("PYTHON_COMPILE_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
