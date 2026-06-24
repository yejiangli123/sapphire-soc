#!/bin/bash
for p in /opt/synopsys/setup.sh /eda/synopsys/setup.sh /tools/synopsys/setup.sh; do
  [ -f "$p" ] && echo "Sourcing: $p" && source "$p" && break
done
echo "=== Tool Check ==="
which vcs   >/dev/null 2>&1 && echo "  VCS:   $(which vcs)"   || echo "  VCS not found"
which verdi >/dev/null 2>&1 && echo "  Verdi: $(which verdi)" || echo "  Verdi not found"
