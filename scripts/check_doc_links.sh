#!/usr/bin/env bash
# 문서 링크 검사 — docs/ 안의 markdown 상대 링크와 코드·설정이 적은 docs/ 경로가 실재하는지 본다.
# 출력이 없으면 전부 유효하다. 문서를 옮기거나 이름을 바꾼 뒤 반드시 돌린다.
cd "$(dirname "$0")/.."
# 1) docs 안의 markdown 상대 링크
while IFS= read -r f; do
  grep -oE '\]\(([^)]+\.md)(#[^)]*)?\)' "$f" | sed -E 's/^\]\(//; s/\)$//; s/#.*//' | sort -u |
  while read -r l; do
    case "$l" in http*|file:*) echo "EXTERNAL  $f -> $l"; continue;; esac
    p="$(printf '%b' "${l//%/\\x}")"
    [ -e "$(dirname "$f")/$p" ] || echo "BROKEN    $f -> $l"
  done
done < <(find docs -name '*.md')
# 2) 코드·설정·루트 문서의 docs/ 경로
grep -rnoE 'docs/[A-Za-z0-9_가-힣./%-]+\.md' \
  --include='*.py' --include='*.yaml' --include='*.yml' --include='*.sh' \
  --include='*.md' --include='*.json' --include='*.list' \
  src scripts config CLAUDE.md README.md 2>/dev/null |
while IFS=: read -r f n l; do
  p="$(printf '%b' "${l//%/\\x}")"
  [ -e "$p" ] || echo "BROKEN    $f:$n -> $l"
done
