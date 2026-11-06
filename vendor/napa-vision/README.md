# napa-vision's C core

`napa_core.c`, `napa_core.h`, `families.h` and `families/` from
[napa-vision](https://github.com/zwatx/NAPA) (`src/napa_vision/_native/`), unmodified, at commit
`de613adcd5a047e0f9345b7cfebcfd879cac8d98`. Copyright (c) 2026 Zachary Wilson, GPL-3.0-only; see `LICENSE` and `NOTICE`.

`build.rs` compiles it into sonoma-vision for `detector::Backend::Napa`. To update, copy the same
files from a newer checkout and update the commit above.
