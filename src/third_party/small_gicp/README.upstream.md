# small_gicp (vendored)

| | |
|---|---|
| Upstream | https://github.com/koide3/small_gicp |
| Pinned commit | `8a2d3734f699c042db74ae61d295b0b928163526` |
| Commit date | 2026-07-22 |
| License | MIT — see [LICENSE](LICENSE) |
| Vendored on | 2026-08-20 |

## 為什麼要 vendor

原本 `src/CMakeLists.txt` 用 `FetchContent` + `GIT_TAG master` 在 configure
階段從 GitHub 抓原始碼。這代表 (a) 建置需要網路，(b) `master` 沒有釘住版本，
上游一改 API 就會在某次 `cmake` 時突然壞掉。改為 in-tree 之後兩個問題都消失。

## 這裡包含什麼

只有建置所需的最小集合：

- `include/small_gicp/**` — 完整 header tree（含 TBB / PCL 變體的 header；
  它們互相 include，砍掉會破壞相依關係，而且總共只有 328 KB）
- `src/small_gicp/registration/registration.cpp`
- `src/small_gicp/registration/registration_helper.cpp`
- `LICENSE`
- `CMakeLists.txt` — 本專案自行撰寫，非上游版本

已刻意排除：`data/`（2.2 MB 測試點雲）、`docs/`、`.github/`、
`src/python`、`src/test`、`src/example`、`src/benchmark`。

## 如何更新版本

1. `git clone https://github.com/koide3/small_gicp.git /tmp/small_gicp`
2. `git -C /tmp/small_gicp checkout <新的 SHA>`
3. 覆蓋 `include/` 與 `src/small_gicp/registration/*.cpp`
4. 比對上游 `CMakeLists.txt` 的 helper library 段落，確認來源檔清單與
   `target_link_libraries` 沒有變動；有變動就同步到本目錄的 `CMakeLists.txt`
5. 更新本檔案上方表格的 commit 與日期
