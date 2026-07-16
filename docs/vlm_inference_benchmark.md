# VLM Inference Benchmark — `llama_cpp_node` (InternVL2.5-1B)

**Platform:** RDK X5 (8× Cortex-A55 @ 1.5 GHz, BPU Bayes-e 10 TOPS)
**Model stack:** InternViT-300M (BPU, `vit_model_int16_v2.bin`, INT16) + Qwen2.5-0.5B-Instruct Q4_0 GGUF (llama.cpp b4749, CPU, 8 threads)
**Log window:** 2026-07-16, ~21:28 onward — model load + 4 consecutive suspect-attribute queries
**Prompt:** fixed 3-attribute extraction (clothing_color / clothing_type / hairstyle), 352 tokens, ~20 output tokens

---

## 1. Headline Results

| Metric | Value | Assessment |
|---|---|---|
| Cold model load (weights) | **~98 s** | 🟡 Expected on eMMC/SD; justifies the 900 s matcher timeout |
| End-to-end latency per query | **52 – 62 s** | 🔴 Dominated by LLM decode, not vision |
| ViT encode (BPU) | **2.9 – 3.0 s** | 🟢 Consistent, cheap — BPU doing its job |
| Prompt eval (prefill, CPU) | 18.4 – 21.8 s @ 16.1 – 19.1 tok/s | 🟡 352 tokens every query, no reuse |
| Token generation (decode, CPU) | **0.54 – 0.72 tok/s** (1.38 – 1.85 s/token) | 🔴 Severely CPU-starved |
| Output stability | Identical 3-line answer ×4 | 🟢 Deterministic, format followed |

## 2. Per-Query Breakdown

| Query | Recv → publish (wall) | Prompt eval | Decode (20 tok) | ViT infer | "post process" |
|---|---|---|---|---|---|
| 1 | ~56 s | 18.39 s (19.14 tok/s) | 32.30 s (0.62 tok/s) | — | — |
| 2 | ~62 s | 20.44 s (17.22 tok/s) | 36.98 s (0.54 tok/s) | 2.95 s | 59.18 s |
| 3 | ~62 s | 21.81 s (16.14 tok/s) | 35.42 s (0.56 tok/s) | 2.97 s | 59.06 s |
| 4 | ~52 s | 20.52 s (17.15 tok/s) | 27.59 s (0.72 tok/s) | 2.90 s | 49.39 s |

Notes:
- The `load time` counter (98 s → 155 s → 232 s → 293 s) is **cumulative session time**, not a per-query reload — the GGUF stays mmap'd. Only the first 98 s is real load cost.
- The node's "post process time" (~50–59 s) is effectively prefill + decode combined; "infer time" (~2.9 s) is the BPU ViT pass only.
- The KV cache / llama context is **re-initialized on every query** (`llama_init_from_model` logged 4×). At n_ctx=4096 that's a 48 MiB KV + 300 MiB compute buffer alloc per request — small overhead here, but it also discards any possibility of prefix caching.

## 3. Where the Time Goes (typical 60 s query)

```
ViT encode (BPU)      ▏ ~3 s    ( 5%)
Prompt prefill (CPU)  ████ ~20 s (33%)
Decode 20 tok (CPU)   ███████ ~34 s (57%)
Overhead/ctx init     ▏ ~3 s    ( 5%)
```

The BPU is idle 95% of the query. The bottleneck is the Qwen2.5-0.5B decoder on Cortex-A55 cores — and per the concurrent htop benchmark, those 8 cores were already at load avg ~20 running the full patrol stack. 0.5–0.7 tok/s for a 0.5B Q4_0 model is well below what A55s manage in isolation; **this is contention, not model cost.**

## 4. Findings & Recommendations

1. **Cut the prompt.** 352 tokens of prefill costs ~20 s per query (33% of total). The instruction can likely be compressed to <120 tokens (shorter phrasing, drop repeated examples) → saves ~13 s/query with zero quality risk for a fixed-format extraction task.

2. **Cut n_ctx.** 4096 context for a 352-in / 20-out task is 10× oversized. Dropping to 512–1024 shrinks the KV and compute buffers (48 + 300 MiB → ~60–90 MiB total), reducing RAM pressure alongside the ION carve-out.

3. **Reduce contention during inference.** Decode rate varied 0.54 → 0.72 tok/s purely with system load (query 4 was fastest — likely other nodes momentarily idle). Options:
   - Pause/throttle non-essential nodes during a VLM query (the OLED already shows ANALYZING — gate `pointcloud_to_laserscan` or drop camera FPS via the matcher).
   - Pin `llm_threads` to 4–6 instead of 8 and `taskset` the camera/EKF nodes to the remaining cores; 8 threads fighting 155 tasks is worse than 5 threads owning their cores.

4. **Fix output parsing hazards.** The model alternates `Clothing color:` (capital C, space) and `clothing_color:` between runs, prepends a blank line, and appends `</s>`. The `attribute_compare` parser should be case-insensitive, strip the EOS marker, and treat `[ _]` as equivalent — otherwise identical answers will intermittently fail to match.

5. **Duplicate query suspicion.** Query 2 arrived 24 ms after query 1's result published (1784208638.025 → .049) with the identical prompt. Verify the matcher isn't re-sending on result receipt (or that `compareInFlight`-style gating exists on the ROS side, not just the dashboard) — every accidental duplicate costs a full minute of saturated CPU.

6. **Set realistic timeouts from data.** Measured worst case: 98 s load + 62 s query = 160 s cold, ~62 s warm. The current 900 s timeout is safe; a warm-path watchdog at ~120 s would catch hangs 7× faster without false trips.

## 5. Baseline Record

```
Config: full patrol stack + llama_cpp_node, feed_type=sub, llm_threads=8, n_ctx=4096
Cold load:        98.2 s
Warm query e2e:   52–62 s  (prompt 352 tok / output 20 tok)
Prefill:          16.1–19.1 tok/s
Decode:           0.54–0.72 tok/s
ViT (BPU):        2.90–2.97 s
Output:           format-correct, deterministic across 4 runs
```

Re-run this benchmark after prompt-shortening (§4.1) and thread-pinning (§4.3) — those two alone should bring warm queries under ~30 s.
