---
title: Muse Glimmer 30B
---

Meta released the weights for its Muse Glimmer model, following up on Muse Spark 1.2. One of my hobbies these days is trying out different models and wasting electricity and a few brain cells on them.

The server configuration below follows Meta's [Muse Glimmer 30B model card](https://huggingface.co/meta-models/Muse-Glimmer-30B-GGUF).

```bash
llama serve \
  -hf meta-models/Muse-Glimmer-30B-GGUF:17gb \
  --spec-type draft-dflash \
  --spec-draft-n-max 15 \
  -ngl all -ngld all \
  -c 131072 -np 4 \
  -fa on --fit on --jinja \
  --temp 1.0 --top-p 0.95 --top-k 64 --min-p 0 \
  --host 0.0.0.0 --alias muse-glimmer --kv-unified --reasoning-preserve
```

```bash
llama serve \
  -hf meta-models/Muse-Glimmer-30B-GGUF:dynamic \
  --spec-type draft-dflash \
  --spec-draft-n-max 15 \
  -ngl all -ngld all \
  -c 131072 -np 4 \
  -fa on --fit on --jinja \
  --temp 1.0 --top-p 0.95 --top-k 64 --min-p 0 \
  --host 0.0.0.0 --alias muse-glimmer --kv-unified --reasoning-preserve
```

## Performance

- `17gb`: 57.95 tokens/s, using about 20 GB of VRAM with a 128K-token context.
- `dynamic`: 56.84 tokens/s, using about 23 GB of VRAM with a 128K-token context.

In initial testing, Muse Glimmer is noticeably faster than Gemma 4, uses less memory, and comfortably handles four agents in parallel. I also did not notice much difference between the `17gb` and `dynamic` variants. The former may be slightly faster, but I could not tell in practice.

I will try Muse Glimmer for a few days to see whether it can replace Gemma 4 and Qwen 3.6 in my workflow. For reference, here is my current Gemma 4 configuration:

```bash
llama serve \
  -hf ggml-org/gemma-4-31B-it-GGUF:Q4_0 \
  --spec-type draft-mtp \
  --spec-draft-n-max 4 \
  -ngl all -ngld all \
  -c 65536 -np 2 \
  -fa on --fit on --jinja \
  --temp 1.0 --top-k 64 \
  --reasoning on \
  --chat-template-kwargs '{"preserve_thinking":true}' \
  --host 0.0.0.0 \
  --port 8080 \
  --alias gemma4 \
  --kv-unified
```
