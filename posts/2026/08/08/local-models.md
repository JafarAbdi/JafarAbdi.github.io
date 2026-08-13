---
title: Local Inference Models
tags: llms, local-inference
---

One of the most exciting thing in AI is not just that powerful models can solve a fair number of tasks in a single attempt. It is that more of these models can now run locally.

Here are some local models I currently use:

- [Whisper](https://github.com/ggml-org/whisper.cpp/): I use it with my personal assistant when I cannot be bothered to type. It generally works very well.
- [GLM-OCR](https://huggingface.co/ggml-org/GLM-OCR-GGUF): This is an excellent model for turning screenshots of equations or text from PDFs and images into copyable text. Here is my [script](https://github.com/JafarAbdi/myconfigs/blob/main/scripts/.local/bin/eq2latex).
- [Qwen3.6](https://huggingface.co/unsloth/Qwen3.6-27B-MTP-GGUF): I used it for coding, but these days I prefer Gemma4, especially after the latest fixes to the chat template.
- [Gemma4](https://huggingface.co/ggml-org/gemma-4-31B-it-GGUF): I use it for my personal assistant and simple coding tasks.

Today, I experimented with [MiniMax-H3](https://www.minimax.io/blog/minimax-h3). I asked Fable, using its `xhigh` effort setting, to write an optimized inference script. MiniMax-H3 fits easily in the VRAM of my RTX 3090 and takes about 22 minutes to generate a 124-frame video at 864x480 resolution.

I am looking forward to trying Qwen3.8 when it is released.
