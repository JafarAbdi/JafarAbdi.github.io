---
title: Podman & Ubuntu
tags: podman, containers, virtualisation
---

I've always liked the idea of rootless containerization. I think it's even more crucial now that we have agents running all the time on our machines.

I learned about Podman before the LLM era from [Podman in Action](https://www.manning.com/books/podman-in-action), which I found useful. One of the most annoying things about Podman, though, is using it on Ubuntu. The version available through `apt` is outdated, so you have to build it from source if you want a newer release.

Well, now that we have AI, I decided to take a stab at it and see if I could get it to build easily so I could use it on my servers or as a sandbox for my agents.

Credit goes to [podman-static](https://github.com/mgoltzsche/podman-static) for open-sourcing his repo. My agent definitely took a lot of things from it.

I pushed it to my [dotfiles](https://github.com/JafarAbdi/myconfigs/tree/main/podman). Here is how to build and install it:

```bash
git clone https://github.com/JafarAbdi/myconfigs.git --depth 1
cd ./myconfigs/podman
just build
just install
```

And ta-da! You now have a fully static Podman build that you can copy to any machine, and it will work out of the box.

I need to write a blog post about how much I love musl and zigbuild and how much easier they make my life by making fully static binaries possible.
