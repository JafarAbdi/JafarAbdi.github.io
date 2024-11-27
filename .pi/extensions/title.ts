/**
 * /title <file> - suggest a title and filename for a post.
 * Uses constrained sampling so the model must return {title, slug}.
 */

import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import type { Tool } from "@earendil-works/pi-ai";
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { Type } from "typebox";

const suggestTitle: Tool = {
	name: "suggest_title",
	description: "Return the final title and slug for the post",
	parameters: Type.Object(
		{
			title: Type.String({ description: "Post title, max 60 chars, sentence case" }),
			slug: Type.String({
				description: "Filename slug, 2-5 words",
				pattern: "^[a-z0-9]+(-[a-z0-9]+)*$",
			}),
		},
		{ additionalProperties: false },
	),
	constrainedSampling: { type: "json_schema", strict: "prefer" },
};

export default function (pi: ExtensionAPI) {
	pi.registerCommand("title", {
		description: "Suggest title + filename for a post (usage: /title <file>)",
		handler: async (args, ctx) => {
			const file = args.trim().replace(/^@/, "");
			if (!file) {
				ctx.ui.notify("Usage: /title <file>", "warning");
				return;
			}

			let post: string;
			try {
				post = readFileSync(resolve(ctx.cwd, file), "utf-8");
			} catch {
				ctx.ui.notify(`Cannot read ${file}`, "error");
				return;
			}

			const model = ctx.modelRegistry.find("openai-codex", "gpt-5.6-luna");
			if (!model) {
				ctx.ui.notify("Model openai-codex/gpt-5.6-luna not found", "error");
				return;
			}
			if (!ctx.modelRegistry.hasConfiguredAuth(model)) {
				ctx.ui.notify("No auth configured for openai-codex", "error");
				return;
			}

			ctx.ui.notify("Suggesting title...", "info");
			const response = await ctx.modelRegistry.complete(
				model,
				{
					messages: [
						{
							role: "user",
							content: [
								{
									type: "text",
									text: `Call suggest_title exactly once for this blog post. No clickbait.\n\n${post}`,
								},
							],
							timestamp: Date.now(),
						},
					],
					tools: [suggestTitle],
				},
				{ reasoningEffort: "medium", cacheRetention: "none" },
			);

			const call = response.content.find((block) => block.type === "toolCall");
			if (!call || call.type !== "toolCall") {
				ctx.ui.notify("Model returned no suggestion", "error");
				return;
			}

			const { title, slug } = call.arguments as { title: string; slug: string };
			const date = new Date().toISOString().slice(0, 10).replaceAll("-", "/");
			ctx.ui.notify(`${title} -> posts/${date}/${slug}.md`, "info");
		},
	});
}
