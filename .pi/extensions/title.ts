/**
 * /title <file> - suggest three titles and filenames for a post.
 */

import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import type { Tool } from "@earendil-works/pi-ai";
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { Type } from "typebox";

const SYSTEM_PROMPT = `You name posts for a small personal blog. Sound like a person, not a content-marketing generator.

Title the post's actual point, not its broad subject. Prefer a memorable phrase the author already used when it captures that point. Keep the author's bluntness, humor, uncertainty, and technical vocabulary.

Avoid:
- generic category labels such as "LLM failure modes"
- SEO copy, clickbait, grand claims, or forced cleverness
- formulas such as "Why X matters", "The future of X", "Thoughts on X", or "X: A guide"
- claims the post does not support
- copying a title from YAML front matter; it may be stale or a placeholder

For example, prefer "You're right — my bad" over "LLM failure modes", and "I don't want smarter models" over "Thoughts on long-horizon agents".

Return three genuinely different options, strongest first. Keep each title under 60 characters. Derive each slug directly from its title using 2-6 meaningful words in the same order. Call suggest_titles exactly once and return no prose.`;

const suggestTitles: Tool = {
	name: "suggest_titles",
	description: "Return three plain, specific title and filename options for the post",
	parameters: Type.Object(
		{
			suggestions: Type.Array(
				Type.Object(
					{
						title: Type.String({ minLength: 2, maxLength: 60 }),
						slug: Type.String({
							minLength: 3,
							maxLength: 60,
							pattern: "^[a-z0-9]+(-[a-z0-9]+)*$",
						}),
					},
					{ additionalProperties: false },
				),
				{ minItems: 3, maxItems: 3 },
			),
		},
		{ additionalProperties: false },
	),
	constrainedSampling: { type: "json_schema", strict: "prefer" },
};

export default function (pi: ExtensionAPI) {
	pi.registerCommand("title", {
		description: "Suggest titles + filenames for a post (usage: /title <file>)",
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

			ctx.ui.notify("Suggesting titles...", "info");
			const response = await ctx.modelRegistry.complete(
				model,
				{
					systemPrompt: SYSTEM_PROMPT,
					messages: [
						{
							role: "user",
							content: [{ type: "text", text: `<post>\n${post}\n</post>` }],
							timestamp: Date.now(),
						},
					],
					tools: [suggestTitles],
				},
				{ reasoningEffort: "medium", cacheRetention: "none" },
			);

			const call = response.content.find((block) => block.type === "toolCall");
			if (!call || call.type !== "toolCall") {
				ctx.ui.notify("Model returned no suggestion", "error");
				return;
			}

			const { suggestions } = call.arguments as {
				suggestions: Array<{ title: string; slug: string }>;
			};
			const date = new Date().toISOString().slice(0, 10).replaceAll("-", "/");
			const options = suggestions
				.map(({ title, slug }) => `${title} -> posts/${date}/${slug}.md`)
				.join("\n");
			ctx.ui.notify(options, "info");
		},
	});
}
