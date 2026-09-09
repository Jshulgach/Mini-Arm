import { previewPlan, type Device, type Plan } from './home';
export type HomeTool = {
  name: string;
  description: string;
  inputSchema: object;
  annotations: { readOnlyHint: boolean };
  execute: (input: unknown) => unknown;
};
export interface HomeModelContext {
  registerTool: (
    tool: HomeTool,
    options: { signal: AbortSignal },
  ) => void | Promise<void>;
}
export function registerHomeTools(
  context: HomeModelContext,
  inventory: () => Device[],
  stage: (command: string, plan: Plan) => void,
) {
  const lifecycle = new AbortController();
  const tools: HomeTool[] = [
    {
      name: 'read_home_inventory',
      description:
        'Read configured inventory labels and rooms. All live states are unknown; no network discovery is performed.',
      inputSchema: {
        type: 'object',
        properties: {},
        additionalProperties: false,
      },
      annotations: { readOnlyHint: true },
      execute: () =>
        inventory().map(({ id, name, room }) => ({
          id,
          name,
          room,
          connection: 'not_connected',
          observedAt: null,
        })),
    },
    {
      name: 'stage_home_task_preview',
      description:
        'Display a template task preview and its blockers. Does not call a model, send a device command or execute a task.',
      inputSchema: {
        type: 'object',
        properties: {
          command: { type: 'string', minLength: 1, maxLength: 500 },
        },
        required: ['command'],
        additionalProperties: false,
      },
      annotations: { readOnlyHint: false },
      execute: (input) => {
        if (!input || typeof input !== 'object' || Array.isArray(input))
          throw new Error('Expected a command object.');
        const values = input as Record<string, unknown>;
        if (
          Object.keys(values).some((k) => k !== 'command') ||
          typeof values.command !== 'string' ||
          !values.command.trim() ||
          values.command.length > 500
        )
          throw new Error('Expected a command between 1 and 500 characters.');
        const plan = previewPlan(values.command);
        stage(values.command, plan);
        return { status: 'preview_only', ...plan, executed: false };
      },
    },
  ];
  for (const tool of tools) {
    try {
      void Promise.resolve(
        context.registerTool(tool, { signal: lifecycle.signal }),
      ).catch(() => {});
    } catch {
      /* Optional browser API; visible controls remain available. */
    }
  }
  return () => lifecycle.abort();
}
