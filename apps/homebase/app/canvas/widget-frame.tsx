'use client';
import type { CSSProperties } from 'react';
import { Grip, MoveLeft, MoveRight, X } from 'lucide-react';
import {
  widgetCatalog,
  resizeWidget,
  type WidgetLayout,
} from '@/lib/canvas-layout';
import { widgetIcons } from './canvas-icons';
import { CanvasWidgetContent } from './canvas-widgets';
import type { CanvasController } from './use-canvas-controller';
export function WidgetFrame({
  widget,
  index,
  model,
}: {
  widget: WidgetLayout;
  index: number;
  model: CanvasController;
}) {
  const {
    layout,
    focusedWidgets,
    editing,
    attention,
    board,
    weatherCard,
    drag,
    setDrag,
    change,
    move,
    remove,
  } = model;
  const expanded =
    (widget.id === 'weather' && attention) ||
    focusedWidgets.includes(widget.id);
  const Icon = widgetIcons[widget.id];
  return (
    <section
      key={widget.id}
      ref={widget.id === 'weather' ? weatherCard : undefined}
      data-widget={widget.id}
      className={`canvas-widget widget-${widget.id} ${expanded ? 'canvas-attention' : ''}`}
      data-height={expanded ? 4 : widget.height}
      style={
        {
          '--widget-width': expanded ? Math.max(6, widget.width) : widget.width,
          '--widget-height': expanded ? 4 : widget.height,
        } as CSSProperties
      }
      aria-label={`${widget.id} widget`}
    >
      <header className="canvas-widget-header">
        <h2>
          <Icon size={17} />
          {widgetCatalog[widget.id].title}
        </h2>
        {editing && (
          <div className="canvas-widget-tools">
            <button
              disabled={index === 0}
              onClick={() => move(widget.id, -1)}
              aria-label={`Move ${widget.id} earlier`}
            >
              <MoveLeft size={14} />
            </button>
            <button
              disabled={index === layout.length - 1}
              onClick={() => move(widget.id, 1)}
              aria-label={`Move ${widget.id} later`}
            >
              <MoveRight size={14} />
            </button>
            <details>
              <summary aria-label={`Size ${widget.id} widget`}>
                <Grip size={16} />
              </summary>
              <div className="canvas-size-menu">
                <strong>Width</strong>
                {([3, 6, 12] as const).map((width, i) => (
                  <button
                    key={width}
                    aria-pressed={widget.width === width}
                    onClick={() => change(widget.id, { width })}
                  >
                    {['Compact', 'Wide', 'Full'][i]}
                  </button>
                ))}
                <strong>Height</strong>
                {([2, 3, 4] as const).map((height, i) => (
                  <button
                    key={height}
                    aria-pressed={widget.height === height}
                    onClick={() => change(widget.id, { height })}
                  >
                    {['Short', 'Medium', 'Tall'][i]}
                  </button>
                ))}
              </div>
            </details>
            <button
              onClick={() => remove(widget.id)}
              aria-label={`Remove ${widgetCatalog[widget.id].title}`}
            >
              <X size={14} />
            </button>
          </div>
        )}
      </header>
      <div className="canvas-widget-body">
        {<CanvasWidgetContent id={widget.id} model={model} />}
      </div>
      {editing && (
        <button
          className="canvas-resize"
          aria-label={`Resize ${widget.id}: use arrow keys or drag`}
          title="Drag to resize · arrow keys also work"
          onPointerDown={(e) => {
            if (e.button !== 0) return;
            e.currentTarget.setPointerCapture(e.pointerId);
            setDrag({
              id: widget.id,
              x: e.clientX,
              y: e.clientY,
              original: {
                ...widget,
                width: expanded
                  ? (Math.max(6, widget.width) as WidgetLayout['width'])
                  : widget.width,
                height: expanded ? 4 : widget.height,
              },
              gridWidth: board.current?.clientWidth || 1000,
            });
            if (expanded && drag.current)
              change(widget.id, drag.current.original);
          }}
          onPointerMove={(e) => {
            const start = drag.current;
            if (
              !start ||
              start.id !== widget.id ||
              !e.currentTarget.hasPointerCapture(e.pointerId)
            )
              return;
            const next = resizeWidget(
              start.original,
              e.clientX - start.x,
              e.clientY - start.y,
              start.gridWidth,
            );
            change(widget.id, next);
          }}
          onPointerUp={(e) => {
            setDrag(null);
            if (e.currentTarget.hasPointerCapture(e.pointerId))
              e.currentTarget.releasePointerCapture(e.pointerId);
          }}
          onPointerCancel={() => {
            const start = drag.current;
            if (start) change(start.id, start.original);
            setDrag(null);
          }}
          onLostPointerCapture={() => {
            setDrag(null);
          }}
          onKeyDown={(e) => {
            const offsets: Record<string, [number, number]> = {
              ArrowRight: [1, 0],
              ArrowLeft: [-1, 0],
              ArrowDown: [0, 1],
              ArrowUp: [0, -1],
            };
            const offset = offsets[e.key];
            if (offset) {
              e.preventDefault();
              change(
                widget.id,
                resizeWidget(
                  widget,
                  offset[0] *
                    Math.max(80, (board.current?.clientWidth || 1000) / 4),
                  offset[1] * 120,
                  board.current?.clientWidth || 1000,
                ),
              );
            }
          }}
        >
          <Grip size={16} />
        </button>
      )}
    </section>
  );
}
