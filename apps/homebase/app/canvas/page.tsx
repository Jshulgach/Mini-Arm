'use client';
import Link from 'next/link';
import {
  ArrowLeft,
  ArrowRight,
  House,
  RotateCcw,
  Settings2,
  Plus,
  X,
} from 'lucide-react';
import { Dialog as LibraryDialog } from '@base-ui/react/dialog';
import { defaultLayout, widgetCatalog, widgetIds } from '@/lib/canvas-layout';
import { defaultWorkspaces, presetIds } from '@/lib/canvas-workspaces';
import type { Preferences } from '@/contracts/assistant';
import { widgetIcons } from './canvas-icons';
import { useCanvasController } from './use-canvas-controller';
import { WidgetFrame } from './widget-frame';
import './canvas.css';
export default function Canvas() {
  const model = useCanvasController();
  const {
    workspaces,
    setWorkspaces,
    activeWorkspace,
    layout,
    setLayout,
    workspaceEditor,
    setWorkspaceEditor,
    workspaceName,
    setWorkspaceName,
    focusedWidgets,
    setFocusedWidgets,
    setFocusedDeviceIds,
    preferences,
    setPreferences,
    editing,
    setEditing,
    ready,
    storageError,
    status,
    setAttention,
    busy,
    settings,
    setSettings,
    libraryOpen,
    setLibraryOpen,
    libraryNotice,
    setLibraryNotice,
    home,
    homeError,
    board,
    setDrag,
    leaveWorkspace,
    switchWorkspace,
    saveWorkspace,
    add,
  } = model;

  return (
    <main
      className={`adaptive-canvas ${editing ? 'canvas-editing' : 'canvas-viewing'}`}
      aria-busy={!ready}
      inert={!ready}
    >
      <header className="canvas-top">
        <Link href="/" className="canvas-brand">
          <House size={24} /> HOMEBASE <span>CANVAS</span>
        </Link>
        <nav>
          <Link href="/playground">Virtual home</Link>
          <Link href="/systems">Systems</Link>
          <Link href="/">
            <ArrowLeft size={14} /> Overview
          </Link>
        </nav>
      </header>
      <div className="canvas-heading">
        <div>
          <p>YOUR HOME, IN CONTEXT</p>
          <h1>
            Your home, at a glance<span>.</span>
          </h1>
        </div>
        <div>
          <button
            aria-pressed={editing}
            onClick={() => {
              setEditing((value) => !value);
              setWorkspaceEditor(false);
              setLibraryOpen(false);
              setDrag(null);
            }}
          >
            <Settings2 size={16} /> {editing ? 'Done editing' : 'Edit canvas'}
          </button>
          {editing && (
            <button
              className="canvas-add-widget"
              onClick={() => {
                setLibraryNotice('');
                setLibraryOpen(true);
              }}
            >
              <Plus size={17} /> Add widget
            </button>
          )}
          <button
            aria-expanded={settings}
            onClick={() => setSettings(!settings)}
          >
            <Settings2 size={16} /> Preferences
          </button>
          {editing && (
            <button
              onClick={() => {
                setLayout(
                  (
                    defaultWorkspaces().items.find(
                      (w) => w.id === workspaces.active,
                    )?.layout ?? defaultLayout
                  ).map((v) => ({ ...v })),
                );
                setFocusedWidgets([]);
                setFocusedDeviceIds(null);
                setAttention(false);
              }}
            >
              <RotateCcw size={16} /> Reset layout
            </button>
          )}
        </div>
      </div>
      <section className="canvas-workspaces" aria-label="Saved workspaces">
        <nav className="canvas-workspace-tabs" aria-label="Choose workspace">
          {workspaces.items.map((workspace) => (
            <button
              key={workspace.id}
              aria-pressed={workspace.id === workspaces.active}
              onClick={() => switchWorkspace(workspace.id)}
            >
              {workspace.name}
            </button>
          ))}
        </nav>
        {editing && (
          <button
            aria-expanded={workspaceEditor}
            onClick={() => {
              setWorkspaceName(activeWorkspace.name);
              setWorkspaceEditor((v) => !v);
            }}
          >
            Manage workspace
          </button>
        )}
        {editing && <small>Saved automatically in this browser</small>}
      </section>
      {editing && workspaceEditor && (
        <form
          className="canvas-workspace-editor"
          onSubmit={(event) => {
            event.preventDefault();
            saveWorkspace(true);
          }}
        >
          <label>
            Workspace name
            <input
              value={workspaceName}
              maxLength={30}
              onChange={(e) => setWorkspaceName(e.target.value)}
            />
          </label>
          <button
            type="submit"
            disabled={!workspaceName.trim() || workspaces.items.length >= 8}
          >
            Save as new workspace
          </button>
          <button
            type="button"
            disabled={!workspaceName.trim()}
            onClick={() => saveWorkspace(false)}
          >
            Rename current
          </button>
          <button
            type="button"
            disabled={presetIds.includes(workspaces.active)}
            onClick={() => {
              const id = workspaces.active;
              leaveWorkspace();
              setWorkspaces((state) => ({
                ...state,
                active: 'overview',
                items: state.items.filter((w) => w.id !== id),
              }));
            }}
          >
            Delete current
          </button>
          <p>
            Up to eight workspaces. Overview, Cameras and Workshop can be
            renamed but cannot be deleted. Switching clears this chat session
            and stops camera previews.
          </p>
        </form>
      )}
      {focusedWidgets.length > 0 && (
        <output className="canvas-focus-bar">
          Showing related widgets{' '}
          <button
            onClick={() => {
              setFocusedWidgets([]);
              setFocusedDeviceIds(null);
            }}
          >
            Return to my layout
          </button>
        </output>
      )}
      {!editing && (
        <section className="canvas-now-summary" aria-label="Home summary">
          <span>
            <House size={16} />{' '}
            {home
              ? `${home.devices.length} registered devices`
              : 'Loading home registry…'}
          </span>
          {home && (
            <span>
              {home.devices.filter((device) => device.monitor).length} monitored
              · {home.devices.filter((device) => !device.monitor).length}{' '}
              monitoring off
            </span>
          )}
          <Link href="/systems">
            {homeError
              ? 'Gateway unavailable · view last snapshot'
              : 'Review home status'}{' '}
            <ArrowRight size={14} />
          </Link>
        </section>
      )}
      {editing && (
        <p className="canvas-instructions">
          Drag a card’s corner to resize, or use its size buttons. Ask about
          your home to bring relevant widgets into focus.
        </p>
      )}
      {settings && (
        <section className="canvas-preferences">
          <label>
            Your first name{' '}
            <input
              maxLength={40}
              value={preferences.name}
              onChange={(e) =>
                setPreferences((p) => ({ ...p, name: e.target.value }))
              }
            />
          </label>
          <label>
            Temperature{' '}
            <select
              disabled={busy}
              value={preferences.units}
              onChange={(e) =>
                setPreferences((p) => ({
                  ...p,
                  units: e.target.value as Preferences['units'],
                }))
              }
            >
              <option value="fahrenheit">Fahrenheit</option>
              <option value="celsius">Celsius</option>
            </select>
          </label>
          <p>
            Home city: {status?.city || 'Pittsburgh'}. Weather uses city-level
            coordinates. If enabled, AI receives your question, recent chat,
            first name and forecast. No cameras or device controls are connected
            to chat.
          </p>
          {!status?.aiReady && (
            <p>
              General AI replies need OPENAI_API_KEY and OPENAI_MODEL in the
              service’s .env.local file. Forecast questions work without them.
            </p>
          )}
        </section>
      )}
      {storageError && <output>{storageError}</output>}
      {!libraryOpen && libraryNotice && (
        <output className="canvas-library-notice">{libraryNotice}</output>
      )}
      {!layout.length && (
        <section className="canvas-empty-board">
          <Plus size={30} />
          <h2>Your canvas, your choice.</h2>
          <p>
            Add a widget to begin. Your home devices and settings are unchanged.
          </p>
          <button onClick={() => setLibraryOpen(true)}>
            Add your first widget
          </button>
        </section>
      )}
      <LibraryDialog.Root open={libraryOpen} onOpenChange={setLibraryOpen}>
        <LibraryDialog.Portal>
          <LibraryDialog.Backdrop className="canvas-library-backdrop" />
          <LibraryDialog.Popup className="canvas-library-dialog">
            <header>
              <div>
                <LibraryDialog.Title>Widget library</LibraryDialog.Title>
                <LibraryDialog.Description>
                  Choose what belongs on your canvas. Resize and arrange it
                  afterward.
                </LibraryDialog.Description>
              </div>
              <LibraryDialog.Close aria-label="Close widget library">
                <X size={22} />
              </LibraryDialog.Close>
            </header>
            <div className="canvas-library-grid">
              {widgetIds.map((id) => {
                const item = widgetCatalog[id],
                  Icon = widgetIcons[id],
                  added = layout.some((widget) => widget.id === id);
                return (
                  <article key={id}>
                    <Icon size={24} />
                    <span>{item.category}</span>
                    <h3>{item.title}</h3>
                    <p>{item.description}</p>
                    <button
                      disabled={added}
                      onClick={() => add(id)}
                      aria-label={
                        added
                          ? `${item.title} already added`
                          : `Add ${item.title}`
                      }
                    >
                      {added ? (
                        'Added to canvas'
                      ) : (
                        <>
                          <Plus size={15} /> Add widget
                        </>
                      )}
                    </button>
                  </article>
                );
              })}
            </div>
            <output aria-live="polite">
              {libraryNotice ||
                'Camera previews stay stopped until you start them.'}
            </output>
            <LibraryDialog.Close className="canvas-library-done">
              Done
            </LibraryDialog.Close>
          </LibraryDialog.Popup>
        </LibraryDialog.Portal>
      </LibraryDialog.Root>
      <div className="canvas-board" ref={board} key={workspaces.active}>
        {layout.map((widget, index) => (
          <WidgetFrame
            key={widget.id}
            widget={widget}
            index={index}
            model={model}
          />
        ))}
      </div>
      <footer className="canvas-footnote">
        Layout and preferences save in this browser. Forecast answers use
        provider data; AI is optional. No device actions.
      </footer>
    </main>
  );
}
