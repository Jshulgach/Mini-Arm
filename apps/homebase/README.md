# Homebase has moved

The dashboard is now maintained in the sibling [Homebase repository](../../../Homebase/README.md):

`C:\Users\jonat\Documents\GitHub\Homebase`

Run Homebase commands from that repository. Mini-Arm retains robot firmware, ROS packages, CAD, and the Python SDK.

The local dashboard address remains http://127.0.0.1:5173/. The saved SQLite registry was migrated on September 9, 2026. Ignored runtime files in this old directory are retained as a rollback copy; they are no longer the active data store.

Homebase history was preserved with a subtree extraction. The Mini-Arm checkpoint is commit 630b837; the standalone extraction branch is codex/homebase-standalone. No remote publishing was performed.
