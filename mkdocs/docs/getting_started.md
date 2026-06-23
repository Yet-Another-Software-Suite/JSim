# Getting Started

## Prerequisites

- WPILib 2026 installed
- A GradleRIO robot project (Java)
- Java 17

---

## Installation

### Option A — VS Code WPILib extension

1. Open the command palette (`Ctrl+Shift+P` / `Cmd+Shift+P`)
2. Run **WPILib: Manage Vendor Libraries** → **Install new libraries (online)**
3. Paste the vendordep URL:

```
https://jsim.dev/JSim.json
```

4. Run **WPILib: Build Robot Code** to download and verify the dependency.

### Option B — manual vendordep JSON

Copy the following into your project's `vendordeps/JSim.json`:

```json
{
  "fileName": "JSim.json",
  "name": "JSim",
  "version": "<latest>",
  "frcYear": "2026",
  "uuid": "a87eb0a9-efa5-4de6-9bb0-9ca5565d20c1",
  "mavenUrls": ["https://jsim.dev/maven/releases/"],
  "jsonUrl": "https://jsim.dev/JSim.json",
  "javaDependencies": [
    {
      "groupId": "jsim",
      "artifactId": "JSim-java",
      "version": "<latest>"
    }
  ],
  "jniDependencies": [],
  "cppDependencies": []
}
```

Replace `<latest>` with the current version shown at [jsim.dev/JSim.json](https://jsim.dev/JSim.json).

---

## Running the example project

Clone the repo and open `examples/java/simple_robot/` in VS Code. It includes JSim pre-configured as a local vendordep. Build and run the desktop simulation:

```bash
./gradlew simulatejava
```

---

## Your first simulation

See [API Usage](api_usage.md) for a step-by-step integration guide.
