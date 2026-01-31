# Frontend – IoT Robot Web Interface

Web-based frontend for the IoT Robot Platform.

This application provides a real-time interface to visualize robot telemetry and (in later stages) send control commands.  
It communicates with the backend using **Socket.IO** and follows a clean, component-based architecture.

---

## Tech Stack

- React
- Vite
- Tailwind CSS
- Socket.IO client

---

## Features

- Real-time telemetry visualization
- Clean layout with reusable components
- Responsive UI using Tailwind CSS
- Environment-based configuration (Vite)

---

## Requirements

- Node.js >= 18
- Backend server running (see `backend/README.md`)

---

## Configuration

The frontend uses **Vite environment variables**.

Create a `.env` file based on the example:

```bash
cp .env.example .env
```

### Environment variables

| Variable | Description | Example |
|--------|------------|--------|
| `VITE_BACKEND_URL` | Backend base URL | `http://localhost:3001` |

> ⚠️ All frontend environment variables **must start with `VITE_`**.

The `.env` file is not committed.  
Default values are documented in `.env.example`.

---

## Setup

Install dependencies:

```bash
npm install
```

---

## Run (Development)

Start the development server:

```bash
npm run dev
```

Vite will print the local URL, usually:

```
http://localhost:5173
```

---

## Project Structure

```
frontend/
├─ src/
│  ├─ components/     # Reusable UI components (Header, Footer, etc.)
│  ├─ App.jsx         # Main application layout
│  ├─ main.jsx        # Application entry point
│  └─ index.css       # Tailwind CSS entry
├─ public/
├─ .env.example
├─ vite.config.js
└─ README.md
```

---

## Styling

Styling is handled with **Tailwind CSS** using the official Vite plugin.

- No custom CSS framework
- Utility-first approach
- Consistent layout across components

Tailwind is enabled via:

```css
@import "tailwindcss";
```

---

## Backend Communication

- Protocol: **Socket.IO**
- Backend URL: `VITE_BACKEND_URL`
- Event: `telemetry`

On connection, the backend immediately sends the last known telemetry message.

---

## Development Notes

- The frontend does **not** read shared configuration files directly
- All configuration is injected at build time via `.env`
- Backend and simulator handle shared defaults

This separation keeps the frontend secure and portable.

---

## Next Steps

- Telemetry dashboard (cards, charts)
- Robot control panel
- Connection status indicator
- Error handling and reconnection logic

---

## License

This project is intended for educational and experimental purposes.
