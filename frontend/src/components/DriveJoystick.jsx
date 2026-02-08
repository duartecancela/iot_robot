import { useEffect, useMemo, useRef, useState } from 'react'
import { io } from 'socket.io-client'

function clamp(v, min, max) {
  return Math.max(min, Math.min(max, v))
}

function round2(v) {
  return Math.round(v * 100) / 100
}

// Uses VITE_BACKEND_URL if defined, otherwise defaults to localhost:3001
const BACKEND_URL = import.meta.env.VITE_BACKEND_URL || 'http://localhost:3001'

export default function DriveJoystick() {
  const baseRef = useRef(null)

  // ----------------------------
  // Socket.IO (added)
  // ----------------------------
  const socketRef = useRef(null)
  const lastSentRef = useRef({ left: 0, right: 0, ts: 0 })

  useEffect(() => {
    // Connect once
    const socket = io(BACKEND_URL, {
      transports: ['websocket'],
    })
    socketRef.current = socket

    socket.on('connect', () => {
      // Optional: console log
      console.log('[WS] connected', socket.id)
    })

    socket.on('disconnect', () => {
      console.log('[WS] disconnected')
    })

    // Optional: listen to backend confirmation
    socket.on('cmd:drive:sent', (msg) => {
      // msg: { ok, left, right, payload, ts }
      // console.log('[WS] cmd:drive:sent', msg)
    })

    return () => {
      socket.disconnect()
      socketRef.current = null
    }
  }, [])

  // Normalized: x,y in [-1..1] (y up = forward)
  const [pos, setPos] = useState({ x: 0, y: 0 })

  function updateFromClientPoint(clientX, clientY) {
    const el = baseRef.current
    if (!el) return

    const r = el.getBoundingClientRect()
    const cx = r.left + r.width / 2
    const cy = r.top + r.height / 2

    const dx = clientX - cx
    const dy = clientY - cy

    const radius = Math.min(r.width, r.height) / 2
    const max = radius * 0.78

    const dist = Math.hypot(dx, dy)
    const k = dist > max ? max / dist : 1

    const nx = (dx * k) / max
    const ny = (-dy * k) / max // up = forward

    setPos({ x: clamp(nx, -1, 1), y: clamp(ny, -1, 1) })
  }

  function onPointerDown(e) {
    e.currentTarget.setPointerCapture(e.pointerId)
    updateFromClientPoint(e.clientX, e.clientY)
  }

  function onPointerMove(e) {
    if (!e.currentTarget.hasPointerCapture(e.pointerId)) return
    updateFromClientPoint(e.clientX, e.clientY)
  }

  function onPointerUp(e) {
    if (e.currentTarget.hasPointerCapture(e.pointerId)) {
      e.currentTarget.releasePointerCapture(e.pointerId)
    }
    setPos({ x: 0, y: 0 })
  }

  const mapped = useMemo(() => {
    const leftN = clamp(pos.y + pos.x, -1, 1)
    const rightN = clamp(pos.y - pos.x, -1, 1)

    return {
      left: Math.round(leftN * 255),
      right: Math.round(rightN * 255),
    }
  }, [pos.x, pos.y])

  // ----------------------------
  // Send to backend (added)
  // ----------------------------
  useEffect(() => {
    const socket = socketRef.current
    if (!socket || !socket.connected) return

    const now = Date.now()
    const minIntervalMs = 50 // ~20 Hz (adjust if you want)

    const last = lastSentRef.current
    const changed = mapped.left !== last.left || mapped.right !== last.right
    const tooSoon = now - last.ts < minIntervalMs

    if (!changed || tooSoon) return

    socket.emit('cmd:drive', { left: mapped.left, right: mapped.right })
    lastSentRef.current = { left: mapped.left, right: mapped.right, ts: now }
  }, [mapped.left, mapped.right])

  const knobStyle = {
    transform: `translate(${pos.x * 58}px, ${-pos.y * 58}px)`,
  }

  return (
    <section className="mt-4">
      <div className="mb-2 flex items-center justify-between">
        <h2 className="text-sm font-semibold text-stone-900">Drive</h2>
        <button
          onClick={() => setPos({ x: 0, y: 0 })}
          className="rounded border border-stone-300 bg-stone-50 px-2 py-1 text-xs text-stone-800 hover:bg-stone-100"
        >
          Center
        </button>
      </div>

      <div className="rounded border border-stone-200 bg-stone-50/40 p-3">
        <div className="flex flex-col items-center">
          {/* Joystick */}
          <div
            ref={baseRef}
            onPointerDown={onPointerDown}
            onPointerMove={onPointerMove}
            onPointerUp={onPointerUp}
            onPointerCancel={onPointerUp}
            className="relative h-44 w-44 touch-none select-none rounded-full border border-stone-200 bg-stone-50"
            aria-label="Drive joystick"
            role="application"
          >
            {/* crosshair */}
            <div className="pointer-events-none absolute left-1/2 top-0 h-full w-px -translate-x-1/2 bg-stone-200/70" />
            <div className="pointer-events-none absolute top-1/2 left-0 h-px w-full -translate-y-1/2 bg-stone-200/70" />

            {/* knob */}
            <div
              className="absolute left-1/2 top-1/2 h-10 w-10 -translate-x-1/2 -translate-y-1/2"
              style={knobStyle}
            >
              <div className="h-full w-full rounded-full border border-stone-300 bg-white shadow-sm" />
              <div className="absolute left-1/2 top-1/2 h-1.5 w-1.5 -translate-x-1/2 -translate-y-1/2 rounded-full bg-stone-700" />
            </div>
          </div>

          {/* Values BELOW joystick */}
          <div className="mt-3 w-full max-w-xs space-y-2">
            <div className="rounded border border-stone-200 bg-white px-2 py-1.5 text-xs">
              <div className="text-[10px] uppercase tracking-wide text-stone-600">
                x / y
              </div>
              <div className="mt-0.5 font-mono text-stone-900">
                x: {round2(pos.x)} &nbsp; y: {round2(pos.y)}
              </div>
            </div>

            <div className="rounded border border-stone-200 bg-white px-2 py-1.5 text-xs">
              <div className="text-[10px] uppercase tracking-wide text-stone-600">
                motors (-255..255)
              </div>
              <div className="mt-0.5 font-mono text-stone-900">
                L: {mapped.left}
                <br />
                R: {mapped.right}
              </div>
            </div>

            <div className="text-center text-[10px] text-stone-500">
              Release joystick to return to center (0,0)
            </div>
          </div>
        </div>
      </div>
    </section>
  )
}
